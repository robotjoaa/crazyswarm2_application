#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import os
from functools import partial
import yaml

from typing import Dict, List, Tuple
from dataclasses import dataclass

### 
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

### crazyswarm 
from crazyswarm_application.msg import UserCommand
from crazyswarm_application.msg import AgentsStateFeedback
from crazyswarm_application.msg import AgentState
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from crazyswarm_application.srv import Agents

### smaclike 
import numpy as np
from ament_index_python.packages import get_package_share_directory

# from smaclike.mamp.envs.mampenv import MACAEnv
# from smaclike.mamp.agents.agent import Agent
# from smaclike.obstacle.obstacle import from_file
# from smaclike.mamp.policies.orca3dPolicy import ORCA3DPolicy
# from smaclike.mamp.configs.config import DT

from smaclike.mamp.util import mod2pi,l3norm, l3normsq, takeSecond, sqr, absSq
from smaclike.units.unit import Unit
from smaclike.units.unit_command import (
    AttackMoveCommand,
    AttackUnitCommand,
    MoveCommand,
    TurnCommand,
    NoopCommand,
    StopCommand,
)
from smaclike.units.unit_type import UnitType, CombatType
from smaclike.util import point_inside_cone, point_inside_sphere
from smaclike.util.faction import Faction
from smaclike.util.direction import Direction
from smaclike.maps.map import MapInfo, SmaclikeGroup, get_smaclike_map

### constants
## same as minimum scan range
AGENT_SIGHT_RANGE = 2.0
AGENT_SIGHT_RANGE_SQ = AGENT_SIGHT_RANGE**2
## same as LASER_MAX_DIST
#AGENT_TARGET_RANGE = 1.0
#AGENT_TARGET_RANGE_SQ = AGENT_TARGET_RANGE**2
MOVE_AMOUNT = 0.2
STEP_MUL = 8
REWARD_SCALE = 20 
REWARD_WIN = 200
REWARD_KILL = 10
REWARD_TIME = 0.1
NEG_SCALE = 0.5

### Unit w/o MACA unit, use agent_state instead ### 
from enum import Enum 
# do not use MOVE, INTERNAL_TRACKING
class FlightState(Enum) : 
    IDLE = 0
    TAKEOFF = 1
    MOVE = 2
    MOVE_VELOCITY = 3
    INTERNAL_TRACKING = 4
    HOVER = 5
    LAND = 6
    EMERGENCY = 7

    def __str__(self) :
        str_dict = {
            0: "IDLE", 1: "TAKEOFF", 2: "MOVE", 3: "MOVE_VELOCITY",
            4: "INTERNAL_TRACKING", 5: "HOVER", 6: "LAND", 7: "EMERGENCY"
        }
        return str_dict[self.value]

class Agent_State : 
    def __init__(self, id, node_time):
        self.id = id
        self.t = node_time
        self.transform = np.eye(4)
        self.velocity = np.zeros(3)
        self.flight_state = FlightState.IDLE
        self.radio_connection = False 

        ### original struct
        self.target_queue = []
        self.target_yaw = 0.0
        self.previous_target = np.zeros(3)
        self.previous_yaw = 0.0
        self.completed = False
        self.mission_capable = True

    def convert_to_msg(self):
        msg = AgentState()
        msg.id = self.id
        msg.flight_state = self.flight_state
        msg.connected = self.radio_connection
        msg.completed = self.completed
        msg.mission_capable = self.mission_capable

        return msg 

class SmaclikeUnit(Unit) : 
    def __init__(self, node, kwargs):
        super().__init__(**kwargs)
        self.node = node
        node_time = self.node.get_clock().now()
        self.MACA_agent = Agent_State(self.id, node_time)
    
@dataclass
class Commander : 
    task : str
    cont : str
    agents : List[str]
    target : List[float]
    duration : float
    options : str 
    sent_mission : bool
    
    # TODO : convert to (list of) UserCommand message
    # depending on agents (all, cf_1, cf_2, etc...)
    def convert_to_msg (self, all_keys, is_external = False):
        if len(self.agents) == 0 : 
            return

        # takeoff : always takeoff all 
        if self.task == "takeoff" and self.agents[0] == "all":  
            command = UserCommand()
            command.cmd = "takeoff_all"
            return command 
        # goto_velocity : always individual
        elif self.task == "goto_velocity" and self.agents[0] != "all": 
            result = []
            for agent in self.agents : 
                tmp_cmd = UserCommand() 
                tmp_cmd.cmd = "goto_velocity"
                tmp_cmd.uav_id = [agent]
                # goal 
                tmp_cmd.goal.x = self.target[0]
                tmp_cmd.goal.y = self.target[1]
                tmp_cmd.goal.z = self.target[2]
                tmp_cmd.yaw = self.target[3]
                # if is_external : velocity control else position control
                tmp_cmd.is_external = is_external                
                
                result.append(tmp_cmd)            
            return result
        # land : can be all(when terminates normally) or individual 
        elif self.task == "land" : 
            if self.agents[0] == "all" : 
                command = UserCommand()
                command.cmd = "land"
                command.uav_id = list(all_keys)
                return command
            else : 
                command = UserCommand()
                command.cmd = "land"
                command.uav_id = self.agents.copy()
                return command 

### HELPER Functions 
def key_to_id(key) : 
    return int(key[3:]) - 1

def call_state_printer(id, agent_state) : 
    print(f"{id} (rc: {"y" if agent_state.radio_connection else "n"}) \
          (tsk: {"done" if agent_state.completed else "not-done"}) \
            {str(agent_state.flight_state)}|")
          
class SmaclikeAviary(Node):

    def __init__(self):
        super().__init__('smaclike_aviary')

        self.get_logger().info("start constructor") 
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)

        # service called when all unit's in initial position from mission node
        # self.create_service(Agents, "/external/receive", self.housekeeping)

        self.rate = 10 # 10Hz
        self.subscriber = []

        ### mission_handler ###
        # get config file path 
        self.declare_parameter("crazyflies_path", "")
        self.declare_parameter("config_path", "")
        
        self.crazyflies_path = self.get_parameter("crazyflies_path").value
        self.config_path = self.get_parameter("config_path").value

        self.agent_description = {}
        # set parameters 
        with open(self.crazyflies_path, 'r') as ymlfile:
            crazyflie_params = yaml.safe_load(ymlfile)
        
        # load crazyflies from params ### 
        cf_names = crazyflie_params['robots'].keys()

        # all cf_names should start with prefix "cf_"        
        self.cf_prefix = "cf_"
        if not all([i[:3] == self.cf_prefix for i in cf_names]) : 
            self.raise_error("prefix for name is not cf_")
            return
        
        # check cf_x starts from 1
        idx = 1
        tmp = list(cf_names)
        tmp.sort()
        for i in tmp : 
            if int(i[3:]) != idx :
                self.raise_error("crazyflie id is not in numerical order")
                return
        
        self.start_mission = False
        # all commands 
        self.commander_queue = None
        # currently running commands
        self.commander_buffer = None
        # UserCommand to publish
        self.user_buffer = None
        ### smac ###
        self.random_init = False
        self.takeoff_height = 1.0

        with open(self.config_path, 'r') as ymlfile:
            config_params = yaml.safe_load(ymlfile)
        self.protected_zone = config_params['trajectory_parameters']['protected_zone']
        self.move_land_tolerance = config_params['move_land_tolerance']
        smaclike_file = config_params['smaclike_map']
        
        unit_yaml = os.path.join(
            get_package_share_directory('crazyswarm_application'),
            'launch',
            smaclike_file)
        
        self.unit_params = None 
        with open(unit_yaml, 'r') as ymlfile: 
            tmp = yaml.safe_load(ymlfile)
            self.unit_params = tmp["smaclike_map"]

        map_name = self.unit_params["name"]
        self.map_info = get_smaclike_map(map_name)

        initial_poses = []
        base_points = []
        for g in self.map_info.groups : 
            self.get_logger().info(f"init : {g.init_pose} / base : {g.base_point}")
            if len(g.init_pose)!=4 or len(g.base_point)!=3 :
                self.raise_error(f"invalid length : init_pose \
                                 {len(g.init_pose)}, base_point {len(g.base_point)}")
            initial_poses.append(g.init_pose)
            base_points.append(g.base_point)
        self.initial_poses = np.array(initial_poses)
        self.base_points = np.array(base_points)

        self.max_unit_radius = max(
            type.radius for group in self.map_info.groups for type in group.unit
        ) 

        self.n_agents = self.map_info.num_allied_units
        self.n_enemies = self.map_info.num_enemy_units
        num_target_actions = self.n_enemies
        # Noop, Stop, Move 6 ways, Turn, 10 + attack target
        self.n_actions = 10 + num_target_actions
        self.NUM_DRONES = self.n_agents + self.n_enemies
        
        if self.NUM_DRONES != len(cf_names) : 
            self.raise_error(f"Number mismatch between crazyflies {len(cf_names)} and units {self.NUM_DRONES}")
            return    

        self.agents = {}
        self.enemies = {}
        self.all_units = {}
        self.ally_unit_type = None
        self.enemy_unit_type = None
        self.cx_cy_cz = np.array([0,0, self.map_info.height/2])
        # reset all units 
        self.reset_units()
        
        # fill commander_queue
        self.init_commander()

        ### ROS node ###
        self.last_mission_time = self.get_clock().now()        
        self.command_publisher = self.create_publisher(UserCommand, '/user', self.get_qos_profile(25))
        self.agent_state_subscription = self.create_subscription(AgentsStateFeedback, 
                                            "/agents", self.agent_event_callback, self.get_qos_profile(15))
        self.get_logger().info("end constructor")
            
    def get_qos_profile(self,depth):
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

    def raise_error(self, err_msg):
        self.get_logger().info(err_msg)
        self.destroy_node()

    def init_commander(self) : 
        self.user_buffer = {}
        for unit in self.all_units.values() : 
            self.user_buffer[unit.id] = None
    
        self.commander_queue = []

        # takeoff all command 
        takeoff_cmd = Commander(
            task = "takeoff_all",
            cont = "wait",
            agents = ["all"],
            sent_mission = False
        )
        self.commander_queue.append(takeoff_cmd)

        # individual move to initial position
        for i, unit in enumerate(self.all_units.items()) : 
            goto_cmd = Commander(
                task = "goto_velocity",
                cont = "wait" if i == len(self.all_units) - 1 else "cont",
                agents = [self.cf_prefix + str(unit.id+1)],
                target = self.initial_poses[unit.id].copy(),
                sent_mission = False
            )
            self.commander_queue.append(goto_cmd)
        
        # hold when all finished
        hold_cmd = Commander(
            task = "hold",
            cont = "wait",
            agents = ["all"],
            duration = 2.0,
            sent_mission = False
        )
        self.commander_queue.append(hold_cmd)


    def reset_units(self):
        self.ally_unit_type = None
        self.enemy_unit_type = None
        for group in self.map_info.groups : 
            faction = group.faction
            unit_type  = group.unit[0]
            if faction == Faction.ALLY : 
                if self.ally_unit_type is None : 
                    self.ally_unit_type = unit_type
                    # for naive
                    #self.ally_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                else: 
                    self.raise_error("All ally unit type must be same")
                    return

            else : 
                if self.enemy_unit_type is None : 
                    self.enemy_unit_type = unit_type
                    self.enemy_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                else: 
                    self.raise_error("All enemy unit type must be same")
                    return

        self.total_ally_hp = self.ally_unit_type.stats.hp * self.n_agents
        self.total_enemy_hp = self.enemy_unit_type.stats.hp * self.n_enemies

        for group in self.map_info.groups : 
            faction = group.faction
            faction_dict = self.agents if faction == Faction.ALLY else self.enemies
            id_overall = len(self.all_units)
            id_in_faction = len(faction_dict)
            
            #self.last_pos[id_overall, :] = [group.x, group.y, group.z]

            pose = [group.x, group.y, group.z, group.yaw]
            
            if self.random_init : 
                scale = np.array([0.1, 0.1, 0.0, 0.5])
                tmp_noise = scale * (2*self._np_random.random((4,))-1)
                #print(tmp_noise)
                pose += tmp_noise
                self.pos_noise[id_overall] = tmp_noise[:]
                #print(self.pos_noise[id_overall])
            
            tmp_type = self.ally_unit_type if faction == Faction.ALLY else self.enemy_unit_type

            unit = SmaclikeUnit(self, unit_types = tmp_type, faction = faction, pos = pose,
                                    idd = id_overall, idd_in_faction = id_in_faction, ctrl_freq=self.rate)
            
            self.all_units[id_overall] = unit
            faction_dict[id_in_faction] = unit
    
    ### called after takeoff by mission node
    ### initialize all necessary topics 
    def housekeeping(self, cf_names):

        self.get_logger().info("housekeeping finished")
        for name in cf_names:
            # self.agents.update({name: PoseStamped()})
            self.get_logger().info('creating: ', name)

            # necessary topics 
            # pose, neighbor, 
            self.subscriber.append(
                self.create_subscription(
                PoseStamped, 
                name + "/pose",
                partial(self.listener_callback, agent_id=name), 
                10))
            
        #self.is_takeoff
        #self.collision
        self.is_alive = [True]*self.NUM_DRONES

        self._enemy_attack()

        self.get_logger().info("start main loop")
        self.timer = self.create_timer(1./self.rate, self._main_loop)

    def _enemy_atack(self) : 
        pos = np.array(self.map_info.attack_point)

        for i, enemy in self.enemies.items() : 
            if enemy.hp == 0 : 
                continue 
            attack_move_command = AttackMoveCommand(pos[i], targets = self.agents.values())
            enemy.command = attack_move_command 

        # naive test
        for i, ally in self.agents.items() : 
            if ally.hp == 0:
                continue
            attack_move_command = AttackMoveCommand(self.initial_poses[i][:3], targets=self.enemies.values())
            ally.command = attack_move_command


    def all_units_ready(self) :

        return True
    
    def check_completed_agents(self, command) : 
        if command.agents[0] == "all" : 
            for unit in self.all_units.values() : 
                if not unit.MACA_agent.completed :
                    return False
        else : 
            for key in command.agents : 
                unit = self.all_units.get(key_to_id(key))
                if unit is not None and not unit.MACA_agent.completed : 
                    return False
        return True

    def agent_event_callback(self, msg) : 
        now = self.get_clock().now()

        # copy agent messages into local states 
        for agent in msg.agents :
            tmp_id = key_to_id(agent.id)
            tmp_unit = self.all_units.get(tmp_id)
            if tmp_unit is not None : 
                if agent.flight_state == FlightState.EMERGENCY : 
                    # remove from faction dict, unit dict
                    if tmp_unit.faction == Faction.ALLY :
                        del self.agents[tmp_unit.id_in_faction]
                    else : 
                        del self.enemies[tmp_unit.id_in_faction]
                    del self.all_units[tmp_unit.id]
                else : 
                    tmp_unit.MACA_agent.flight_state = agent.flight_state
                    tmp_unit.MACA_agent.radio_connection = agent.connected
                    tmp_unit.MACA_agent.completed = agent.completed

        # check agent's states
        agent_remove_list = []
        for id, unit in self.all_units.items() : 
            if not unit.radio_connection : 
                agent_remove_list.append(id)
            
            call_state_printer(id, unit.MACA_agent)
        print("\n")

        for i in agent_remove_list : 
            tmp_unit = self.all_units.get(i)
            if tmp_unit is not None : 
                if agent.flight_state == FlightState.EMERGENCY : 
                    # remove from faction dict, unit dict
                    if tmp_unit.faction == Faction.ALLY :
                        del self.agents[tmp_unit.id_in_faction]
                    else : 
                        del self.enemies[tmp_unit.id_in_faction]
                    del self.all_units[tmp_unit.id]
            
        # command handler
        if len(self.commander_buffer) > 0 : 
            success_count = 0 
            for cmd in self.commander_buffer : 
                # hold
                if cmd.task == "hold" : 
                    duration_seconds = (self.get_clock().now() - self.last_mission_time).seconds()
                    if cmd.duration < duration_seconds:
                        self.get_logger().info("waiting over {:.3}/{:.3}s".format(duration_seconds, cmd.duration))
                        success_count +=1 
                    else : 
                        self.get_logger().info("waiting {:.3}/{:.3}s".format(duration_seconds, cmd.duration))
                        return 
                # goto_velocity 
                elif cmd.task == "goto_velocity" : 
                    duration_seconds = (self.get_clock().now() - self.goto_velocity_command_time).seconds()
                    if duration_seconds > self.move_land_tolerance :
                        if self.check_completed_agents(cmd) :
                            success_count+=1 
                else :
                    if self.check_completed_agents(cmd) :
                        success_count+=1
            
            self.get_logger().info(f"Success {success_count}/{len(self.commander_buffer)}")
            
            if success_count == len(self.commander_buffer) : 
                self.last_mission_time = self.get_clock().now()
                # clear commander_buffer
                self.commander_buffer = []


        if len(self.commander_buffer) == 0 and len(self.commander_queue) > 0 :
            while len(self.commander_queue) > 0 :
                self.commander_buffer.append(self.commander_queue.pop(0))
                if self.commander_buffer[-1].cont == "wait" :
                    break
        
        # if no command left, change to asynchronous control?
        if len(self.commander_buffer) == 0 and len(self.commander_queue) == 0 : 
            self.raise_error("all commands finished, destroy the node...")
            return

        # fill user_buffer
        for cmd in self.commander_buffer : 
            if cmd.sent_mission : 
                continue

            if len(cmd.agents) == 0 : 
                self.raise_error("empty agent list in command")
        
        # publish UserCommand
        

    def listener_callback(self, msg, agent_id):
        self.agents[agent_id] = msg
        return
    
    # def timer_callback(self):
    #     key, value = random.choice(list(self.agents.items()))
    #     # string cmd
    #     # string[] uav_id
    #     # geometry_msgs/Point goal
    #     # float32 yaw
    #     msg = UserCommand()
    #     msg.uav_id.append(key)
    #     msg.goal.x = value.pose.position.x + self.random_generator()
    #     msg.goal.y = value.pose.position.y + self.random_generator()
    #     msg.goal.z = self.height

    #     print(msg.uav_id, msg.goal.x, msg.goal.y, msg.goal.z)
    #     self.publisher_.publish(msg)

    def seed(self, seed=None):
        # https://gymnasium.farama.org/_modules/gymnasium/utils/seeding/
        if seed is not None and not (isinstance(seed, int) and 0 <= seed): 
            if isinstance(seed, int) is False : 
                self.raise_error(f"seed is not integer {type(seed)}") 
            else : 
                self.raise_error(f"seed must be >=0, {seed}")

        seed_seq = np.random.SeedSequence(seed)
        #np_seed = seed_seq.entropy
        rng = np.random.Generator(np.random.PCG64(seed_seq))
        self._np_random = rng

    def _main_loop(self) : 

        ### step
        # action from policy 
        action = None
        reward = 0
        self._set_unit_command(action)
        shuffled_order = list(range(len(self.all_units)))
        self._np_random.shuffle(shuffled_order)

        # world step
        pre_action, _ = self._world_step()

        # simulation step 
        reward_attack = 0 
        unit_keys = list(self.all_units.keys())
        for i in shuffled_order :
            unit = self.all_units[unit_keys[i]]
            reward_tmp = self._game_step(unit)
            if not self.reward_dense and self.reward_time and reward_tmp < 0:
                reward_tmp = -1
            if abs(reward_tmp) > 0 : 
                print("unit {} : reward {}".format(unit.id, reward_tmp))
            reward_attack += reward_tmp 

            if unit.hp == 0 : 
                # TODO : update unit to dead in ORCA
                for i in range(self.NUM_DRONES) : 
                    if i== unit.id : 
                        continue

            drone_id = unit.id
            # TODO : publish action 

            # self._check_collision()
            obs = self._computeObs()

        reward+= reward_attack

        for i, unit in self.all_units.items() : 
            state = 
            if not self.is_alive[i] : 
                unit.hp = 0 
        
        self._update_deaths()

        if self.unit_died_this_step > 0 :
            obs = self._update_dead_obs(obs)

        all_enemies_dead = len(self.enemies) == 0 
        done = all_enemies_dead or len(self.agents) == 0
        
        if not done : 
            # update agent in MACA env 
    
        bonus_reward = self._computeReward()
        reward += bonus_reward


        if all_enemies_dead : 
            reward += REWARD_WIN

        if self.reward_time : 
            reward -= REWARD_TIME

        if self.reward_dense : 
            reward /= self.max_reward / REWARD_SCALE

        return

    def get_avail_actions(self) : 
        avail_for_dead = np.zeros(self.n_actions)
        avail_for_dead[0] = 1
        return [
            self._get_agent_avail_actions(self.agents[i])
            if i in self.agents
            else avail_for_dead
            for i in range(self.n_agents)
        ]

    def _getDroneStateVector(self, nth_drone) : 
        # 3 / 4 / 3 / 3 / 3 
        state =  np.hstack([self.pos[nth_drone, :], self.quat[nth_drone, :], self.rpy[nth_drone, :],
                           self.vel[nth_drone, :], self.ang_v[nth_drone, :]])
        return state.reshape(16,)

    def _get_command(self, unit, action) : 
        state = self._getDroneStateVector(unit.id)
        if action == 0 :
            return NoopCommand()
        if action == 1:
            return StopCommand()
        if 2 <= action <= 7:
            cur_yaw = unit.get_heading()[0]
            dpos = Direction(action - 2).dx_dy_dz * MOVE_AMOUNT
            dpos = np.array([dpos[0]*np.cos(cur_yaw) - dpos[1]*np.sin(cur_yaw),\
                              dpos[0]*np.sin(cur_yaw) + dpos[1]*np.cos(cur_yaw), dpos[2]])
            pos = state[0:3]
            return MoveCommand(pos + dpos)
        if 8 <= action <= 9 : 
            ## 8 counter clockwise, 9 clockwise
            sign = 1 if action == 8 else -1 
            #dyaw = sign * np.pi / 12
            return TurnCommand(sign)
        
        return AttackUnitCommand(self.enemies[action - 10])

    def _set_unit_command(self, action) : 
        # avail_actions = self.get_avail_actions()
        
        # for i, act in enumerate(action) : 
        #     ### action of dead agents
        #     if i not in self.agents :
        #         assert action[i] == 0
        #         continue 
            
        #     agent = self.agents[i]
        #     """ error occurs """
        #     if not avail_actions[i][act] : 
        #         self.raise_error(f"Invalid action for agent {i}: {act}")
        #     agent.command = self._get_command(agent, act)
        
        # for unit in self.all_units.values():
        #     unit.clean_up_target()
        # #print("targets : ", [unit.target for unit in self.all_units.values()])

        attackmoving_units = [
            ally
            for ally in self.agents.values()
            if ally.target is None
        ]

        # naive test
        print("attackmoving : ", attackmoving_units)
        if attackmoving_units :
            ## neighbour_finder_ally
            for unit in attackmoving_units : 
                attackmoving_radii = (unit.minimum_scan_range + self.max_unit_radius)**2
                cur_agent = unit.MACA_agent
                self.__find_neighbor(cur_agent, attackmoving_radii ,isAgent = True, reset = True, option = False)
                #print("ally neighbor: ",[enemy[0].id for enemy in cur_agent.neighbors])
                unit.potential_targets = [(self.enemies[a_.id - self.n_agents], dist) for a_, dist in cur_agent.neighbors if not a_.is_collision]
            
            for unit in self.enemies.values():
                if unit.target is not None : 
                    # add this agent as target of this agent's target, but last priority
                    unit.target.potential_targets.append((unit, 2e9))

        ## enemy with no target
        attackmoving_units = [
            enemy
            for enemy in self.enemies.values()
            if enemy.target is None
        ]
        #print("attackmoving : ", attackmoving_units)
        if attackmoving_units :
            ## neighbour_finder_ally
            for unit in attackmoving_units : 
                attackmoving_radii = (unit.minimum_scan_range + self.max_unit_radius)**2
                cur_agent = unit.MACA_agent
                self.__find_neighbor(cur_agent, attackmoving_radii ,isAgent = True, reset = True, option = True)
                unit.potential_targets = [(self.agents[a_.id], dist) for a_, dist in cur_agent.neighbors if not a_.is_collision]
            
            for unit in self.agents.values():
                if unit.target is not None : 
                    # add this agent as target of this agent's target, but last priority
                    unit.target.potential_targets.append((unit, 2e9))


    def _get_agent_avail_actions(self, unit, targets) : 
        actions = np.zeros(self.n_actions)
        actions[1] = 1

        # 2~7
        result = self.__can_move(unit)
        #print("can move result : ",result)
        for direction in Direction:
            # check all directions
            actions[2 + direction.value] = result[direction.value]
        if targets is None:
            targets = self.enemies.values()
        
        can_turn = self.__can_turn(unit)
        actions[8] = can_turn
        actions[9] = can_turn
        
        distance = None
        for target in targets:
            if type(target) is tuple:
                target, distance = target
            if target is unit:
                continue
            actions[10 + target.id_in_faction] = self.__can_target(
                unit, target, distance=distance
            )
        # save available action 
        # self.last_avail_action[unit.id_in_faction] = actions[:]
        return actions
    
    def __can_move(self, unit) : 
        result = []
        check_value = 0.5 * unit.pref_spped
        for direction in Direction : 
            cur_yaw = unit.get_heading()[0]
            dpos = direction.dx_dy_dz
            dpos = np.array([dpos[0]*np.cos(cur_yaw) - dpos[1]*np.sin(cur_yaw),\
                              dpos[0]*np.sin(cur_yaw) + dpos[1]*np.cos(cur_yaw), dpos[2]])
            npos = unit.get_pos() + dpos * check_value
            l_ = self.map_info.length 
            w_ = self.map_info.width
            h_ = self.map_info.height
            # 0.3 minimum height
            check_map_limit= abs(npos[0]) < l_/2 and abs(npos[1]) < w_/2 \
                and 0.3 < npos[2] < h_
            if not check_map_limit :
                result.append(False)
                continue

            # create tmp_agent in npos
            tmp_agent.pos_global_frame = npos
            #tmp_agent.vel_global_frame = dpos*unit.pref_speed 
            rangeSq = (unit.type.radius + unit.minimum_scan_range)**2
            # 4.6225
            # print("rangeSq : ",rangeSq)
            # except myself
            tmp_agent.is_collision = False
            self.__find_neighbor(tmp_agent, rangeSq, isAgent = True, reset = True, option = None)
            self.__find_neighbor(tmp_agent, rangeSq, isAgent = False, reset = False, option = None)
            #print("can move : {}, npos : {}, col : {}".format(unit.id, npos, tmp_agent.is_collision))
            #print("neighbors : ",tmp_agent.neighbors)
            ## whether npos is close to neighbors
            if self.__agent_check_collision(tmp_agent) : 
                result.append(False)
            else : 
                result.append(True)
        return result 

    def __can_turn(self, unit) : 
        ### check if roll, pitch under certain threshold
        states = self._getDroneStateVector(unit.id)  
        roll, pitch, yaw = states[7:10]
        threshold = 0.2 # threshold should be lower than emergency state threshold
        if roll < threshold and pitch < threshold :
            return True
        return False
    
    def __can_target(self, unit, target, distance):
        if target.hp == 0 or unit.hp == 0:
            return 0
        if distance is not None:
            return distance <= unit.attack_range
        res, _, _ = point_inside_cone(unit.get_pos(), target.get_pos(), 
                                      unit.get_heading(), unit.fov_range,  
                                      [target.radius, unit.target_range])
        return res

    def _game_step(self, unit) : 
        hit_unit = None
        if unit.attacking and unit.cooldown < 1e-7 : 
            hit_unit = self._do_laser_action(unit)

        hit_reward = unit.game_step(hit_unit = hit_unit, is_dense = self.reward_dense, \
                                    only_positive = self.reward_only_positive)
        
        return hit_reward
    
    def _do_laser_action(self, unit) : 
        target_id = unit.target.id
        target_pos = 
        laser_length = unit.attack_range

        hit_unit = None

    
        # TODO : use visibility polygon to check result

        # TODO : add marker for visualization
        return hit_unit    


def main(args=None):
    rclpy.init(args=args)

    try : 
        node = SmaclikeAviary()
        thread_count = 1
        executor = MultiThreadedExecutor(num_threads=thread_count)
        executor.add_node(node)
        try: 
        # wait until all topics have been received
        # wait until all units reached initial position
        # while rclpy.ok() : 
        #     rclpy.spin_once(node, timeout_sec=0.1)
        #     if node.all_state_ready() :
        #         node.get_logger().info("All state messages received. Start")
        #         break
        #     node.get_logger().info("Waiting for pose, velocity, attitude data...")
            executor.spin()
        
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt')
        
        finally : 
            executor.shutdown()
            # node.get_logger().info('Shut down')
            # node is destroyed explicitly
            # when error occurs / main loop finishes
            #node.destroy_node()

    finally :
        rclpy.shutdown()


if __name__ == '__main__':
    main()