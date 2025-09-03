#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from functools import partial

### crazyswarm 
from crazyswarm_application.msg import UserCommand
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from crazyswarm_application.srv import Agents

### smaclike 
import numpy as np
from smaclike.mamp.envs.mampenv import MACAEnv
from smaclike.mamp.agents.agent import Agent
from smaclike.obstacle.obstacle import from_file
from smaclike.mamp.policies.orca3dPolicy import ORCA3DPolicy
from smaclike.mamp.configs.config import DT

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
from smaclike.maps.map import MapInfo, Group, get_standard_map

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
class ExternalPublisher(Node):

    def __init__(self):
        super().__init__('external_publisher')
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)

        # service called when all unit's in initial position from mission node
        self.create_service(Agents, "/external/receive", self.housekeeping)

        self.rate = 10 # 10Hz
        
        self.max = 2.0
        self.height = 1.0
        self.NUM_DRONES = 2

        self.agents = {}
        self.enemies = {}
        self.all_units = {}
        self.subscriber = []

    def _reset_units(self):
        self.ally_unit_type = None
        self.enemy_unit_type = None
        for group in self.map_info.groups : 
            faction = group.faction
            unit_type, _  = group.units[0]
            if faction == Faction.ALLY : 
                if self.ally_unit_type is None : 
                    self.ally_unit_type = unit_type
                    # for naive
                    #self.ally_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                else: 
                    assert self.ally_unit_type == unit_type, "[SimpleCAAviary] All ally unit type must be same"
            else : 
                if self.enemy_unit_type is None : 
                    self.enemy_unit_type = unit_type
                    self.enemy_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                else: 
                    assert self.enemy_unit_type == unit_type, "[SimpleCAAviary] All enemy unit type must be same"

        self.total_ally_hp = self.ally_unit_type.stats.hp * self.n_agents
        self.total_enemy_hp = self.enemy_unit_type.stats.hp * self.n_enemies

        for group in self.map_info.groups : 
            faction = group.faction
            faction_dict = self.agents if faction == Faction.ALLY else self.enemies
            id_overall = len(self.all_units)
            id_in_faction = len(faction_dict)
            self.last_pos[id_overall, :] = [group.x, group.y, group.z]
            pos = [group.x, group.y, group.z, group.roll, group.pitch, group.yaw]
            
            if self.random_init : 
                scale = np.array([0.1, 0.1, 0.0, 0.0, 0.0, 0.5])
                tmp_noise = scale * (2*self._np_random.random((6,))-1)
                #print(tmp_noise)
                pos += tmp_noise
                self.pos_noise[id_overall] = tmp_noise[:]
                #print(self.pos_noise[id_overall])
            
            if faction == Faction.ALLY : 
                unit = Unit(self.ally_unit_type, faction, pos, id_overall, id_in_faction, self.CTRL_FREQ)
            else : 
                unit = Unit(self.enemy_unit_type, faction, pos, id_overall, id_in_faction, self.CTRL_FREQ)
            self.all_units[id_overall] = unit
            faction_dict[id_in_faction] = unit
    
    ### called after takeoff by mission node
    ### initialize all necessary topics 
    def housekeeping(self, request, response):
        if self.NUM_DRONES != len(request.names) : 
            self.get_logger().info("[external] housekeeping failed")
            self.destroy_node()
            return  

        self.get_logger().info("[external] housekeeping finished")
        for name in request.names:
            self.agents.update({name: PoseStamped()})
            self.get_logger().info('creating: ', name)

            # necessary topics 
            # pose, neighbor, 
            self.subscriber.append(
                self.create_subscription(
                PoseStamped, 
                name + "/pose",
                partial(self.listener_callback, agent_id=name), 
                10))
            
        self.get_logger().info("[external] start timer")
        self.timer = self.create_timer(1./self.rate, self.step)
        return response
    
    def listener_callback(self, msg, agent_id):
        self.agents[agent_id] = msg
        return
    
    # def random_generator(self):
    #     return ((random.random()*2) - 1) * self.max

    def timer_callback(self):
        key, value = random.choice(list(self.agents.items()))
        # string cmd
        # string[] uav_id
        # geometry_msgs/Point goal
        # float32 yaw
        msg = UserCommand()
        msg.uav_id.append(key)
        msg.goal.x = value.pose.position.x + self.random_generator()
        msg.goal.y = value.pose.position.y + self.random_generator()
        msg.goal.z = self.height

        print(msg.uav_id, msg.goal.x, msg.goal.y, msg.goal.z)
        self.publisher_.publish(msg)

    def all_state_ready(self) :

        return True
    
    def step(self) : 

        return


def main():
    if (len(sys.argv) > 1):
        print('too number of arguments', len(sys.argv))
        exit()
        
    rclpy.init(args=None)

    node = ExternalPublisher()

    try: 
        # wait until all topics have been received
        # wait until all units reached initial position
        while rclpy.ok() : 
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.all_state_ready() :
                node.get_logger().info("All state messages received. Start")
                break
            node.get_logger().info("Waiting for pose, velocity, attitude data...")

        # Now spin normally
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Shut down')

    # node is destroyed explicitly
    # when error occurs / main loop finishes
    rclpy.shutdown()


if __name__ == '__main__':
    main()