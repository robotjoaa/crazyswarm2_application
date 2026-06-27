#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import os
from functools import partial
import yaml
import math

from typing import Dict, List, Tuple
from dataclasses import dataclass, field
from collections import deque

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
from geometry_msgs.msg import Twist

### visualization
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as ScipyR

### smaclike
import numpy as np
from ament_index_python.packages import get_package_share_directory
from smaclike_hp_handlers import HpEventHandler, RvizHpHandler, LedHpHandler

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
from smaclike.maps.map import MapInfo, SmaclikeGroup, get_smaclike_map, get_standard_map

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
    def __init__(self, node, **kwargs):
        super().__init__(**kwargs)
        self.node = node
        node_time = self.node.get_clock().now()
        # Keep the Unit's MACA_agent but attach fields expected by mission-style logic.
        self.MACA_agent.t = node_time
        self.MACA_agent.flight_state = FlightState.IDLE
        self.MACA_agent.radio_connection = False
        self.MACA_agent.completed = False
        self.MACA_agent.mission_capable = True
    
@dataclass
class Commander:
    task: str
    cont: str
    agents: List[str] = field(default_factory=list)
    # [x, y, z, yaw] for goto/attack; ignored for takeoff/land/hold
    target: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    duration: float = 0.0
    # For attack: target drone id string. For goto_velocity: unused.
    options: str = ""
    # Explicit yaw rate (rad/s) for goto_velocity turn-in-place; 0 = use target[3] yaw instead.
    yaw_rate: float = 0.0
    sent_mission: bool = False

    def convert_to_msg(self, all_keys, is_external=False):
        if not self.agents:
            return None

        # takeoff_all
        if self.task == "takeoff" and self.agents[0] == "all":
            cmd = UserCommand()
            cmd.cmd = "takeoff_all"
            return cmd

        # goto_velocity — individual agents only
        if self.task == "goto_velocity" and self.agents[0] != "all":
            result = []
            for agent in self.agents:
                cmd = UserCommand()
                cmd.cmd = "goto_velocity"
                cmd.uav_id = [agent]
                cmd.goal.x = float(self.target[0])
                cmd.goal.y = float(self.target[1])
                cmd.goal.z = float(self.target[2])
                cmd.yaw = float(self.target[3])
                cmd.yaw_rate = float(self.yaw_rate)
                cmd.is_external = is_external
                result.append(cmd)
            return result

        # attack — one command per shooter; options holds the target drone id
        if self.task == "attack":
            result = []
            for agent in self.agents:
                cmd = UserCommand()
                cmd.cmd = "attack"
                cmd.uav_id = [agent]
                if self.options:
                    cmd.uav_id.append(self.options)
                cmd.goal.x = float(self.target[0])
                cmd.goal.y = float(self.target[1])
                cmd.goal.z = float(self.target[2])
                result.append(cmd)
            return result

        # land — all or individual
        if self.task == "land":
            cmd = UserCommand()
            cmd.cmd = "land"
            cmd.uav_id = list(all_keys) if self.agents[0] == "all" else self.agents.copy()
            return cmd

        return None

### HELPER Functions 
def key_to_id(key) : 
    return int(key[3:]) - 1

def call_state_printer(id, agent_state) : 
    rc = 'y' if agent_state.radio_connection else 'n'
    tsk = 'done' if agent_state.completed else 'not-done'
    print(f"{id} (rc: {rc}) (tsk: {tsk}) {str(agent_state.flight_state)}|")
          
class SmaclikeAviary(Node):

    def __init__(self):
        super().__init__('smaclike_aviary')

        self.get_logger().info("start constructor") 
        self.publisher_ = self.create_publisher(UserCommand, '/user/external', 10)

        self.rate = 10 # 10Hz
        self.control_dt = 1.0 / float(self.rate)
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
        tmp = list(cf_names)
        tmp.sort()
        #print(tmp)
        for idx, i in enumerate(tmp) : 
            if int(i[3:]) != idx + 1 :
                self.raise_error("crazyflie id is not in numerical order")
                return
        
        self.start_mission = False
        self.housekeeping_done = False
        # all commands 
        self.commander_queue = None
        # currently running commands
        self.commander_buffer = None
        # UserCommand to publish
        self.user_buffer = None
        ### smac ###
        self.random_init = False
        self.takeoff_height = 1.0

        self.seed(42)

        with open(self.config_path, 'r') as ymlfile:
            config_params = yaml.safe_load(ymlfile)
        self.protected_zone = config_params['trajectory_parameters']['protected_zone']
        self.move_land_tolerance = config_params['move_land_tolerance']
        smaclike_file = config_params['smaclike_map']

        # Resolve smaclike map from configured file/name without hardcoded absolute paths.
        # `smaclike_map` in config can be either "test" or "test.yaml".
        map_name = os.path.splitext(os.path.basename(smaclike_file))[0]
        launch_dir_candidates = [
            os.path.dirname(os.path.abspath(self.config_path)),
            os.path.join(get_package_share_directory('crazyswarm_application'), 'launch'),
        ]

        self.map_info = None
        load_error = None
        for launch_dir in launch_dir_candidates:
            try:
                self.map_info = get_smaclike_map(map_name, launch_dir)
                break
            except FileNotFoundError as e:
                load_error = e

        # Fallback 1: try launch/maps/<name>.json (installed alongside launch files).
        # Fallback 2: try JSON format maps from the scripts smaclike_maps directory.
        # Both use the old Group schema (x,y,z,yaw,faction,units); convert to SmaclikeGroup.
        if self.map_info is None:
            json_candidates = [
                os.path.join(d, 'maps', f'{map_name}.json')
                for d in launch_dir_candidates
            ]
            json_candidates.append(None)  # sentinel → use get_standard_map()

            for json_path in json_candidates:
                try:
                    if json_path is None:
                        raw = get_standard_map(map_name)
                        source = f"get_standard_map('{map_name}')"
                    else:
                        raw = MapInfo.from_file(json_path, is_json=True)
                        source = json_path
                    new_groups = []
                    for g in raw.groups:
                        if hasattr(g, 'init_pose'):
                            new_groups.append(g)
                        else:
                            unit_type = g.units[0][0]
                            new_groups.append(SmaclikeGroup(
                                init_pose=[float(g.x), float(g.y), float(g.z), float(g.yaw)],
                                base_point=[float(g.x), float(g.y), float(g.z)],
                                faction=g.faction,
                                unit=unit_type,
                            ))
                    raw.groups = new_groups
                    self.map_info = raw
                    self.get_logger().info(f"Loaded JSON map '{map_name}' from {source}")
                    break
                except (FileNotFoundError, KeyError, Exception) as e:
                    load_error = e

        if self.map_info is None:
            available = []
            for launch_dir in launch_dir_candidates:
                for subdir, ext in [('smaclike_maps', '.yaml'), ('maps', '.json')]:
                    d = os.path.join(launch_dir, subdir)
                    if os.path.isdir(d):
                        available.extend(sorted([f for f in os.listdir(d) if f.endswith(ext)]))
            self.raise_error(
                f"smaclike map '{smaclike_file}' not found. "
                f"Looked under: {launch_dir_candidates}. "
                f"Available maps: {sorted(set(available))}. "
                f"Last error: {load_error}"
            )
            return

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
            group.unit.radius for group in self.map_info.groups
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

        # topic-fed drone states: [pos(3), quat(4), rpy(3), vel(3), ang_v(3)]
        self.pos = np.zeros((self.NUM_DRONES, 3), dtype=np.float64)
        self.quat = np.zeros((self.NUM_DRONES, 4), dtype=np.float64)
        self.rpy = np.zeros((self.NUM_DRONES, 3), dtype=np.float64)
        self.vel = np.zeros((self.NUM_DRONES, 3), dtype=np.float64)
        self.ang_v = np.zeros((self.NUM_DRONES, 3), dtype=np.float64)

        self.agents = {}
        self.enemies = {}
        self.all_units = {}
        self.ally_unit_type = None
        self.enemy_unit_type = None
        self.cx_cy_cz = np.array([0,0, self.map_info.height/2])
        # reset all units 
        self.reset_units()
        for i in range(self.NUM_DRONES):
            self.pos[i, :] = self.initial_poses[i][:3]
            self.quat[i, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        
        # fill commander_queue
        self.init_commander()

        ### ROS node ###
        self.last_mission_time = self.get_clock().now()
        self.goto_velocity_command_time = self.last_mission_time
        self.command_publisher = self.create_publisher(UserCommand, '/user', self.get_qos_profile(25))
        self.unit_info_publisher = self.create_publisher(MarkerArray, 'rviz/unit_info', self.get_qos_profile(10))
        self.map_boundary_publisher = self.create_publisher(Marker, 'rviz/map_boundary', self.get_qos_profile(10))
        self.smac_obstacles_publisher = self.create_publisher(MarkerArray, 'rviz/smac_obstacles', self.get_qos_profile(10))
        self.agent_state_subscription = self.create_subscription(AgentsStateFeedback,
                                            "/agents", self.agent_event_callback, self.get_qos_profile(15))
        self.pose_state_subscriptions = []
        self.vel_state_subscriptions = []
        for name in sorted(cf_names):
            self.pose_state_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/{name}/pose",
                    partial(self.pose_state_callback, agent_id=name),
                    self.get_qos_profile(20)))
            self.vel_state_subscriptions.append(
                self.create_subscription(
                    Twist,
                    f"/{name}/vel",
                    partial(self.vel_state_callback, agent_id=name),
                    self.get_qos_profile(20)))
        # source of truth for visibility from crazyswarm_application/planning node
        self.visible_enemy_cache = {}  # unit_id -> List[(neighbor_ros_id, distance)]
        self.attack_hit_subscription = self.create_subscription(
            UserCommand, '/attack_hit', self._attack_hit_callback,
            self.get_qos_profile(30))

        # HP event handlers — append new HpEventHandler subclasses here to add
        # feedback channels without touching the hit-processing logic.
        self._hp_event_handlers: List[HpEventHandler] = [
            RvizHpHandler(self),
            LedHpHandler(self, self.cf_prefix),
        ]
        self.command_publish_queue = []
        # Per-unit exit queues for the safe-zone → land sequence (Option B).
        self.unit_cmd_queues: Dict[int, deque] = {}
        self._exit_hold_start: Dict[int, object] = {}
        self._game_over = False
        self._waiting_for_landing = False
        self.timer = None
        # Start bootstrap lifecycle immediately on node startup.
        self.housekeeping(list(cf_names))
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
        self.commander_buffer = []

        # takeoff all command 
        takeoff_cmd = Commander(
            task = "takeoff",
            cont = "wait",
            agents = ["all"],
            sent_mission = False
        )
        self.commander_queue.append(takeoff_cmd)

        # individual move to initial position
        for i, unit in enumerate(self.all_units.values()) : 
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
            unit_type  = group.unit
            if faction == Faction.ALLY :
                if self.ally_unit_type is None :
                    self.ally_unit_type = unit_type
                    self.ally_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                elif unit_type != self.ally_unit_type :
                    self.raise_error("All ally unit type must be same")
                    return

            else :
                if self.enemy_unit_type is None :
                    self.enemy_unit_type = unit_type
                    self.enemy_unit_type.stats.minimum_scan_range = 4 * AGENT_SIGHT_RANGE
                elif unit_type != self.enemy_unit_type :
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

            #pose = [group.x, group.y, group.z, group.yaw]
            pose = group.init_pose[:]
            
            if self.random_init : 
                scale = np.array([0.1, 0.1, 0.0, 0.5])
                tmp_noise = scale * (2*self._np_random.random((4,))-1)
                #print(tmp_noise)
                pose += tmp_noise
                self.pos_noise[id_overall] = tmp_noise[:]
                #print(self.pos_noise[id_overall])
            
            tmp_type = self.ally_unit_type if faction == Faction.ALLY else self.enemy_unit_type

            unit = SmaclikeUnit(self, unit_type = tmp_type, faction = faction, pos = pose,
                                    idd = id_overall, idd_in_faction = id_in_faction, ctrl_freq=self.rate)
            
            self.all_units[id_overall] = unit
            faction_dict[id_in_faction] = unit
    
    ### called at node startup
    def housekeeping(self, cf_names):
        self.get_logger().info(f"housekeeping start for {len(cf_names)} units")
        self.is_alive = [True] * self.NUM_DRONES
        self.housekeeping_done = True

        if self.timer is None:
            self.get_logger().info("start main loop")
            self.timer = self.create_timer(1./self.rate, self._main_loop)

        # publish static map elements at 1 Hz so rviz always shows them
        self.create_timer(1.0, self._publish_static_map)

    # ------------------------------------------------------------------ #
    #  Visualization helpers                                              #
    # ------------------------------------------------------------------ #

    def _publish_unit_info(self):
        """Publish TEXT_VIEW_FACING markers above each live drone showing HP and faction."""
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        for unit_id, unit in self.all_units.items():
            m = Marker()
            m.header.frame_id = "/world"
            m.header.stamp = now
            m.ns = "unit_info"
            m.id = unit_id
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD

            pos = unit.get_pos()
            if pos is None:
                pos = self.pos[unit_id]
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2]) + 0.25
            m.pose.orientation.w = 1.0

            m.scale.z = 0.12  # text height in metres

            faction_str = "ALLY" if unit.faction.name == "ALLY" else "ENEMY"
            hp_str = f"{int(unit.hp)}/{int(unit.max_hp)}"
            m.text = f"cf_{unit_id+1} [{faction_str}]\nHP:{hp_str}"

            if unit.faction.name == "ALLY":
                m.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
            else:
                m.color = ColorRGBA(r=1.0, g=0.3, b=0.2, a=1.0)

            m.lifetime.sec = 2
            array.markers.append(m)

        if array.markers:
            self.unit_info_publisher.publish(array)

    def _publish_static_map(self):
        """Publish map boundary wireframe and obstacle cubes (1 Hz, persistent)."""
        now = self.get_clock().now().to_msg()
        self._publish_map_boundary(now)
        self._publish_smac_obstacles(now)
        self._publish_unit_info()

    def _publish_map_boundary(self, now):
        """Publish a wireframe box showing the map extents."""
        l = float(self.map_info.length)
        w = float(self.map_info.width)
        h = float(self.map_info.height)
        hl, hw, hh = l / 2.0, w / 2.0, h / 2.0

        corners = [
            (-hl, -hw, 0.0), ( hl, -hw, 0.0),
            ( hl,  hw, 0.0), (-hl,  hw, 0.0),
            (-hl, -hw,  h ), ( hl, -hw,  h ),
            ( hl,  hw,  h ), (-hl,  hw,  h ),
        ]
        edges = [
            (0,1),(1,2),(2,3),(3,0),  # bottom face
            (4,5),(5,6),(6,7),(7,4),  # top face
            (0,4),(1,5),(2,6),(3,7),  # verticals
        ]

        m = Marker()
        m.header.frame_id = "/world"
        m.header.stamp = now
        m.ns = "map_boundary"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.6, g=0.6, b=1.0, a=0.4)
        m.lifetime.sec = 3

        for a, b in edges:
            pa, pb = corners[a], corners[b]
            m.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
            m.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))

        self.map_boundary_publisher.publish(m)

    def _publish_smac_obstacles(self, now):
        """Publish render_obs from the loaded map as transparent cube markers."""
        if not self.map_info.render_obs:
            return

        array = MarkerArray()
        for idx, obs in enumerate(self.map_info.render_obs):
            m = Marker()
            m.header.frame_id = "/world"
            m.header.stamp = now
            m.ns = "smac_obs"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = float(obs['pos'][0])
            m.pose.position.y = float(obs['pos'][1])
            m.pose.position.z = float(obs['pos'][2])
            quat = obs.get('ori', [0, 0, 0, 1])
            m.pose.orientation.x = float(quat[0])
            m.pose.orientation.y = float(quat[1])
            m.pose.orientation.z = float(quat[2])
            m.pose.orientation.w = float(quat[3])

            size = obs['size']
            m.scale.x = float(size[0])
            m.scale.y = float(size[1])
            m.scale.z = float(size[2])

            m.color = ColorRGBA(r=0.7, g=0.8, b=1.0, a=0.18)
            m.lifetime.sec = 3
            array.markers.append(m)

        if array.markers:
            self.smac_obstacles_publisher.publish(array)

    def _check_collisions(self):
        """Kill any live unit that has collided with a boundary wall or internal obstacle."""
        if self._game_over:
            return

        half_l = float(self.map_info.length) / 2.0
        half_w = float(self.map_info.width)  / 2.0
        drone_r = 0.15  # drone radius (m)

        for unit in list(self.all_units.values()):
            if unit.hp <= 0:
                continue
            pos = unit.get_pos()
            if pos is None:
                continue

            hit = False
            reason = ""

            # ── Map boundary (inner face at ±half, drone touches at ±(half−r)) ──
            if abs(pos[0]) >= half_l - drone_r or abs(pos[1]) >= half_w - drone_r:
                hit = True
                reason = "boundary wall"

            # ── Internal obstacles: oriented-bounding-box check on render_obs ──
            # render_obs has only ~9 entries vs 627 maca_obs cubes, and each
            # carries the exact shape/orientation from the JSON preset.
            if not hit:
                for obs in self.map_info.render_obs:
                    center   = np.array(obs['pos'],  dtype=float)
                    half_sz  = np.array(obs['size'], dtype=float) / 2.0 + drone_r
                    quat     = obs.get('ori', [0, 0, 0, 1])  # [qx,qy,qz,qw]
                    # Transform pos to obstacle local frame
                    local = ScipyR.from_quat(quat).inv().apply(pos - center)
                    if np.all(np.abs(local) <= half_sz):
                        hit = True
                        reason = "internal obstacle"
                        break

            if hit:
                unit.hp = 0
                key = self.cf_prefix + str(unit.id + 1)
                self.get_logger().info(f"{key} killed by {reason} collision")
                if unit.id not in self.unit_cmd_queues:
                    self._trigger_exit_sequence(unit)

    def _check_win_loss(self):
        if not self.start_mission or self._game_over:
            return
        live_enemies = [u for u in self.all_units.values()
                        if u.faction != Faction.ALLY and u.hp > 0]
        live_allies  = [u for u in self.all_units.values()
                        if u.faction == Faction.ALLY and u.hp > 0]
        if not live_enemies:
            self._end_game("ALLIES WIN")
        elif not live_allies:
            self._end_game("ENEMIES WIN")

    def _end_game(self, result: str):
        self._game_over = True
        self._waiting_for_landing = True
        self.get_logger().info(f"=== GAME OVER: {result} — waiting for all units to land ===")
        # Trigger landing for any live unit not already in an exit sequence.
        for unit_id, unit in list(self.all_units.items()):
            if unit.hp > 0 and unit_id not in self.unit_cmd_queues:
                self._trigger_exit_sequence(unit)
        self.start_mission = False

    # ------------------------------------------------------------------ #

    def _start_combat(self):
        pos = np.array(self.map_info.attack_point)
        # attack_point may be a flat [x,y,z] shared by all units, or [[x,y,z],...] per unit.
        def get_attack_pos(idx):
            return pos if pos.ndim == 1 else pos[idx]

        # Notify handlers so they can set initial per-unit state (e.g. LED ring mode).
        for unit in self.all_units.values():
            if unit.hp > 0:
                for handler in self._hp_event_handlers:
                    handler.on_init(unit)

        # Allies advance toward the center of the enemy deployment area.
        enemy_center = np.mean(self.initial_poses[self.n_agents:, :3], axis=0)
        for i, ally in self.agents.items():
            if ally.hp == 0:
                continue
            ally.command = AttackMoveCommand(enemy_center, targets=self.enemies.values())

        for i, enemy in self.enemies.items():
            if enemy.hp == 0:
                continue
            enemy.command = AttackMoveCommand(get_attack_pos(i), targets=self.agents.values())        


    def _find_safe_zone(self, unit) -> np.ndarray:
        """Return a landing spot near the nearest map boundary wall (inside the arena).
        The dead unit retreats to the closest edge, clear of combat, without
        needing to cross any boundary obstacle."""
        pos = unit.get_pos().copy()
        half_l = float(self.map_info.length) / 2.0
        half_w = float(self.map_info.width)  / 2.0
        margin = 0.35  # stay this far inside the inner wall face

        # Distances from current XY position to each of the four inner wall faces.
        d_px =  half_l - pos[0]   # to +X wall
        d_nx =  pos[0] + half_l   # to -X wall
        d_py =  half_w - pos[1]   # to +Y wall
        d_ny =  pos[1] + half_w   # to -Y wall

        min_d = min(d_px, d_nx, d_py, d_ny)
        if min_d == d_px:
            safe = np.array([ half_l - margin, pos[1], pos[2]])
        elif min_d == d_nx:
            safe = np.array([-half_l + margin, pos[1], pos[2]])
        elif min_d == d_py:
            safe = np.array([pos[0],  half_w - margin, pos[2]])
        else:
            safe = np.array([pos[0], -half_w + margin, pos[2]])

        # Keep height in the drone's normal flight band.
        safe[2] = float(np.clip(pos[2],
                                float(self.map_info.height) * 0.15,
                                float(self.map_info.height) * 0.55))
        return safe

    def _trigger_exit_sequence(self, unit) -> None:
        safe_pos = self._find_safe_zone(unit)
        key = self.cf_prefix + str(unit.id + 1)
        self.unit_cmd_queues[unit.id] = deque([
            Commander(task="goto_velocity", cont="wait", agents=[key],
                      target=[float(safe_pos[0]), float(safe_pos[1]),
                               float(safe_pos[2]), 0.0],
                      sent_mission=False),
            Commander(task="hold", cont="wait", agents=[key],
                      duration=1.0, sent_mission=False),
            Commander(task="land", cont="wait", agents=[key],
                      sent_mission=False),
        ])
        # Remove from combat factions so live units stop targeting it.
        if unit.faction == Faction.ALLY:
            self.agents.pop(unit.id_in_faction, None)
        else:
            self.enemies.pop(unit.id_in_faction, None)
        self.get_logger().info(
            f"Unit {key} hp=0, exit sequence: safe_zone={safe_pos.round(2)}")

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
        for obs in msg.pre_obs :
            agent = obs.agent
            tmp_id = key_to_id(agent.id)
            tmp_unit = self.all_units.get(tmp_id)
            if tmp_unit is not None : 
                if agent.flight_state == FlightState.EMERGENCY.value :
                    # Unit may already be removed from faction dicts by _trigger_exit_sequence;
                    # use pop() so a second removal is a no-op rather than a KeyError.
                    if tmp_unit.faction == Faction.ALLY :
                        self.agents.pop(tmp_unit.id_in_faction, None)
                    else :
                        self.enemies.pop(tmp_unit.id_in_faction, None)
                    self.all_units.pop(tmp_unit.id, None)
                    self.unit_cmd_queues.pop(tmp_unit.id, None)
                    self.visible_enemy_cache.pop(tmp_id, None)
                else : 
                    tmp_unit.MACA_agent.flight_state = FlightState(agent.flight_state)
                    tmp_unit.MACA_agent.radio_connection = agent.connected
                    tmp_unit.MACA_agent.completed = agent.completed
                    self.visible_enemy_cache[tmp_id] = [
                        (int(n.id), float(n.distance)) for n in obs.visible_enemy
                    ]

        # check agent's states
        agent_remove_list = []
        for id, unit in self.all_units.items():
            if not unit.MACA_agent.radio_connection:
                agent_remove_list.append(id)

        for i in agent_remove_list : 
            tmp_unit = self.all_units.get(i)
            if tmp_unit is not None : 
                if tmp_unit.faction == Faction.ALLY :
                    self.agents.pop(tmp_unit.id_in_faction, None)
                else : 
                    self.enemies.pop(tmp_unit.id_in_faction, None)
                self.all_units.pop(tmp_unit.id, None)
                self.visible_enemy_cache.pop(tmp_unit.id, None)

        if not self.housekeeping_done:
            return
            
        # command handler
        if len(self.commander_buffer) > 0 : 
            success_count = 0 
            for cmd in self.commander_buffer : 
                # hold
                if cmd.task == "hold":
                    duration_seconds = (self.get_clock().now() - self.last_mission_time).nanoseconds / 1e9
                    if cmd.duration < duration_seconds:
                        self.get_logger().debug("hold done {:.3}/{:.3}s".format(duration_seconds, cmd.duration))
                        success_count += 1
                    else:
                        self.get_logger().debug("holding {:.3}/{:.3}s".format(duration_seconds, cmd.duration))
                        return
                # goto_velocity 
                elif cmd.task == "goto_velocity" : 
                    duration_seconds = (self.get_clock().now() - self.goto_velocity_command_time).nanoseconds / 1e9
                    if duration_seconds > self.move_land_tolerance :
                        if self.check_completed_agents(cmd) :
                            success_count+=1 
                else :
                    if self.check_completed_agents(cmd) :
                        success_count+=1
            
            self.get_logger().debug(f"bootstrap progress {success_count}/{len(self.commander_buffer)}")
            
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
            if not self.start_mission:
                self.start_mission = True
                self._start_combat()
                self.get_logger().info("=== ALL UNITS FINISHED TAKEOFF — combat started ===")
            return

        # fill user_buffer
        for cmd in self.commander_buffer : 
            if cmd.sent_mission : 
                continue

            if len(cmd.agents) == 0 : 
                self.raise_error("empty agent list in command")
                return

            if cmd.task == "hold":
                cmd.sent_mission = True
                continue

            msg = cmd.convert_to_msg((self.cf_prefix + str(i + 1) for i in self.all_units.keys()))
            if msg is None:
                cmd.sent_mission = True
                continue

            if isinstance(msg, list):
                for m in msg:
                    self.command_publisher.publish(m)
            else:
                self.command_publisher.publish(msg)

            if cmd.task == "goto_velocity":
                self.goto_velocity_command_time = self.get_clock().now()
            cmd.sent_mission = True

    def _quat_to_rpy(self, x, y, z, w):
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = np.sign(sinp) * (np.pi / 2.0)
        else:
            pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def pose_state_callback(self, msg, agent_id):
        idx = key_to_id(agent_id)
        if idx < 0 or idx >= self.NUM_DRONES:
            return

        p = msg.pose.position
        q = msg.pose.orientation
        self.pos[idx, :] = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.quat[idx, :] = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self.rpy[idx, :] = np.array([roll, pitch, yaw], dtype=np.float64)

        unit = self.all_units.get(idx)
        if unit is not None:
            unit.MACA_agent.pos_global_frame = self.pos[idx, :].copy()
            # unit command logic expects heading[0] to be yaw
            unit.MACA_agent.heading_global_frame = np.array([yaw, pitch, roll], dtype=np.float64)

    def vel_state_callback(self, msg, agent_id):
        idx = key_to_id(agent_id)
        if idx < 0 or idx >= self.NUM_DRONES:
            return

        self.vel[idx, :] = np.array(
            [msg.linear.x, msg.linear.y, msg.linear.z], dtype=np.float64)
        self.ang_v[idx, :] = np.array(
            [msg.angular.x, msg.angular.y, msg.angular.z], dtype=np.float64)

        unit = self.all_units.get(idx)
        if unit is not None:
            unit.MACA_agent.vel_global_frame = self.vel[idx, :].copy()

    # Backward-compatible alias used by housekeeping()
    def listener_callback(self, msg, agent_id):
        self.pose_state_callback(msg, agent_id)
    
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

    def _notify_hp_event(self, unit, shooter_unit,
                         old_hp: float, new_hp: float) -> None:
        """Broadcast a hit event to all registered HP event handlers."""
        for handler in self._hp_event_handlers:
            handler.on_hit(unit, shooter_unit, old_hp, new_hp)

    def _attack_hit_callback(self, msg: UserCommand):
        """C++ raycasting determined which unit was actually struck.
        Apply one shot of damage to that unit; notify HP handlers; if HP
        reaches 0 trigger exit and confirm the kill to C++ so it sets
        mission_capable=false and initiates landing."""
        if len(msg.uav_id) < 2:
            return
        shooter_key = msg.uav_id[0]
        hit_key     = msg.uav_id[1]

        shooter_unit = self.all_units.get(key_to_id(shooter_key))
        hit_unit     = self.all_units.get(key_to_id(hit_key))

        if shooter_unit is None or hit_unit is None:
            return
        if hit_unit.hp <= 0:
            return

        old_hp = hit_unit.hp
        shooter_unit.deal_damage(hit_unit)
        new_hp = hit_unit.hp
        self.get_logger().info(
            f"{shooter_key} hits {hit_key}: hp {old_hp:.0f} → {new_hp:.0f}")

        self._notify_hp_event(hit_unit, shooter_unit, old_hp, new_hp)

        if hit_unit.hp <= 0 and hit_unit.id not in self.unit_cmd_queues:
            self.get_logger().info(f"{hit_key} dead (hp=0), confirming kill to C++")
            for handler in self._hp_event_handlers:
                handler.on_death(hit_unit)
            self._trigger_exit_sequence(hit_unit)
            kill_cmd = UserCommand()
            kill_cmd.cmd = "kill"
            kill_cmd.uav_id = [hit_key]
            self.command_publisher.publish(kill_cmd)

    def _main_loop(self):
        # ── Combat block (skipped after game over) ──────────────────────
        if self.start_mission:
            self._set_unit_command(None)
            unit_ids = list(self.all_units.keys())
            self._np_random.shuffle(unit_ids)

            self.command_publish_queue.clear()
            for unit_id in unit_ids:
                unit = self.all_units[unit_id]
                if unit.hp <= 0:
                    continue
                if unit.command is None:
                    continue

                unit.prepare_velocity()
                cmd = self._build_prepared_command(unit)
                if cmd is not None:
                    self.command_publish_queue.append(cmd)
                
                # C++ do_laser_action handles actual hit resolution (raycasting).
                # Pass hit_unit=None so game_step only ticks cooldown/state,
                # not HP — the Python HP model is updated via mission_capable feedback.
                unit.game_step(hit_unit=None)

            for cmd in self.command_publish_queue:
                self.command_publisher.publish(cmd)

            self._check_collisions()
            self._check_win_loss()

        elif not self._waiting_for_landing:
            return  # nothing to do before mission starts

        # ── Exit-sequence block (always runs while units are landing) ───
        all_keys = [self.cf_prefix + str(i + 1) for i in self.all_units.keys()]
        for unit_id in list(self.unit_cmd_queues.keys()):
            q = self.unit_cmd_queues[unit_id]
            if not q:
                del self.unit_cmd_queues[unit_id]
                self._exit_hold_start.pop(unit_id, None)
                continue

            step = q[0]

            if not step.sent_mission:
                if step.task == "land":
                    self.get_logger().info(
                        f"{self.cf_prefix + str(unit_id + 1)} landing")
                msgs = step.convert_to_msg(all_keys, is_external=True)
                if msgs is not None:
                    for m in (msgs if isinstance(msgs, list) else [msgs]):
                        self.command_publisher.publish(m)
                if step.task == "hold":
                    self._exit_hold_start[unit_id] = self.get_clock().now()
                step.sent_mission = True

            if step.task == "hold":
                start = self._exit_hold_start.get(unit_id)
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9 if start else 0.0
                if elapsed >= step.duration:
                    q.popleft()
            else:
                unit = self.all_units.get(unit_id)
                if unit is not None and unit.MACA_agent.completed:
                    q.popleft()

        # ── Shutdown once every unit has landed ─────────────────────────
        if self._waiting_for_landing and not self.unit_cmd_queues:
            all_idle = all(
                u.MACA_agent.flight_state in (FlightState.IDLE, FlightState.LAND)
                for u in self.all_units.values()
            )
            if all_idle or not self.all_units:
                self.get_logger().info("All units landed. Shutting down node.")
                import rclpy
                rclpy.shutdown()

    def _build_prepared_command(self, unit):
        shooter = self.cf_prefix + str(unit.id + 1)

        # Attack command is emitted when AttackUnitCommand marks attacking=True and
        # cooldown has expired — prevents firing faster than the smaclike model allows.
        if unit.attacking and unit.target is not None and unit.target.hp > 0 and unit.cooldown < 1e-7:
            cmd = UserCommand()
            cmd.cmd = "attack"
            target_key = self.cf_prefix + str(unit.target.id + 1)
            cmd.uav_id = [shooter, target_key]
            tpos = unit.target.get_pos()
            cmd.goal.x = float(tpos[0]); cmd.goal.y = float(tpos[1]); cmd.goal.z = float(tpos[2])
            return cmd

        pref_vel = np.array(unit.pref_velocity[:3], dtype=np.float64)
        pref_yaw = np.array(unit.pref_yaw[:3], dtype=np.float64)
        vel_norm = np.linalg.norm(pref_vel)
        yaw_rate = float(pref_yaw[0]) if pref_yaw.size > 0 else 0.0

        # Turn-in-place: send yaw_rate (rad/s) explicitly — no target_yaw estimation.
        # The firmware's MOVE_VELOCITY case uses agent.yaw_rate directly when non-zero.
        if vel_norm < 1e-4 and abs(yaw_rate) > 1e-4:
            pos = unit.get_pos()
            cmd = UserCommand()
            cmd.cmd = "goto_velocity"
            cmd.uav_id = [shooter]
            cmd.goal.x = float(pos[0])
            cmd.goal.y = float(pos[1])
            cmd.goal.z = float(pos[2])
            cmd.yaw_rate = float(yaw_rate)
            cmd.is_external = True
            return cmd

        # No motion and no yaw: nothing to publish this tick.
        if vel_norm < 1e-4:
            return None

        # Linear movement: short-horizon goal, hold current heading (yaw_rate=0).
        cur_yaw = float(unit.get_heading()[0]) if unit.get_heading() is not None else 0.0
        step_goal = unit.get_pos() + pref_vel * self.control_dt * STEP_MUL
        cmd = UserCommand()
        cmd.cmd = "goto_velocity"
        cmd.uav_id = [shooter]
        cmd.goal.x = float(step_goal[0])
        cmd.goal.y = float(step_goal[1])
        cmd.goal.z = float(step_goal[2])
        cmd.yaw = float(np.degrees(cur_yaw))
        cmd.is_external = True
        return cmd

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
        
        for unit in self.all_units.values():
            if unit.command is not None:
                unit.clean_up_target()
        # #print("targets : ", [unit.target for unit in self.all_units.values()])

        # Use visibility already computed by underlying planning node.
        for unit in self.all_units.values():
            if unit.target is not None:
                continue

            potentials = []
            for neighbor_ros_id, dist in self.visible_enemy_cache.get(unit.id, []):
                target_unit = self.all_units.get(neighbor_ros_id - 1)
                if target_unit is None:
                    continue
                if target_unit.hp <= 0:
                    continue
                if target_unit.faction == unit.faction:
                    continue
                potentials.append((target_unit, dist))
            unit.potential_targets = potentials

        # preserve "retaliation" bias used by AttackMoveCommand.
        for unit in self.all_units.values():
            if unit.target is not None and unit.target.hp > 0:
                unit.target.potential_targets.append((unit, 2e9))

    '''
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
        # Laser hit resolution is handled by crazyswarm_application attack command.
        # Keep local reward path no-op for now.
        return None
    '''

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
