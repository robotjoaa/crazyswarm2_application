import numpy as np
import smaclike.units.unit_type as ut
from smaclike.units.combat_type import CombatType
from smaclike.util import point_inside_sphere, point_inside_cone
from smaclike.util.faction import Faction

from smaclike.mamp.agents.agent import Agent
from smaclike.mamp.policies.orca3dPolicy import ORCA3DPolicy
from smaclike.mamp.configs.config import DT
#from smaclike.units.unit_command import TurnCommand
REWARD_KILL = 10
NEG_SCALE = 0.5
TICKS_PER_SECOND = 16
GAME_TICK_TIME = 1 / TICKS_PER_SECOND

HEAL_PER_SECOND = 9
HEAL_PER_ENERGY = 3
ENERGY_PER_SECOND = 0.5625


class Unit(object):
    def __init__(self, unit_type: ut.UnitType, faction: Faction,
                 pos, idd: int, idd_in_faction: int, ctrl_freq: int) -> None:
        ### related to mamp ### 
        '''
        self.pos = np.array([x, y, z], dtype=np.float32)
        self.velocity = np.array([0, 0, 0], dtype=np.float32)
        self.next_velocity: np.ndarray = None
        self.pref_velocity: np.ndarray = None
        self.max_velocity = unit_type.stats.speed
        self.size = unit_type.stats.size
        self.radius = unit_type.radius
        self.radius_sq = self.radius ** 2
        '''
        
        ## MACA
        self.MACA_dt = 1/ctrl_freq
        
        init_vel = [0.0, 0.0, 0.0]
        self.size = unit_type.stats.size
        self.radius = unit_type.radius
        self.radius_sq = self.radius ** 2
        self.fov_range = np.array([np.pi /3, np.pi /3])
        ## at the beginning, goal == start
        ## when move_command, set new goal 
        self.pref_speed = unit_type.stats.speed # max velocity
        self.pref_velocity = np.zeros(3)
        self.pref_yaw = np.zeros(3)
        self.max_velocity = unit_type.stats.speed
        self.max_ang_vel  = unit_type.stats.ang_speed
        self.MACA_agent = Agent(start_pos=pos, goal_pos=pos,
                            vel=init_vel, radius=unit_type.radius,
                            pref_speed=self.pref_speed, policy=ORCA3DPolicy,
                            id=idd, faction=faction, dt=self.MACA_dt)
        
        
        ## need to update after pybullet simulation
        #self.pos = np.array(pos[:3], dtype='float64')
        #self.velocity = np.array(init_vel, dtype='float64')
        #self.heading = np.array(pos[3:], dtype='float64')
        
        ### related to smac ###    
        self.id = idd
        self.id_in_faction = idd_in_faction
        self.type = unit_type
        self.max_hp = unit_type.stats.hp
        self.max_energy = unit_type.stats.energy
        self.hp = self.max_hp
        self.energy = unit_type.stats.starting_energy
        self.healing_available = 0
        self.faction = faction
       
        self.cooldown = 0
        self.command = None
        self.target: 'Unit' = None
        self.max_cooldown = unit_type.stats.cooldown
        self.attack_range = unit_type.stats.attack_range
        self.minimum_scan_range = unit_type.stats.minimum_scan_range
        self.damage = unit_type.stats.damage
        

        self.combat_type = unit_type.stats.combat_type
        self.attacking = False
        self.targeter = ut.TARGETER_CACHE[self.type.stats.name]
        # Used for the purpose of attack-moving
        self.potential_targets = []
        #self.priority_targets = []
        self.prev_target: 'Unit' = None
        self.hit = False

    def clean_up_target(self):
        self.hit = False
        self.potential_targets = []
        #self.priority_targets = []
        self.prev_target = self.target
        self.command.clean_up_target(self)

    def prepare_velocity(self):
        pref_vel = self.command.prepare_velocity(self)
        self.pref_velocity = pref_vel[:3]
        self.pref_yaw = pref_vel[3:]
        #print("unit ",self.id,", pref vel :",pref_vel)

    def update_agent(self, start_pos, goal_pos, vel, col):
        # start_pos : position + heading
        # goal_pos : position + heading 
        return self.MACA_agent.updateAgent(start_pos, goal_pos, vel, col)

    def get_pos(self):
        return self.MACA_agent.pos_global_frame
        
    def get_vel(self):
        return self.MACA_agent.vel_global_frame
        
    def get_heading(self):
        return self.MACA_agent.heading_global_frame

    def game_step(self, **kwargs):
        if self.hp == 0:
            return 0
        
        ### update unit, agent status according to pybullet ### 

        # attack cooldown
        if self.cooldown > 0:
            self.__decrease_cooldown()
        
        # currently don't use energy
        # if self.energy < self.max_energy:
        #     self.energy = min(self.energy +
        #                       ENERGY_PER_SECOND / TICKS_PER_SECOND,
        #                       self.max_energy)
            
        return self.command.execute(self, **kwargs)

    def has_within_attack_range(self, target: 'Unit'):
        #radius = self.attack_range + target.radius + self.radius
        radius = self.attack_range
        return point_inside_sphere(self.get_pos(), target.get_pos(), radius)

    # whether unit has to turn to attack
    # need to visualize cone
    def has_within_attack_angle(self, target: 'Unit'):
        radius = self.attack_range
        return point_inside_cone(self.get_pos(), target.get_pos(), self.get_heading(), self.fov_range, [target.radius, radius])

    def has_within_scan_range(self, target: 'Unit'):
        radius = self.minimum_scan_range + target.radius
        return point_inside_sphere(self.get_pos(), target.get_pos(), radius)

    # def heal(self, target: 'Unit'):
    #     if self.combat_type != CombatType.HEALING:
    #         raise ValueError("Can't heal with this unit.")
    #     if self.prev_target != self.target and self.energy < 5 \
    #             or target.hp == target.max_hp:
    #         self.target = None
    #         return
    #     target_hp_missing = target.max_hp - target.hp
    #     desired_heal_amount = min(target_hp_missing,
    #                               HEAL_PER_SECOND / TICKS_PER_SECOND)
    #     if self.healing_available < desired_heal_amount:
    #         energy_spent = min(1, self.energy)
    #         self.energy -= energy_spent
    #         self.healing_available += energy_spent * HEAL_PER_ENERGY
    #     heal_amount = min(desired_heal_amount,
    #                       self.healing_available)
    #     self.healing_available -= heal_amount
    #     target.hp += heal_amount

    def deal_damage(self, target: 'Unit', is_dense=True, only_positive=True) -> float:
        damage = self.damage
        #print("unit {} deal damage to {}".format(self.id, target.id))
        tmp = target.take_damage(damage, is_dense, only_positive)
        return tmp

    def take_damage(self, amount, is_dense, only_positive) -> float:
        #print("take damage :",self.id)
        only_positive = only_positive if is_dense else False
        self.hit = True
        if self.hp == 0:
            return 0

        reward = 0
        # when dense reward, accumulate damage as reward
        amount_dealt = max(0, min(amount, self.hp))
        self.hp -= amount_dealt

        if is_dense :
            reward += amount_dealt
        
        if self.hp == 0:
            reward += REWARD_KILL # reward kill
            
        if self.faction == Faction.ALLY:
            # No rewards for allies taking damage
            if only_positive : 
                return 0 
            else : 
                return -1 * reward * NEG_SCALE
        else : 
            return reward

    def __decrease_cooldown(self) -> None:
        if self.cooldown > 0:
            #self.cooldown = max(0, self.cooldown - GAME_TICK_TIME)
            self.cooldown = max(0, self.cooldown - self.MACA_dt)

    def __hash__(self) -> int:
        return hash(self.id)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Unit) and self.id == other.id
