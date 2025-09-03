from typing import List

import numpy as np
from smaclike.units.combat_type import CombatType
from smaclike.units.unit import Unit


class Command:
    def clean_up_target(self, unit: Unit) -> None:
        """Remove the target from the unit if it is not valid anymore,
        e.g. it's too far away or it's dead or this command type
        doesn't support having a target.

        Args:
            unit (Unit): the unit to clean up for.
        """
        raise NotImplementedError

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        """Prepare the velocity for the unit in the current game step.
        This velocity will then be submitted to the collision avoidance
        algorithm and adjusted accordingly.

        Args:
            unit (Unit): the unit to prepare velocity for

        Returns:
            np.ndarray: [x, y, z] vector with the prepared velocity
        """
        raise NotImplementedError

    def execute(self, unit: Unit, **kwargs) -> float:
        """Execute the command using the unit

        Args:
            unit (Unit): the unit that should execute the command

        Returns:
            float: the reward obtained by the unit in the step
        """
        raise NotImplementedError


class AttackUnitCommand(Command):
    def __init__(self, target: Unit):
        self.target = target

    def clean_up_target(self, unit: Unit) -> None:
        assert self.target.hp >= 0
        unit.target = self.target if self.target.hp > 0 else None

    def prepare_velocity(self, unit: Unit) -> None:
        #print("unit_",unit.id,",AttackUnitCommand")
        # Target too far away
        if not unit.has_within_attack_range(self.target) :
            return MoveCommand(self.target.get_pos()).prepare_velocity(unit)
        
        # Target not in the attack angle
        res, azi_diff, ele_diff = unit.has_within_attack_angle(self.target)
        if not res : 
            dpos = self.target.get_pos() - unit.get_pos()
            # distance is too far or too close
            if ele_diff is None : 
                new_pos = unit.get_pos() + azi_diff * dpos / np.linalg.norm(dpos)
                return MoveCommand(new_pos).prepare_velocity(unit)
            # if elevation difference is large, adjust height
            if ele_diff != 0 : 
                new_pos = unit.get_pos() + ele_diff*np.array([0,0,1])
                return MoveCommand(new_pos).prepare_velocity(unit)
            # if yaw difference is large, adjust yaw
            if azi_diff != 0 : 
                dyaw = np.arctan2(dpos[1],dpos[0]) - unit.get_heading()[0]
                dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
                if dyaw < -unit.max_ang_vel / 2 :
                    sign = -1
                elif dyaw > unit.max_ang_vel / 2 :
                    sign = 1
                else :  
                    sign = 0
                return TurnCommand(sign).prepare_velocity(unit)
        #if res :
        #    print("unit {}: target {} within attack angle".format(unit.id,self.target.id))
        # Target is in range
        unit.attacking = True
        return np.zeros(6)

    def execute(self, unit: Unit, **kwargs) -> float:
        if not unit.attacking:
            return MoveCommand(self.target.get_pos()).execute(unit, **kwargs)
        unit.attacking = False
        if unit.cooldown > 0:
            return 0
        unit.cooldown = unit.max_cooldown
        hit_unit = kwargs.get('hit_unit', None)
        
        if hit_unit : 
            return unit.targeter.target(unit, hit_unit, **kwargs)

        return 0


class MoveCommand(Command):
    def __init__(self, pos: np.ndarray):
        self.pos = pos

    def clean_up_target(self, unit: Unit) -> None:
        unit.target = None

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        #print("unit_",unit.id,",MoveCommand")
        if unit.max_velocity == 0:
            return np.zeros(6)
        max_velocity = unit.max_velocity
        dpos = self.pos - unit.get_pos()
        distance = np.linalg.norm(dpos)
        return np.zeros(6) if distance == 0 \
            else np.hstack((dpos * max_velocity  / distance, [0,0,0]))

    def execute(self, unit: Unit, **kwargs) -> float:
        return 0

class TurnCommand(Command):
    # turn clockwise, counter clockwise
    def __init__(self, sign):
        # target position
        self.sign = sign

    def clean_up_target(self, unit: Unit) -> None:
        unit.target = None

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        #print("unit_",unit.id,",TurnCommand")
        ## unit yaw
        #cur_yaw = unit.get_heading()[0]
        #target_yaw = np.arctan2(dpos[1], dpos[2]) 
        #sign1 = 1 if dyaw1 > 0 else -1
        #sign2 = 1 if dyaw2 > 0 else -1
        #dyaw1 = abs(target_yaw - cur_yaw)
        #dyaw2 = 2 * np.pi - abs(dyaw1)
        #if abs(dyaw1) == 0 or abs(dyaw2) == 0 : 
        #    return np.zeros(3) 

        #if abs(dyaw1) >= abs(dyaw2) : 
        #    dyaw = sign1 * min(unit.max_ang_vel, dyaw1)
        #else :  
        #    dyaw = sign2 * min(unit.max_ang_vel, dyaw2)
        dyaw = self.sign * unit.max_ang_vel 
        return np.array([0,0,0,dyaw, 0, 0])

    def execute(self, unit: Unit, **kwargs) -> float:
        return 0

class AttackMoveCommand(Command):
    def __init__(self, pos: np.ndarray, targets: List[Unit]):
        ## target position 
        self.pos = pos
        self.move_command = MoveCommand(pos)

    def clean_up_target(self, unit: Unit) -> None:
        if unit.target is None:
            # No target to lose.
            return
        if unit.target.hp == 0:
            # Always lose a dead target.
            unit.target = None
            return
        if unit.target.target == unit:
            # Don't lose an alive target who's attacking you.
            return
        if not unit.has_within_attack_range(unit.target):
            # Otherwise lose the target if it's not in range.
            unit.target = None

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        #print("unit_",unit.id,",AttackMoveCommand")
        if unit.target is None:
            closest_target = self.__pick_target_damage(unit)
            if closest_target is None:
                return self.move_command.prepare_velocity(unit)
            
            unit.target = closest_target
        
        #if unit.has_within_attack_range(unit.target) : 
        return AttackUnitCommand(unit.target).prepare_velocity(unit)
        

    def execute(self, unit: Unit, **kwargs) -> float:
        if unit.target is None:
            return self.move_command.execute(unit, **kwargs)
        return AttackUnitCommand(unit.target).execute(unit, **kwargs)

    def __pick_target_damage(self, unit: Unit) -> Unit:
        tgt = unit.potential_targets
        # choose one in attack_range (don't have to turn) with minimum distance
        candidate = min(((u, d) for u, d in tgt
                         if u.hp > 0
                         and (d < unit.minimum_scan_range + u.radius
                              or u.target == unit)),
                        key=lambda p: (0 if unit.has_within_attack_range(p[0]) else 1,
                                       p[1]), default=None)
        return candidate[0] if candidate is not None else None

class StopCommand(Command):
    def clean_up_target(self, unit: Unit) -> None:
        unit.target = None

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        return np.zeros(6)

    def execute(self, unit: Unit, **kwargs) -> float:
        unit.target = None
        return 0


class NoopCommand(Command):
    def clean_up_target(self, unit: Unit) -> None:
        unit.target = None

    def prepare_velocity(self, unit: Unit) -> np.ndarray:
        return np.zeros(6)

    def execute(self, unit: Unit, **kwargs) -> None:
        assert unit.hp == 0, f"Unit's hp is not 0: {unit.hp}"
        return 0
