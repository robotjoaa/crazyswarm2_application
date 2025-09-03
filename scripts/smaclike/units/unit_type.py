import numpy as np
import os
from enum import Enum
from typing import Dict, Set
from dataclasses import dataclass
import smaclike.units.unit_type as ut
from smaclike.units.combat_type import CombatType
#from smaclike.util import point_inside_circle
from smaclike.util.faction import Faction
import json

import smaclike.units.targeters.targeter as t

TARGETER_CACHE: Dict[str, t.Targeter] = {}

@dataclass
class UnitStats(object):
    name: str
    hp: int
    damage: int
    cooldown: float
    speed: float
    ang_speed: float
    attack_range: int
    size: float
    energy: int = 0
    starting_energy: int = 0
    combat_type: CombatType = CombatType.DAMAGE
    minimum_scan_range: int = 2

    @classmethod
    def from_file(cls, filename, custom_unit_path):
        if not os.path.isabs(filename):
            filename = os.path.join(os.path.abspath(custom_unit_path),
                                    filename)
        if not filename.endswith(".json"):
            filename += ".json"
        with open(filename) as f:
            stats_dict = json.load(f)
        stats_dict['name'] = os.path.splitext(os.path.basename(filename))[0]
        
        if 'combat_type' in stats_dict:
            stats_dict['combat_type'] = CombatType(stats_dict['combat_type'])
            
        targeter_kwargs = stats_dict.pop('targeter_kwargs', {})
        TARGETER_CACHE[stats_dict['name']] = \
            t.TargeterType[stats_dict.pop(
                'targeter', 'STANDARD')].value(**targeter_kwargs)
        return cls(**stats_dict)


class UnitType(object):
    @property
    def stats(self) -> UnitStats:
        raise NotImplementedError

    @property
    def radius(self):
        return self.stats.size / 2

    @property
    def size(self):
        return self.stats.size
    
    @property
    def combat_type(self):
        return self.stats.combat_type

    @classmethod
    def from_str(cls, s):
        return STANDARD_UNIT_TYPES[s]


STANDARD_UNIT_PATH = os.path.join(os.path.dirname(__file__), "unit_config")


class StandardUnit(UnitType, Enum):
    @property
    def stats(self) -> UnitStats:
        return self.value

    #default
    DRONE = UnitStats.from_file("drone", STANDARD_UNIT_PATH)
    WEAK_DRONE = UnitStats.from_file("weak_drone", STANDARD_UNIT_PATH)
    STRONG_DRONE = UnitStats.from_file("strong_drone", STANDARD_UNIT_PATH)

STANDARD_UNIT_TYPES = {unit_type.name: unit_type
                       for unit_type in StandardUnit}