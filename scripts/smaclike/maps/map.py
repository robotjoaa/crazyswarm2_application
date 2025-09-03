#from smaclike.maps.map import Group, MapInfo, MapPreset

import json
from dataclasses import dataclass
from enum import Enum
import os
from typing import Dict, List, Tuple

from smaclike.units.unit_type import StandardUnit, UnitType
from smaclike.util.faction import Faction

from smaclike.obstacle.obstacle import OBSTACLE_PRESETS, Obstacle

@dataclass
class Group(object):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    faction: Faction
    units: List[Tuple[UnitType, int]]

@dataclass
class MapInfo(object):
    name: str
    num_allied_units: int
    num_enemy_units: int
    groups: List[Group]
    attack_point: List[Tuple[float, float, float]]
    length: int 
    width: int 
    height: int
    maca_obs: List[Obstacle]
    render_obs: List[Obstacle]
    @classmethod
    def from_file(cls, filename):
        with open(filename) as f:
            map_info_dict = json.load(f)
        #custom_unit_path = '.'
        
        groups = []
        for group in map_info_dict['groups']:
            group['faction'] = Faction(group['faction'])
            group['units'] = [(UnitType.from_str(t), c)
                              for t, c in group['units'].items()]
            groups.append(Group(**group))
        map_info_dict['groups'] = groups
        map_info_dict['attack_point'] = list(map_info_dict['attack_point'])
        # if "terrain_preset" in map_info_dict:
        #     map_info_dict["terrain"] = \
        #         TERRAIN_PRESETS[map_info_dict["terrain_preset"]].value
        #     del map_info_dict["terrain_preset"]
        # else:
        #     for i, row in enumerate(map_info_dict['terrain']):
        #         new_row = [None] * len(row)
        #         for j, terrain_type in enumerate(row):
        #             new_row[j] = TerrainType(terrain_type)
        #         map_info_dict['terrain'][i] = new_row
        
        if "obstacle_preset" in map_info_dict : 
            map_size, map_info_dict["maca_obs"], map_info_dict["render_obs"] = OBSTACLE_PRESETS[map_info_dict['obstacle_preset']].value
            map_info_dict['width'],map_info_dict['length'],map_info_dict['height'] = map_size
            del map_info_dict["obstacle_preset"]
        else : 
            map_size, map_info_dict["maca_obs"], map_info_dict["render_obs"] = OBSTACLE_PRESETS['EMPTY'].value
            map_info_dict['width'],map_info_dict['length'],map_info_dict['height'] = map_size
        #if 'unit_type_ids' in map_info_dict:
        #    map_info_dict['unit_type_ids'] = \
        #        {UnitType.from_str(k, custom_unit_path): v
        #         for k, v in map_info_dict['unit_type_ids'].items()}

        #if map_info_dict['num_unit_types'] == 1:
        #    map_info_dict['num_unit_types'] = 0
        return cls(**map_info_dict)


def get_standard_map(map_name):
    return MapInfo.from_file(os.path.join(os.path.dirname(__file__),
                                          'smaclike_maps', f"{map_name}.json"))


MAP_PRESET_DIR = os.path.join(os.path.dirname(__file__), 'smaclike_maps')


class MapPreset(Enum):
    """
    Args:
        Enum (_type_): _description_
    """

    @property
    def map_info(self) -> MapInfo:
        return self.value

    TEST = get_standard_map('test')
    #BASIC_3_VS_3 = get_standard_map('3_vs_3')
    