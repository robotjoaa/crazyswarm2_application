import numpy as np
from math import sqrt
import os
import json
from enum import Enum
from scipy.spatial.transform import Rotation as R

class Obstacle(object):
    def __init__(self, pos, shape_dict, id, quat=[0,0,0,1]):
        self.shape = shape_dict['shape']
        self.feature = shape_dict['feature']
        if self.shape == 'cube':
            self.length, self.width, self.height = shape_dict['feature']
            self.radius = sqrt(self.length ** 2 + self.width ** 2 + self.height ** 2) / 2
        elif self.shape == 'sphere':
            self.radius = shape_dict['feature']
        else:
            raise NotImplementedError
        self.pos_global_frame = np.array(pos, dtype='float64')
        self.vel_global_frame = np.array([0.0, 0.0, 0.0])
        self.pos = pos
        self.id = id
        self.t = 0.0
        self.step_num = 0
        self.is_at_goal = True
        self.is_obstacle = True
        self.was_in_collision_already = False
        self.is_collision = False

        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.quat = quat

#read obstacles from json file
#returns map size, list of obstacle objects
def from_file(name):
    if name == "empty" : 
        return [4,4,4], [], []
    filename = os.path.join(os.path.dirname(__file__),
                            "preset",
                            f"{name}.json")
    with open(filename) as f:
        obs_dict = json.load(f)
    map_size = obs_dict['map_size']
    obs_list = obs_dict['obstacle_dict']
    count = 0
    maca_obs = []
    render_obs = []
    cube_res = 0.4
    cube_size = [cube_res]*3
    for obstacle in obs_list :
        #shape = obstacle['shape']
        pos = obstacle['pos']
        ori = obstacle['ori']
        size = obstacle['size']
        render_obs.append(obstacle)
        ### divide obstacle into smaller cubes ###
        w, l, h = size
        base_pos = np.array(pos)
        rotation = R.from_quat(ori)
        rotation_matrix = rotation.as_matrix()
        x_cnt = int(w*(1/cube_res))
        y_cnt = int(l*(1/cube_res))
        z_cnt = int(h*(1/cube_res))
        base_x = (1 - x_cnt)*0.5
        base_y = (1 - y_cnt)*0.5
        base_z = (1 - z_cnt)*0.5
        for x in range(x_cnt) : 
            for y in range(y_cnt) : 
                for z in range(z_cnt) : 
                    tmp_pos = np.array([base_x + x, base_y + y, base_z + z])*cube_res
                    tmp_pos = rotation_matrix @ tmp_pos
                    tmp_pos += base_pos
                    maca_obs.append(Obstacle(pos=tmp_pos,shape_dict={'shape': 'cube', 'feature': cube_size},id=count))
                    count += 1
    return map_size, maca_obs, render_obs


class ObstaclePreset(Enum):
    EMPTY = from_file('empty')
    BOX = from_file('box')
    BOX_V2 = from_file('box_v2')
    BOX_V3 = from_file('box_v3')
    CORRIDOR = from_file('corridor')
    ARENA_V0 = from_file('arena_v0')
    ARENA_V1 = from_file('arena_v1')
    ARENA_BG = from_file('arena_bg')

OBSTACLE_PRESETS = {t.name: t for t in ObstaclePreset}
