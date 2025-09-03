import numpy as np
import math
from smaclike.mamp.util import l3norm

def distance(x1, y1, x2, y2):
        """Distance between two points."""
        return math.hypot(x2 - x1, y2 - y1)

def point_inside_sphere(point: np.ndarray,
                        center: np.ndarray,
                        radius: float) -> bool:
    dpos = np.abs(point - center)
    if np.any(dpos > radius):
        return False
    if dpos.sum() <= radius:
        return True
    return np.inner(dpos, dpos) <= radius ** 2

## if too close, cannot attack
## Laser offset : 0.1, 0, 0
## min_rad, max_rad
#point_inside_cone(self.get_pos(), target.get_pos(), self.get_heading(), self.fov_range, [target.radius, radius])
def point_inside_cone(center: np.ndarray, point: np.ndarray, fov_dir: np.ndarray, fov_range: np.ndarray, radius: np.ndarray) -> bool:   
    laser_start = center #+ np.array([0.1, 0, 0]) do not consider laser start
    distance = l3norm(point, center)
    min_r, max_r = radius
    EPS = 1e-7
    if distance > max_r:
        return False, 1, None
    if distance < min_r: 
        return False, -1, None
    
    x_diff = point[0] - center[0]
    #x_diff = max(x_diff, EPS) if x_diff > 0 else min(x_diff, -EPS)
    azimuth = np.arctan2((point[1] - center[1]), x_diff)
    elevation = np.arctan2((point[2]- center[2]),np.sqrt((point[0]-center[0])**2 + (point[1]-center[1])**2))
    #x = fov_dir[0]
    #x = max(x, EPS) if x > 0 else min(x, -EPS)
    #fov_angle = np.arctan2(fov_dir[1], x)
    fov_angle = fov_dir[0]
    #x_ = fov_dir[0]**2 + fov_dir[1]**2
    #x_ = max(x_, EPS) if x_ > 0 else min(x_, -EPS)
    #fov_angle2 = np.arctan2(fov_dir[2],np.sqrt(fov_dir[0]**2 + fov_dir[1]**2))
    fov_angle2 = fov_dir[1]
    ## default fov_res = 2*np.pi / 12 
    
    # print("azi: {}, fov: {}, ele: {}, fov: {}".format(azimuth, fov_angle, elevation, fov_angle2))
    tmp1 = azimuth - fov_angle # -pi to pi
    tmp1 = (tmp1 + np.pi) % (2 * np.pi) - np.pi                 
    tmp2 = elevation - fov_angle2 # -pi to pi
    # print("point_inside_cone : ",tmp1,fov_range[0]/2, tmp2,fov_range[1]/2)
    azi_cond = np.abs(tmp1) < fov_range[0]/2 
    ele_cond = np.abs(tmp2) < fov_range[1]/2
    tmp1= 0 if azi_cond else tmp1 
    tmp2= 0 if ele_cond else tmp2
    return azi_cond and ele_cond, sign(tmp1), sign(tmp2)

def sign(a) : 
    if a == 0 : 
        return 0
    elif a > 0 :
        return 1
    else : 
        return -1