from typing import Dict, List, Set

from smaclike.mamp.neighbour_finder import NeighbourFinder
#from smaclike.mamp.obstacle_finder import ObstacleFinder
#from smaclike.mamp.static_obstacle import StaticObstacle
from smaclike.units.unit import Unit


class VelocityUpdater(object):
    def __init__(self, kd_tree: NeighbourFinder,
                 max_radius: float,
                 map_path: str) -> None:
        raise NotImplementedError

    def reset_all_units(self, all_units: Dict[int, Unit]):
        raise NotImplementedError

    def compute_new_velocities(self, all_units: Dict[int, Unit]):
        raise NotImplementedError


class MACAVelocityUpdater(VelocityUpdater):
    def __init__(self, kd_tree: NeighbourFinder,
                 max_radius: float,
                 map_path: str) -> None:
        from smaclike.mamp.envs.mampenv import MACAEnv
        self.env = MACAEnv()
        ## get obstacle from map_path
        self.obstacles = None

    def reset_all_units(self, all_units: Dict[int, Unit]):
        #remove all units from the environment
        
        agents = [unit.agent for unit in all_units.values()]
        self.env.set_agents(agents, self.obstacles)
        

    def compute_new_velocities(self, all_units: Dict[int, Unit]):
        self.env.step()
