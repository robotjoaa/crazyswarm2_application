from collections import defaultdict
from typing import Any, Dict, Iterable, List, Union

from smaclike.mamp.policies.kdTree import KDTree

class NeighbourFinder:
    def __init__(self):
        self.all_units = []
        self.kd_tree = None

    def set_all_units(self, all_units: Dict[int, Any]):
        self.all_units = all_units
        self.update()

    def update(self):
        if not self.all_units:
            return
        self.kd_trees = KDTree(self.all_units, None)

    def query_radius(self, units: List[Any],
                     radius: Union[List[float], float],
                     return_distance: bool = False,
                     targetting_mode: bool = False,)
        return 