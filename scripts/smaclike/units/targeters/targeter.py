from enum import Enum
from typing import Callable
from smaclike.util.faction import Faction

import numpy as np


class Targeter(object):
    def target(self, origin, target, **kwargs) -> float:
        raise NotImplementedError


class StandardTargeter(Targeter):
    def target(self, origin, target, **kwargs) -> float:
        is_dense = kwargs.get("is_dense", None)
        only_positive = kwargs.get("only_positive", None)
        #print(is_dense, only_positive)
        return origin.deal_damage(target, is_dense, only_positive)


class HealTargeter(Targeter):
    def target(self, origin, target, **kwargs) -> float:
        origin.heal(target)
        return 0

class TargeterType(Enum):
    STANDARD = StandardTargeter
    HEAL = HealTargeter
