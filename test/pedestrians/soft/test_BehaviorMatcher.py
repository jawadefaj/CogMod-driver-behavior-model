import pytest
from agents.pedestrians.soft import  NavPath, NavPoint
from agents.pedestrians.soft.BehaviorMatcher import BehaviorMatcher
from agents.pedestrians.soft.BehaviorType import BehaviorType
from agents.pedestrians.soft.LaneSection import LaneSection
from agents.pedestrians.soft.Side import Side
from .fixtures import *

def test_behavior_matcher_psi004(nav_path_psi004):
    matcher = BehaviorMatcher()
    matcher.tagNavPoints(nav_path_psi004)

    point1 = nav_path_psi004.path[0]
    point2 = nav_path_psi004.path[1]
    point3 = nav_path_psi004.path[2]
    point4 = nav_path_psi004.path[3]

    assert BehaviorType.EVASIVE_SLOWDOWN in point1.behaviorTags
    assert len(point1.behaviorTags) == 1
    assert BehaviorType.EVASIVE_FLINCH in point2.behaviorTags
    assert len(point2.behaviorTags) == 1
