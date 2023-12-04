from typing import List

from dataclasses import dataclass, field
from agents.pedestrians.soft.Direction import Direction

from agents.pedestrians.soft.NavPoint import NavPoint

@dataclass
class NavPathRoadConfiguration:
    roadWidth: float
    nEgoDirectionLanes: int
    nEgoOppositeDirectionLanes: int

@dataclass
class NavPathEgoConfiguration:
    egoLaneWrtCenter: int #left is negative, right is positive.
    egoSpeedStart: float
    egoSpeedEnd: float

@dataclass
class NavPathPedestrianConfiguration:
    avgSpeed: float
    maxSpeed: float
    minSpeed: float
    direction: Direction

class NavPath:

    def __init__(
            self, 
            id: any,
            groupId: any,
            roadConfiguration: NavPathRoadConfiguration,
            egoConfiguration: NavPathEgoConfiguration,
            pedConfiguration: NavPathPedestrianConfiguration,
            path: List[NavPoint], 
            ):
        
        self.id = id
        self.groupId = groupId
        self.roadConfiguration = roadConfiguration
        self.egoConfiguration = egoConfiguration
        self.pedConfiguration = pedConfiguration
        self.path = path
        
        assert egoConfiguration.egoLaneWrtCenter > 0

    
    def __str__(self) -> str:
        return f"NavPath {self.id}"

    @property
    def roadWidth(self):
        return self.roadConfiguration.roadWidth
    
    
    @property
    def nEgoDirectionLanes(self):
        return self.roadConfiguration.nEgoDirectionLanes
    
    
    @property
    def nEgoOppositeDirectionLanes(self):
        return self.roadConfiguration.nEgoOppositeDirectionLanes
    
    
    @property
    def direction(self):
        return self.pedConfiguration.direction
    
    
    @property
    def avgSpeed(self):
        return self.pedConfiguration.avgSpeed
    
    
    @property
    def maxSpeed(self):
        return self.pedConfiguration.maxSpeed
    
    
    @property
    def minSpeed(self):
        return self.pedConfiguration.minSpeed
    
    
    @property
    def egoLaneWrtCenter(self):
        return self.egoConfiguration.egoLaneWrtCenter
    
    def setEgoLaneWrtCenter(self, egoLaneWrtCenter: int):
        self.egoConfiguration.egoLaneWrtCenter = egoLaneWrtCenter
    
    
    @property
    def egoSpeedStart(self):
        return self.egoConfiguration.egoSpeedStart
    
    
    @property
    def egoSpeedEnd(self):
        return self.egoConfiguration.egoSpeedEnd

    @property
    def nLanes(self):
        return self.nEgoDirectionLanes + self.nEgoOppositeDirectionLanes

    @property
    def laneWidth(self):
        return self.roadWidth / (self.nLanes)
    
    
    @property
    def roadLength(self):
        maxDistanceToEgo = 0
        for navPoint in self.path:
            maxDistanceToEgo = max(maxDistanceToEgo, navPoint.distanceToEgo)
        return maxDistanceToEgo * 1.5
    
    def getPointLaneIdWrtCenter(self, point: NavPoint) -> int:
        return self.egoLaneWrtCenter + point.laneId

    
