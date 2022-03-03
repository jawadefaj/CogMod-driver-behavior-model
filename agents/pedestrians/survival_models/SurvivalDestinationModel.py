from sys import prefix
import carla
import numpy as np
from lib import ActorManager, ObstacleManager, Utils
from ..ForceModel import ForceModel
from ..PedestrianAgent import PedestrianAgent
from agents.pedestrians.factors import InternalFactors
from ..PedUtils import PedUtils
from .SurvivalModel import SurvivalModel
from ..PedState import PedState


class SurvivalDestinationModel(ForceModel, SurvivalModel):
    """This model finds a safe zone for the pedestrian and moves them there quickly. Planner has the history of last positions. Use this model in survival state only

    """

    def __init__(self, agent: PedestrianAgent, actorManager: ActorManager, obstacleManager: ObstacleManager, internalFactors: InternalFactors, final_destination=None) -> None:

        super().__init__(agent, actorManager, obstacleManager, internalFactors=internalFactors)

        self.haveSafeDestination = False # This is a sequential model. There are some stuff to do at start, and some stuff to do at end.
        self._destination = None

        pass


    @property
    def name(self):
        return f"SurvivalModel {self.agent.id}"

    
    def canSwitchToCrossing(self):
        
        # 1. return to crossing is safe destination is reached
        if self.agent.isSurviving():
            if self.agent.location.distance_2d(self._destination) < 0.001:
                self._destination = None
                self.haveSafeDestination = False
                return True

        TG = self.agent.getAvailableTimeGapWithClosestVehicle()
        if TG is None:
            return True

        TTX = PedUtils.timeToCrossNearestLane(self.map, self.agent.location, self.agent._localPlanner.getDestinationModel().getDesiredSpeed())
        
        diff = abs(TG - TTX) # may be too far
        self.agent.logger.info(f"TG:  {TG} and TTX: {TTX}")
        self.agent.logger.info(f"difference between TG and TTX: {diff}")
        if diff > self.internalFactors["threshold_ttc_survival_state"]:
            return True

        return False

    def getNewState(self):
        # 1. return to crossing is safe destination is reached or vehicle stoppped
        if self.canSwitchToCrossing():
                return PedState.CROSSING

        # 2. any other state means, we need to reset the safe destination
        if self.agent.isSurviving() == False:
            self.haveSafeDestination = False
        
        # 3. no survival state if the current state is not crossing
        if self.agent.isCrossing() == False: # wer change state only when crossing
            return None

        self.agent.logger.info(f"Collecting state from {self.name}")

        # 4. Switch to survival mode if close to a collision
        conflictPoint = self.agent.getPredictedConflictPoint()
        if conflictPoint is None:
            return None

        TG = self.agent.getAvailableTimeGapWithClosestVehicle()
        TTX = PedUtils.timeToCrossNearestLane(self.map, self.agent.location, self.agent._localPlanner.getDestinationModel().getDesiredSpeed())
        
        diff = abs(TG - TTX) # may be too far
        self.agent.logger.info(f"TG:  {TG} and TTX: {TTX}")
        self.agent.logger.info(f"difference between TG and TTX: {diff}")
        if diff < self.internalFactors["threshold_ttc_survival_state"]:
            return PedState.SURVIVAL

        return None


    def calculateForce(self):
        if self.agent.isSurviving() == False:
            return None
        
        if self.haveSafeDestination == False:
            # this is the first step
            self.findSafeDestination()
        
        self.agent.logger.info(f"Survival desitnation = {self._destination}")
        # do normal calculations
        # force is now the same as the destination model.

        direction = Utils.getDirection2D(self.agent.location, self._destination)
        speed = self.internalFactors["desired_speed"] 
        desiredVelocity = direction * speed
        
        oldVelocity = self.agent.getOldVelocity()

        return (desiredVelocity - oldVelocity) / (self.internalFactors["relaxation_time"] * 0.1)

        # now 

    
    def findSafeDestination(self):

        # find a location from previous locations that is 5-10 meter back.

        prevLocations = self.agent.previousLocations

        if len(prevLocations) == 0:
            raise Exception(f"No previous locations to safely go to")

        
        # we find a point in the direction firstLocation - currentLocation * min distance

        self.agent.logger.info(prevLocations)

        backwardVector = prevLocations[-1] - self.agent.location
        safeDestination = self.agent.location + backwardVector.make_unit_vector() * self.internalFactors["survival_safety_distance"]
        self._destination = safeDestination

        self.agent.logger.info(safeDestination)

        # if len(prevLocations) == 1:
        #     self._destination = prevLocations[0]
        #     return

        # lastLocation = prevLocations[0]
        # firstLocation = prevLocations[-1]

        # distanceToDestination = lastLocation.distance_2d(firstLocation)

        # if distanceToDestination < self.internalFactors["survival_safety_distance"]:

        #     self._destination = firstLocation
        #     self.agent.logger.info(f"Distance to safe destination: {self.agent.location.distance_2d(self._destination)}")
        #     return

        # # TODO improve this
        # self._destination = firstLocation

        self.agent.logger.info(f"Distance to safe destination: {self.agent.location.distance_2d(self._destination)}")
        self.agent.logger.info(f"Distance to safe destination: {self.agent.location.distance_2d(self._destination)}")

        self.haveSafeDestination = True
