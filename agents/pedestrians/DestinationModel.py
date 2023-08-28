import carla
import numpy as np
from agents.pedestrians.destination.NavPathModel import NavPathModel
from agents.pedestrians.soft.NavPath import NavPath
from lib import ActorManager, ObstacleManager, Utils
from .ForceModel import ForceModel
from .PedestrianAgent import PedestrianAgent
from agents.pedestrians.factors import InternalFactors
from .PedUtils import PedUtils
from .speed_models.SpeedModel import SpeedModel
from .destination import CrosswalkModel


class DestinationModel(ForceModel):

    def __init__(
        self, 
        agent: PedestrianAgent, 
        actorManager: ActorManager, 
        obstacleManager: ObstacleManager, 
        internalFactors: InternalFactors, 
        final_destination=None,
        debug=False
        ) -> None:

        super().__init__(
            agent, 
            actorManager, 
            obstacleManager, 
            internalFactors=internalFactors, 
            debug=debug
            )

        # self._source = source # source may not be current agent location
        self._finalDestination = final_destination
        self._nextDestination = final_destination

        self.skipForceTicks = 0
        self.skipForceTicksCounter = 0

        self.initFactors()

        self.speedModel: SpeedModel = None
        self.crosswalkModel: CrosswalkModel = None
        self.navPathModel: NavPathModel = None

        pass


    def initFactors(self):
        if "desired_speed" not in self.internalFactors:
            self.internalFactors["desired_speed"] = 2 

        if "relaxation_time" not in self.internalFactors:
            self.internalFactors["relaxation_time"] = 0.1 

        if "use_crosswalk_area_model" not in self.internalFactors:
            self.internalFactors["use_crosswalk_area_model"] = False
        
        pass

    @property
    def name(self):
        return f"DestinationModel {self.agent.id}"
        
    @property
    def nextDestination(self):

        if self._nextDestination.distance_2d(self.agent.location) < 0.1:
            self._nextDestination = self._finalDestination

        elif self.crosswalkModel is not None:
            self._nextDestination = self.crosswalkModel.getNextDestinationPoint()

        elif self.navPathModel is not None:
            self._nextDestination = self.navPathModel.getNextDestinationPoint()

        return self._nextDestination
    
    def addNavPathModel(self, navPath: NavPath):
        # print("addNavPathModel")
        self.navPathModel = NavPathModel(
            agent = self.agent,
            internalFactors=self.internalFactors,
            source = self.agent.location,
            idealDestination = self._finalDestination,
            navPath=navPath,
            areaPolygon = None,
            goalLine = None,
            debug=self.debug
        )

        self.crosswalkModel = None

    def addCrossWalkAreaModel(self):

        # print("addCrossWalkAreaModel")
        self.crosswalkModel = CrosswalkModel(
            agent = self.agent,
            internalFactors=self.internalFactors,
            source = self.agent.location,
            idealDestination = self._finalDestination,
            areaPolygon = None,
            goalLine = None,
            debug=self.debug
        )

    def applySpeedModel(self, speedModel):
        self.speedModel = speedModel

    # def skipNextTicks(self, n):
    #     """We can skip n next ticks

    #     Args:
    #         n ([type]): [description]
    #     """
    #     self.skipForceTicks = n
    #     self.skipForceTicksCounter = 0

    # def needSkip(self):
    #     """One time skip counter

    #     Returns:
    #         [type]: [description]
    #     """
    #     if self.skipForceTicks == 0:
    #         return False
        
    #     if self.skipForceTicksCounter > self.skipForceTicks:
    #         self.skipForceTicksCounter = 0
    #         self.skipForceTicks = 0
    #         return False
        
    #     self.skipForceTicksCounter += 1
    #     return True

    
    def setFinalDestination(self, destination):
        """
        This method creates a list of waypoints between a starting and ending location,
        based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed, the vehicle local planner's target location is chosen,
        which corresponds (by default), to a location about 5 meters in front of the vehicle.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        
        self._finalDestination = destination
        # if self._nextDestination is None:
        #     self._nextDestination = destination
        self._nextDestination = destination # TODO what we want to do is keep a destination queue and pop it to next destination when next destination is reached. 
        
        if self.internalFactors["use_crosswalk_area_model"]:
            if self.crosswalkModel is None:
                self.addCrossWalkAreaModel()
        elif self.navPathModel is not None:
            self.navPathModel.setFinalDestination(destination)
        
        
            
    def getFinalDestination(self):
        if self.crosswalkModel is not None:
            return self.crosswalkModel.getFinalDestination()
        elif self.navPathModel is not None:
            return self.navPathModel.getFinalDestination()
        
        return self._finalDestination
    

    
    def getDesiredVelocity(self) -> carla.Vector3D:

        speed = self.getDesiredSpeed()
        
        self.agent.logger.debug(f"Desired speed is {speed}")

        velocity = self.getDesiredDirection() * speed 
        self.agent.logger.debug(f"Desired velocity is {velocity}")

        return velocity

    def getDesiredSpeed(self) -> carla.Vector3D:
        if self.speedModel is None:
            speed = self.internalFactors["desired_speed"]
        else:
            speed = self.speedModel.desiredSpeed
        return speed


    def getDesiredDirection(self) -> carla.Vector3D:
        
        self.agent.logger.debug(f"next destination is {self.nextDestination}")
        if self.debug:
            self.agent.visualizer.drawPoint(self.nextDestination, color=(0, 0, 255, 100), life_time=0.5)
        return Utils.getDirection(self.agent.feetLocation, self.nextDestination, ignoreZ=True)
        
        

    def getDistanceToNextDestination(self):
        return self.agent.getFeetLocation().distance(self.nextDestination)


    def calculateForce(self):

        # return None
        if not self.agent.isCrossing():
            return None
        
        
        if self.navPathModel is not None:
            if self.navPathModel.isDone():
                print("NavPath model is done")
                self.navPathModel = None
            else:
                self.navPathModel.initNavigation()
                if not self.navPathModel.initialized:
                    return None
                
                force = self.navPathModel.calculateForce()
                if force is not None:
                    return force
                    # return self.clipForce(force)


        # self.agent.logger.warn(f"Collecting state from {self.name}")
        
        # if self.needSkip:
        #     return None

        # self.calculateNextDestination()
        self.nextDestination # updates if required. Do not comment it out

        self.agent.logger.debug(f"no force calculating old destination force towads {self.nextDestination}")
        self.agent.visualizer.drawPoint(self.nextDestination, color=(0, 255, 0), life_time=5)
        force = self.calculateForceForDesiredVelocity()

        # now clip force.
        
        return self.clipForce(force) # if we clip, it's dangerous, if we don't clip, it's also dangerous.


    
    def calculateForceForDesiredVelocity(self):

        """We changed the relationship between change in speed and relaxation time (made it linear so that the pedestrian can linearly increase the speed to the desired velocity)

        Returns:
            _type_: _description_
        """
        desiredVelocity = self.getDesiredVelocity()
        oldVelocity = self.agent.getOldVelocity()

        requiredChangeInVelocity = (desiredVelocity - oldVelocity)

        maxChangeInSpeed = desiredVelocity.length()
        requiredChangeInSpeed = requiredChangeInVelocity.length()

        relaxationTime = (requiredChangeInSpeed / maxChangeInSpeed) * self.internalFactors["relaxation_time"]
        
        return requiredChangeInVelocity / relaxationTime

    



