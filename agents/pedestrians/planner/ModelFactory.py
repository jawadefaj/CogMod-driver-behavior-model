
import importlib
from typing import List
from .PedestrianPlanner import PedestrianPlanner
from ..PedestrianAgent import PedestrianAgent
from lib import ActorManager, ObstacleManager, LoggerFactory, TooManyNewStates
from agents.pedestrians.factors import *
from ..DestinationModel import DestinationModel
from ..destination import CrosswalkModel
from ..factors.DrunkenWalkModel import DrunkenWalkModel
from ..gap_models import *
from agents.pedestrians.StopGoModel import StopGoModel
from ..factors.CrossingOncomingFactorModel import CrossingOncomingFactorModel
from ..factors.FreezingModel import FreezingModel
from ..factors.AntiSurvivalTimeGapModel import AntiSurvivalTimeGapModel
from ..survival_models.SurvivalDestinationModel import SurvivalDestinationModel
from .SpeedModelFactory import SpeedModelFactory

class ModelFactory:

    def __init__(self, 
                    planner: PedestrianPlanner,
                    agent: PedestrianAgent, 
                    actorManager: ActorManager, obstacleManager: ObstacleManager,
                    internalFactors: InternalFactors,  
                ) -> None:
        
        self.planner = planner
        self.agent = agent
        self.actorManager = actorManager
        self.obstacleManager = obstacleManager
        self.internalFactors = internalFactors

        self.name = f"ModelFactory {agent.id}"
        self._logger = LoggerFactory.create(self.name)
        pass


    def createRequiredModels(self):
        
        self.planner.destinationModel = DestinationModel(
                                    self.agent, 
                                    actorManager=self.actorManager, 
                                    obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors,
                                    debug=True
                                    )
        if "speed_model" in self.internalFactors:
            speedModel = SpeedModelFactory.createSpeedModel(
                self.internalFactors["speed_model"], 
                agent=self.agent,
                actorManager=self.actorManager, 
                obstacleManager=self.obstacleManager,
                internalFactors=self.internalFactors
                )
            if speedModel is not None:
                self.planner.destinationModel.applySpeedModel(speedModel)
                self._logger.info(f"{self.internalFactors['speed_model']} SpeedModel applied")
            
        pedGapModel = self.getGapModel()

        self.planner.stopGoModel = StopGoModel(         
                                    pedGapModel,
                                    self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                                    )
        # factor models


        self.planner.models.extend([
                        self.planner.destinationModel, 
                        self.planner.stopGoModel
                      ])
        self.planner.stateTransitionModels.append(self.planner.stopGoModel)


    def getGapModel(self):

        if 'gap_model' in self.internalFactors:
            module = importlib.import_module("agents.pedestrians.gap_models")
            class_ = getattr(module, self.internalFactors['gap_model'])
            pedGapModel = class_(
                                self.agent, 
                                actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                internalFactors=self.internalFactors
                            )
        else:
        
            pedGapModel = BrewerGapModel(
                                        self.agent, 
                                        actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                        internalFactors=self.internalFactors
                                        )
        return pedGapModel

    def createOptionalModels(self, optionalFactors: List[Factors]):

        if optionalFactors is None:
            return

        self.createCrossingModels(optionalFactors)
        self.createSurvivalModels(optionalFactors)
        self.createFreezingModels(optionalFactors)
        self.createAntiSurvivalModel(optionalFactors)
        


    def createAntiSurvivalModel(self, optionalFactors: List[Factors]):
        if Factors.ANTISURVIVAL in optionalFactors:
            antiSurvivalTimeGapModel = AntiSurvivalTimeGapModel(self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                                    )

            self.planner.stopGoModel = StopGoModel(         
                                    antiSurvivalTimeGapModel,
                                    self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                                    )

            # self.planner.models = [
            #                 self.planner.destinationModel, 
            #                 self.planner.stopGoModel
            #             ]
            # self.planner.stateTransitionModels = [self.planner.stopGoModel]

    #region crossing models
    def createCrossingModels(self, optionalFactors: List[Factors]):

        if Factors.CROSSING_ON_COMING_VEHICLE in optionalFactors:
            self.createCrossingOncomingVehicleModel(optionalFactors)
        if Factors.DRUNKEN_WALKER in optionalFactors:
            self.createDrunkinWalkModels(optionalFactors)

        # if Factors.CROSSWALK_MODEL in optionalFactors:
        #     self.attachCrossWalkModelToDestinationModel()


    def createCrossingOncomingVehicleModel(self, optionalFactors: List[Factors]):
        
        self.planner.crossingOncomingVehicleModel = CrossingOncomingFactorModel(
                                    self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                                    )
        self.planner.models.append(self.planner.crossingOncomingVehicleModel)
        self.planner.crossingFactorModels.append(self.planner.crossingOncomingVehicleModel)
        self.planner.stateTransitionModels.append(self.planner.crossingOncomingVehicleModel)



    def createSurvivalModels(self, optionalFactors: List[Factors]):
        if Factors.EVASIVE_RETREAT in optionalFactors:
            self.createSurvivalDestinationModel(optionalFactors)

    def createSurvivalDestinationModel(self, optionalFactors: List[Factors]):
        survivalDestModel = SurvivalDestinationModel(
                                    self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                            )
        self.planner.models.append(survivalDestModel)
        self.planner.survivalModels.append(survivalDestModel)
        self.planner.stateTransitionModels.append(survivalDestModel)
        pass
      
    def createDrunkinWalkModels(self, optionalFactors: List[Factors]):
        if Factors.DRUNKEN_WALKER in optionalFactors:
            drunkenwalk = DrunkenWalkModel(self.agent, self.actorManager, self.obstacleManager, self.internalFactors)
            self.planner.models.append(drunkenwalk)
            
            

    #endregion

    #region freezing models
    
    def createFreezingModels(self, optionalFactors: List[Factors]):
        if Factors.FREEZING_FACTOR in optionalFactors:
            freezingModel = FreezingModel(
                                    self.agent, 
                                    actorManager=self.actorManager, obstacleManager=self.obstacleManager, 
                                    internalFactors=self.internalFactors
                            )

            self.planner.models.append(freezingModel)
            self.planner.freezingModels.append(freezingModel)
            self.planner.stateTransitionModels.append(freezingModel)

    #endregion
