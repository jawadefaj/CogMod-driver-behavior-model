from .TimeGapModel import TimeGapModel
from lib import ActorManager, ObstacleManager, Utils, LoggerFactory
from agents.pedestrians.PedestrianAgent import PedestrianAgent
from agents.pedestrians.factors import InternalFactors
from .GapUtils import GapUtils
import carla

class BrewerGapModel(TimeGapModel):
    
    def __init__(self, agent: PedestrianAgent, actorManager: ActorManager, obstacleManager: ObstacleManager, internalFactors: InternalFactors) -> None:

        super().__init__(agent, actorManager, obstacleManager, internalFactors=internalFactors)
        # self.name = f"BrewerGapModel #{agent.id}"
        self.logger = LoggerFactory.create(self.name)
        self.initFactors()

        pass

    @property
    def name(self):
        return f"BrewerGapModel #{self.agent.id}"
    
    def initFactors(self):
        
        self.beta0 = 6.2064
        self.beta1 = -0.9420

        if "brewer_beta0" in self.internalFactors:
            self.beta0 = self.internalFactors["brewer_beta0"]
        if "brewer_beta1" in self.internalFactors:
            self.beta1 = self.internalFactors["brewer_beta1"]
        pass

    def p_stop(self, gap):
        y = self.beta0 + self.beta1 * gap
        return GapUtils.sigmoid(y)

    def p_go(self, gap):
        return 1 - self.p_stop(gap)

