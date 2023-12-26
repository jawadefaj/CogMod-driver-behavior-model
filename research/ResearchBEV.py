# this research is to create the bev image from a carla basic agent and visualize the bev

from typing import List
import carla
from lib import Simulator, EpisodeSimulator
from lib import SimulationMode
from lib.MapManager import MapNames
from research.BaseResearch import BaseResearch
from settings import SettingsManager
from settings.town02_settings import town02_settings as settings
class ResearchBEV(BaseResearch):
    def __init__(self, 
                 client: carla.Client, 
                 mapName=MapNames.varied_width_lanes, 
                 logLevel="INFO", 
                 outputDir:str = "logs", 
                 simulationMode = SimulationMode.ASYNCHRONOUS,
                 ):

        self.name = "ResearchBEV"

        super().__init__(name=self.name, 
                         client=client, 
                         mapName=mapName, 
                         logLevel=logLevel, 
                         outputDir=outputDir,
                         simulationMode=simulationMode,
                         )
        
        self.settings_manager = SettingsManager(self.client, settings)
        self.ego_vehicle = None
        self.other_vehicles: List[carla.Vehicle] = []
    
    def setup(self):
        pass
    
    def setupSimulator(self):
        onTickers = [self.visualizer.onTick, self.onTick] # onTick must be called before restart

        onEnders = [self.onEnd]
        onTickers.append(self.restart)
        self.simulator = Simulator(self.client, 
                                   onTickers=onTickers, 
                                   onEnders=onEnders, 
                                   simulationMode=self.simulationMode)
        pass
    
    def run(self, maxTicks=1000):
        ego_settings = self.settings_manager.getVehicleSettings()
        print("ego settings ", ego_settings)
        pass
    
    def onEnd(self):
        self.logger.info("onEnd")
        pass
    
    def onTick(self):
        self.logger.info("onTick")
        pass