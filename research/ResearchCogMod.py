# first we spawn a vehicle at the spawn point
# we visualize the gaze first

import carla
from agents.vehicles.CogMod.CogModAgent import CogModAgent
from lib import LoggerFactory, SimulationMode, Simulator
from lib.MapManager import MapNames
from research.BaseCogModResearch import BaseCogModResearch
import agents.vehicles.CogMod.Settings.DriverSettings as DriverSettings 
import os

class ResearchCogMod(BaseCogModResearch):
    
    def __init__(self,
                 client: carla.Client,
                 mapName=MapNames.circle_t_junctions,
                 logLevel="INFO",
                 outputDir:str = "logs",
                 simulationMode = SimulationMode.ASYNCHRONOUS,
                 ):
        
        self.name = "ResearchCogMod"
        
        super().__init__(self.name,
                         client,
                         mapName,
                         logLevel,
                         outputDir,
                         simulationMode,
                         showSpawnPoints=True)
        
        self.cogmod: CogModAgent = None
        self.simulator = None
        self.logger = LoggerFactory.getBaseLogger(self.name, defaultLevel=logLevel, file=os.path.join(outputDir, f"{self.name}.log"))
        
        self.driver_settings = DriverSettings.driver_profile['updated_driver']
        self.source_destination_pair = self.source_destination_pairs[0]
        
        self.setupSimulator()
        pass
    
    def setupSimulator(self):
        onTickers = [self.onTick]
        onEnders = [self.onEnd]
        self.simulator = Simulator(self.client, onTickers, 
                                   onEnders, simulationMode=self.simulationMode)
    
    def onTick(self, world_snapshot):
        print('COGMOD onTick', world_snapshot)
        self.SetSpectator(self.cogmod.get_vehicle().get_location())
        self.cogmod.run_step(0.04)
        pass
    
    def onEnd(self):
        print('COGMOD onEnd')
        # self.initWorldSettingsAsynchronousMode()
        pass
    
    def run(self, maxTicks=100):
        
        self.cogmod = self.createCogModAgent(self.source_destination_pair, self.driver_settings)
        
        self.simulator.run(maxTicks)
        
        pass