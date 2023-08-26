from genericpath import sameopenfile
from turtle import distance
import carla
import logging
import random
import os
import numpy as np
import json
from datetime import date

from agents.pedestrians.soft import Direction, LaneSection, NavPath, NavPoint

from .BaseResearch import BaseResearch
from settings.circular_t_junction_settings import circular_t_junction_settings
from settings.town02_settings import town02_settings
from settings.town03_settings import town03_settings
from settings.varied_width_lanes_settings import varied_width_lanes_settings
from settings import SettingsManager
from agents.pedestrians import PedestrianFactory
from agents.pedestrians.factors import Factors
from agents.vehicles import VehicleFactory
from lib import Simulator, EpisodeSimulator, SimulationMode, EpisodeTrajectoryRecorder, ActorClass
from lib import Utils
import pandas as pd
from lib.MapManager import MapNames
from agents.pedestrians.soft import NavPointLocation, NavPointBehavior, LaneSection, Direction, NavPath

from lib.highD import HighD

class SettingHighDResearch(BaseResearch):
    
    def __init__(self, 
                 name:str,
                 client: carla.Client, 
                 mapName=MapNames.circle_t_junctions, 
                 logLevel="INFO", 
                 outputDir:str = "logs", 
                 simulationMode = SimulationMode.ASYNCHRONOUS,
                 filterSettings = None):


        super().__init__(name=name, 
                         client=client, 
                         mapName=mapName, 
                         logLevel=logLevel, 
                         outputDir=outputDir,
                         simulationMode=simulationMode)

        self.filterSettings = filterSettings
        if mapName == MapNames.HighWay_Ring:
            self.logger.info("HighDResearch: setting up for HighWay_Ring")
        else:
            raise Exception(f"Map {mapName} is missing settings")
        
        self.vehicleFactory = VehicleFactory(self.client, visualizer=self.visualizer)

        # self.episodeNumber = 0
        # self.episodeTimeStep = 0
        # self.stats = stats
        # self.maxStepsPerCrossing = maxStepsPerCrossing
        # self.settingsId = settingsId

    pass



    def setup(self):

        self.simulator = None # populated when run

        # change spectator if in setting
        # spectatorSettings = self.settingsManager.getSpectatorSettings()
        # if spectatorSettings is not None:
        #     self.mapManager.setSpectator(spectatorSettings)
        
