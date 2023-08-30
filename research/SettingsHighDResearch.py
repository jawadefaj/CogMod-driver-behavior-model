from genericpath import sameopenfile
from turtle import distance
import carla
import logging
import random
import os
import numpy as np
import json
from datetime import date
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.idm_agent import IDMAgent

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
        self.follow_meta = pd.read_csv(self.filterSettings)



    pass



    def setup(self):

        self.simulator = None # populated when run

    def get_follow_meta(self, follow_meta_index):
        curScenario_id = self.follow_meta['scenario_id'].unique()[follow_meta_index]
        curScenario = self.follow_meta[self.follow_meta['scenario_id'] == curScenario_id]
        return curScenario
        
    def locationToVehicleSpawnPoint(self, location: carla.Location) -> carla.Transform:
    
        # find a way point
        waypoint = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            msg = f"{self.name}: Cannot create way point near {location}"
            self.error(msg)
        transform = carla.Transform(location = waypoint.transform.location + carla.Location(z=1), rotation = waypoint.transform.rotation)
        return transform
    
    def updateVehicleCommand(self, world_snapshot, vehicleAgent, vehicle: carla.Vehicle):
        if not vehicle.is_alive:
            return
        
        if vehicleAgent is None:
            self.logger.error(f"vehicleAgent is None")
            return
        
        control = vehicleAgent.run_step()
        control.manual_gear_shift = False
        # self.logger.info(f"control {control}")
        control_command = carla.command.ApplyVehicleControl(vehicle.id, control)
        
        return control_command
        