from genericpath import sameopenfile
from turtle import distance
from typing import List, Tuple
import carla
import logging
import random
import os
import numpy as np
import json
from datetime import date

from agents.pedestrians.soft import Direction, LaneSection, NavPath, NavPoint
from settings.SourceDestinationPair import SourceDestinationPair
from research.SettingsHighDResearch import SettingHighDResearch

from .BaseResearch import BaseResearch
from settings.circular_t_junction_settings import circular_t_junction_settings
from settings.town02_settings import town02_settings
from settings.town03_settings import town03_settings
from settings.varied_width_lanes_settings import varied_width_lanes_settings
from settings import SettingsManager
from agents.pedestrians import PedestrianFactory
from agents.pedestrians.PedestrianAgent import PedestrianAgent
from agents.pedestrians.factors import Factors
from agents.vehicles import VehicleFactory

from agents.navigation.behavior_agent import BasicAgent
from agents.navigation.idm_agent import IDMAgent
from lib import Simulator, EpisodeSimulator, SimulationMode, EpisodeTrajectoryRecorder, ActorClass
from lib import Utils
import pandas as pd
from lib.MapManager import MapNames
from agents.pedestrians.soft import NavPointLocation, NavPointBehavior, LaneSection, Direction, NavPath

from lib.ScenarioState import ScenarioState
from lib.idm_highd_data_collector import IdmHighDDataCollector

class ResearchIDM(SettingHighDResearch):
    
    def __init__(self, 
                 client: carla.Client, 
                 mapName=MapNames.circle_t_junctions, 
                 logLevel="INFO", 
                 outputDir:str = "logs", 
                 simulationMode = SimulationMode.ASYNCHRONOUS,
                 filterSettings = None,
                 ):

        self.name = "ResearchHighD"

        super().__init__(name=self.name, 
                         client=client, 
                         mapName=mapName, 
                         logLevel=logLevel, 
                         outputDir=outputDir,
                         simulationMode=simulationMode,
                         filterSettings=filterSettings,
                         )
        
        self.speed_buffer = 0.2
        self.distance_boost = 2
        # group the follow_meta by scenario id and then select the first group which is multiple rows
        
        self.cur_scenario_index = 0
        # self.curScenario_id = self.follow_meta['scenario_id'].unique()[0]
        # self.curScenario = self.follow_meta[self.follow_meta['scenario_id'] == self.curScenario_id]
        
        self.idm_vehicle: carla.Vehicle = None
        self.idm_agent: IDMAgent = None
        
        self.preceding_vehicle: carla.Vehicle = None
        self.preceding_agent: BasicAgent = None
        
        self.scenario_state: ScenarioState = None
        self.start_scenario_after_n_frame = 100
        
        self.scenario_start_frame = -1
        self.scenario_end_frame = -1
        
        self.setup()

    def align_scenario(self, scenario_index=0):
        
        cur_scenario = self.get_follow_meta(scenario_index)
        idm_source = carla.Location(-210, 7, 0.5)
        preceding_x = cur_scenario["preceding_x"].iloc[0] + self.distance_boost
        preceding_vehicle_source = carla.Location(-210 + preceding_x, 7, 0.5)
        destination = carla.Location(500, 7, 0.5)
        
        idm_source_destination_pair = SourceDestinationPair(idm_source, destination)
        preceding_vehicle_source_destination_pair = SourceDestinationPair(preceding_vehicle_source, destination)
        return idm_source_destination_pair, preceding_vehicle_source_destination_pair
        
    # filter the highD dataset
    def setup(self):
        super().setup()
        self.data_collector = IdmHighDDataCollector()
        # location = carla.Location(x=500, y=7, z=0.5)
        # self.visualizer.drawPoint(location, color=(0, 0, 255), size=0.2, life_time=1000)
        pass

    
    def createDynamicAgents(self):
    
        command_list = []
        cur_scenario = self.get_follow_meta(self.cur_scenario_index)
        idm_sd_pair, preceding_sd_pair = self.align_scenario(self.cur_scenario_index)
        idm_spawn_command = self.createVehicleCommand(idm_sd_pair)
        preceding_spawn_command = self.createVehicleCommand(preceding_sd_pair)
        
        command_list.append(idm_spawn_command)
        command_list.append(preceding_spawn_command)

        res = self.client.apply_batch_sync(command_list, True)
        for r in res:
            if r.error:
                self.logger.error(r.error)
                print('actor ', r.actor_id, r.error)
            else:
                self.logger.info(f"spawned vehicle {r.actor_id}")

        self.tickOrWaitBeforeSimulation()
        
        self.idm_vehicle = self.world.get_actor(res[0].actor_id)
        self.preceding_vehicle = self.world.get_actor(res[1].actor_id)
        
        idm_target_speed = cur_scenario['ego_vx'].iloc[0] + self.speed_buffer
        preceding_target_speed = cur_scenario['preceding_vx'].iloc[0] + self.speed_buffer
        
        self.idm_agent = IDMAgent(self.idm_vehicle, idm_target_speed)
        self.preceding_agent = BasicAgent(self.preceding_vehicle, preceding_target_speed)
        
        self.visualizer.trackAgentOnTick(self.idm_agent)
        self.visualizer.trackAgentOnTick(self.preceding_agent)
        
        self.scenario_state = ScenarioState.PENDING
        self.scenario_start_frame = -1
        
        self.is_const_vel_enabled = False
        self.is_driver_changed_scenario_starting = False
        self.is_driver_changed_scenario_running = False
        
        print("creating dynamic agents", self.cur_scenario_index, cur_scenario.iloc[0])
        pass

    def enable_constant_velocity(self, idm_target_speed, preceding_target_speed):
        self.idm_vehicle.enable_constant_velocity(carla.Vector3D(x=idm_target_speed, y=0, z=0))
        self.preceding_vehicle.enable_constant_velocity(carla.Vector3D(x=preceding_target_speed, y=0, z=0))

    def recreateDynamicAgents(self):
           
        pass
    
    def restart_scenario(self):
        self.logger.info(f"restarting scenario")
        self.scenario_state = ScenarioState.PENDING
        self.scenario_start_frame = -1
        self.is_const_vel_enabled = False
        self.is_driver_changed_scenario_starting = False
        self.is_driver_changed_scenario_running = False
        self.idm_agent = None
        self.preceding_agent = None
        self.start_scenario_after_n_frame = self.scenario_end_frame + 100
        self.createDynamicAgents()
        pass
    

    
    def createVehicleCommand(self, vehicleSetting: SourceDestinationPair):
        spawn_wp = self.locationToVehicleSpawnPoint(vehicleSetting.source)
        spawn_command = self.vehicleFactory.spawn_command(spawn_wp)
        return spawn_command                

    #region simulation

    def setupSimulator(self, episodic=False):
        """Must be called after all actors are created.

        Args:
            episodic (bool, optional): _description_. Defaults to False.
        """
        # self.episodeNumber = 1 # updated when resetted

        onTickers = [self.visualizer.onTick, self.check_scenario_state, self.onTick] # onTick must be called before restart
        # terminalSignalers = [walkerAgent.isFinished for walkerAgent in self.walkerAgents]
        terminalSignalers = []

        if episodic:
            # this is only to be used from gym environments. It does not call onEnd as we may reset and run
            self.simulator = EpisodeSimulator(
                self.client, 
                terminalSignalers=terminalSignalers, 
                onTickers=onTickers, 
                onEnders=[], 
                simulationMode=self.simulationMode
            )
        else:
            onEnders = [self.onEnd]
            # onTickers.append(self.restart)
            self.simulator = Simulator(
                self.client, 
                onTickers=onTickers, 
                onEnders=onEnders, 
                simulationMode=self.simulationMode
            )

    def run(self, maxTicks=1000):
        try:

            self.createDynamicAgents()
            
            self.tickOrWaitBeforeSimulation()

            self.setupSimulator(False)

            self.simulator.run(maxTicks)
        except Exception as e:
            self.logger.exception(e)
            self.destoryActors()

        # endregion



    
    def onEnd(self):
        self.logger.warn(f"ending simulation")
        self.destoryActors()
        self.idm_agent = None
        self.preceding_agent = None
        self.data_collector.saveCSV(f"follow_scenario_{self.cur_scenario_index}", "logs")
        # self.saveStats()

    def destoryActors(self):
        all_vehicle_actors = self.world.get_actors().filter('vehicle.*')
        id_list = []
        for actor in all_vehicle_actors:
            id_list.append(actor.id)
        print("all vehicle ids : ", id_list)
        command_list = []
        for id in id_list:
            command_list.append(carla.command.DestroyActor(id))
        res = self.client.apply_batch_sync(command_list, False)
        
        for r in res:
            if r.error:
                print('actor ', r.actor_id, r.error)
            else:
                print('actor ', r.actor_id, 'destroyed')
        pass
    
    def onTick(self, world_snapshot):
        
        self.set_spectator()
        cur_scenario = self.get_follow_meta(self.cur_scenario_index)
        
        idm_speed = self.idm_vehicle.get_velocity().length()
        preceding_speed = self.preceding_vehicle.get_velocity().length()
        
        distance = self.idm_vehicle.get_location().distance(self.preceding_vehicle.get_location())
        
        idm_diff = cur_scenario['ego_vx'].iloc[0] - idm_speed
        preceding_diff = cur_scenario['preceding_vx'].iloc[0] - preceding_speed
        
        self.logger.info(f"speed diff idm {round(idm_diff, 2)} ; basic {round(preceding_diff, 2)}, distance {round(distance, 2)}")
        
        command_list = []
        
        if self.scenario_state == ScenarioState.PENDING:
            if not self.is_const_vel_enabled:
                idm_target_speed = cur_scenario['ego_vx'].iloc[0] + self.speed_buffer
                preceding_target_speed = cur_scenario['preceding_vx'].iloc[0] + self.speed_buffer
                self.enable_constant_velocity(idm_target_speed, preceding_target_speed)
                self.is_const_vel_enabled = True
        
        elif self.scenario_state == ScenarioState.STARTING:
            # change driver to idm
            if not self.is_driver_changed_scenario_starting:
                self.idm_vehicle.disable_constant_velocity()
                idm_target_speed = cur_scenario['ego_vx'].iloc[0] + self.speed_buffer
                self.update_driver_settings(idm_target_speed, controlled=True)
                self.is_driver_changed_scenario_starting = True
            pass
        
        elif self.scenario_state == ScenarioState.RUNNING:
            if not self.is_driver_changed_scenario_running:
                idm_target_speed = cur_scenario['ego_vx'].max() + self.speed_buffer
                self.update_driver_settings(idm_target_speed)
                self.preceding_vehicle.disable_constant_velocity()
            scenario_frame = world_snapshot - self.scenario_start_frame
            self.preceding_agent.set_target_speed(cur_scenario['preceding_vx'].iloc[scenario_frame] + self.speed_buffer)
            # self.logger.info(f"scenario frame {scenario_frame}")
            pass
        
        elif self.scenario_state == ScenarioState.FINISHED:
            self.logger.info(f"scenario finished")
            if self.cur_scenario_index < len(self.follow_meta) - 1:
                # self.cur_scenario_index += 1
                self.scenario_end_frame = world_snapshot
                self.data_collector.updateTrajectoryDF()
                # self.restart_scenario()
                self.onEnd()
                self.simulator = None
            return
        
        idm_control_command = self.updateVehicleCommand(world_snapshot, self.idm_agent, self.idm_vehicle)
        preceding_control_command = self.updateVehicleCommand(world_snapshot, self.preceding_agent, self.preceding_vehicle)
        command_list.append(idm_control_command)
        command_list.append(preceding_control_command)
        
        res = self.client.apply_batch_sync(command_list, False)
        for r in res:
            if r.error:
                self.logger.error(r.error)
                print('actor ', r.actor_id, r.error)
        
        # collect data
        self.data_collector.collectStats(
            scenario_id=cur_scenario['scenario_id'].iloc[0],
            exec_num=0,
            frame=world_snapshot,
            scenario_status=self.scenario_state,
            idm_vehicle=self.idm_vehicle,
            actor_vehicle=self.preceding_vehicle
        )
            
    def update_driver_settings(self, desired_speed, controlled=False):
        
        desired_velocity = desired_speed
        safe_time_headway = 0
        max_acceleration = 4.5
        comfort_deceleration = 1.5
        acceleration_exponent = 4
        minimum_distance_1 = 0
        minimum_distance_2 = 0
        vehicle_length = 4
        
        self.idm_agent.set_idm_parameters(desired_velocity, safe_time_headway, 
                                          max_acceleration, comfort_deceleration, 
                                          acceleration_exponent, 
                                          minimum_distance_1, minimum_distance_2,
                                          vehicle_length, controlled)
        pass
    
        
    
    def check_scenario_state(self, world_snapshot):
        
        cur_scenario = self.get_follow_meta(self.cur_scenario_index)
        idm_speed = self.idm_vehicle.get_velocity().length()
        preceding_speed = self.preceding_vehicle.get_velocity().length()
        
        distance = self.idm_vehicle.get_location().distance(self.preceding_vehicle.get_location())
        
        is_idm_speed_ok = abs(cur_scenario['ego_vx'].iloc[0] - idm_speed) < abs(self.speed_buffer) * 2
        is_preceding_speed_ok = abs(cur_scenario['preceding_vx'].iloc[0] - preceding_speed) < abs(self.speed_buffer) * 2
        
        if self.scenario_state == ScenarioState.PENDING and world_snapshot >= self.start_scenario_after_n_frame:
            if is_idm_speed_ok and is_preceding_speed_ok:
                self.scenario_state = ScenarioState.STARTING
                self.logger.info(f"scenario state changed to {self.scenario_state}")
            
        elif self.scenario_state == ScenarioState.STARTING:
            if distance <= cur_scenario['preceding_x'].iloc[0]:
                self.scenario_state = ScenarioState.RUNNING
                self.scenario_start_frame = world_snapshot
                self.logger.info(f"scenario state changed to {self.scenario_state}")
        
        elif self.scenario_state == ScenarioState.RUNNING:
            total_frame = cur_scenario['frame'].iloc[-1]
            if world_snapshot - self.scenario_start_frame >= total_frame:
                self.scenario_state = ScenarioState.FINISHED
                self.logger.info(f"scenario state changed to {self.scenario_state}")
        
        elif self.scenario_state == ScenarioState.FINISHED:
            return
        

    def set_spectator(self):
        mid = (self.idm_vehicle.get_location() - self.preceding_vehicle.get_location()) / 2.0
        location = self.preceding_vehicle.get_location() + mid
        location.z = 200
        rotation = carla.Rotation(pitch=-90)
        transform = carla.Transform(location, rotation)
        self.mapManager.setSpectator(transform)
            











        
    # @property
    # def navPath(self):
    #     if self._navPath is None:
    #         point1 = NavPoint(
    #             NavPointLocation(
    #                 laneId=-1,
    #                 laneSection=LaneSection.LEFT,
    #                 distanceToEgo=24.0, 
    #                 distanceToInitialEgo=24.0, 
    #             ),
    #             NavPointBehavior(
    #                 speed=1,
    #                 direction=Direction.LR
    #             )
    #         )

    #         point2 = NavPoint(
    #             NavPointLocation(
    #                 laneId=-1,
    #                 laneSection=LaneSection.MIDDLE,
    #                 distanceToEgo=7.0, 
    #                 distanceToInitialEgo=25.0, 
    #             ),
    #             NavPointBehavior(
    #                 speed=0.5,
    #                 direction=Direction.LR
    #             )
    #         )

    #         point3 = NavPoint(
    #             NavPointLocation(
    #                 laneId=-1,
    #                 laneSection=LaneSection.MIDDLE,
    #                 distanceToEgo=1.0, 
    #                 distanceToInitialEgo=25.0, 
    #             ),
    #             NavPointBehavior(
    #                 speed=0.1,
    #                 direction=Direction.LR
    #             )
    #         )


    #         point4 = NavPoint(
    #             NavPointLocation(
    #                 laneId=0,
    #                 laneSection=LaneSection.LEFT,
    #                 distanceToEgo=-1, 
    #                 distanceToInitialEgo=25.0, 
    #             ),
    #             NavPointBehavior(
    #                 speed=1,
    #                 direction=Direction.LR
    #             )
    #         )

    #         self._navPath = NavPath(
    #             roadWidth=2 * 3.5,
    #             path=[point1, point2, point3, point4],
    #             nEgoDirectionLanes=1,
    #             nEgoOppositeDirectionLanes=1,
    #             avgSpeed=0.5,
    #             maxSpeed=1.5,
    #             minSpeed=0.0,
    #             egoLaneWrtCenter = 1,
    #             egoSpeedStart=20,
    #             egoSpeedEnd=10
    #         )
    #     return self._navPath

    # def createWalker(self, vehicle: carla.Vehicle, walkerSetting: SourceDestinationPair) -> Tuple[carla.Walker, PedestrianAgent]:

    #     walker, walkerAgent = super().createWalker(walkerSetting)

    #     walkerAgent.setEgoVehicle(vehicle)
    #     walkerAgent.setNavPath(self.navPath)
        
    #     return walker, walkerAgent


    # def resetWalker(self, vehicle: carla.Vehicle, walkerAgent:PedestrianAgent, walkerSetting: SourceDestinationPair, sameOrigin=True):

    #     self.logger.warn(f"Resetting Walker")
    #     walkerAgent.setEgoVehicle(vehicle)
        

    #     if sameOrigin == True:
            
    #         walkerAgent.reset(newStartPoint=walkerSetting.source)
    #         walkerAgent.setDestination(walkerSetting.destination)

    #     elif walkerAgent.location.distance_2d(walkerSetting.source) < 1: # currently close to source
    #         walkerAgent.reset()
    #         walkerAgent.setDestination(walkerSetting.destination)
    #     else:
    #         walkerAgent.reset()
    #         walkerAgent.setDestination(walkerSetting.source)
        
        
    
    
    # def getVehicleWalkerSettingsPairs(self) -> List[Tuple[SourceDestinationPair, SourceDestinationPair]]:
        
    #     walkerSettings = self.settingsManager.getWalkerSettings()
    #     vehicleSettings = self.settingsManager.getVehicleSettings()

    #     pairs = []
    #     for idx in range(len(vehicleSettings)):
    #         if idx < len(walkerSettings):
    #             pairs.append((vehicleSettings[idx], walkerSettings[idx]))

    #     return pairs