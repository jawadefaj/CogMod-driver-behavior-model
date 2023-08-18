
import carla
from enum import Enum
import logging
from .BaseCogModResearch import BaseCogModResearch
from lib.HighD import HighD
from lib.Filter import Filter
from collections import defaultdict
from lib import SimulationMode, Simulator
import pandas as pd
import numpy as np
from analysis.DataCollectorDriverResearch import DataCollectorDriverResearch


class ScenarioState(Enum):
    START = 0
    RUNNING = 1
    END = 2
    PENDING = 3
    DISCARD = 4
    pass

def read_stable_height_dict(path):
    stable_height = pd.read_csv(path)
    stable_height_dict = {}
    for i in range(len(stable_height)):
        stable_height_dict[stable_height.iloc[i,0]] = stable_height.iloc[i,1]
    return stable_height_dict

def get_left_right_lanes(df):
    left_lanes = df[df['drivingDirection'] == 1]['laneId'].unique()
    right_lanes = df[df['drivingDirection'] == 2]['laneId'].unique()

    return {'left_lane': left_lanes, 'right_lane': right_lanes}

class DriverModifier():
    
    @staticmethod
    def change_driver_settings_running_simulation(agent_settings, ego_agent_df):
        
        subtasks_parameters = agent_settings['subtasks_parameters']
        lane_following_subtask = subtasks_parameters['lane_following']

        lane_following_subtask['desired_velocity'] = 50
        lane_following_subtask['safe_time_headway'] = 0.5
        lane_following_subtask['max_acceleration'] = 2.9
        lane_following_subtask['comfort_deceleration'] = 1.67
        lane_following_subtask['acceleration_exponent'] = 4
        lane_following_subtask['minimum_distance'] = 2
        lane_following_subtask['vehicle_length'] = 4
        

        subtasks_parameters['lane_following'] = lane_following_subtask
        agent_settings['subtasks_parameters'] = subtasks_parameters
        print('agent settings after ', lane_following_subtask)
        return agent_settings
    
    @staticmethod
    def change_cogmod_settings_pending_simulation(preceding_agent, 
                                                  spawn_distance, 
                                                  vehicle_tracking_radius, 
                                                  driver_profile, 
                                                  ego_agent_df):
        _map = preceding_agent.vehicle.get_world().get_map()
        preceding_agent_location = preceding_agent.vehicle.get_location()
        nearest_waypoint_preceding_agent = _map.get_waypoint(preceding_agent_location, project_to_road=True)
        velocity_df = np.sqrt(ego_agent_df['xVelocity']**2 + ego_agent_df['yVelocity']**2)
        desired_velocity = velocity_df.max() # desired velocity at the beginning of the scenario
        
        local_map = driver_profile['local_map']
        local_map['vehicle_tracking_radius'] = vehicle_tracking_radius
        
        spawn_waypoint = nearest_waypoint_preceding_agent.previous(spawn_distance)[0]
        spawn_transform = spawn_waypoint.transform
        spawn_location = spawn_transform.location
        spawn_location = carla.Vector3D(spawn_location.x, spawn_location.y, 1.0)
        
        destination_transform = preceding_agent.get_destination_transform()
        destination_waypoint = _map.get_waypoint(destination_transform.location, project_to_road=True)
        destination_location = destination_waypoint.transform.location
        
        agent_settings = {'driver_profile': driver_profile,
                          'source': spawn_location,
                          'destination': destination_location}

        lane_following_subtask = agent_settings['driver_profile']['subtasks_parameters']['lane_following']
        lane_following_subtask['desired_velocity'] = desired_velocity
        lane_following_subtask['safe_time_headway'] = 0.5
        lane_following_subtask['max_acceleration'] = 2.9
        lane_following_subtask['comfort_deceleration'] = 1.67
        lane_following_subtask['acceleration_exponent'] = 4
        lane_following_subtask['minimum_distance'] = 2
        lane_following_subtask['vehicle_length'] = 4
        return agent_settings
    
    @staticmethod
    def change_idm_settings_pending_simulation(preceding_agent, 
                                                  spawn_distance, 
                                                  vehicle_tracking_radius, 
                                                  driver_profile, 
                                                  ego_agent_df):
        _map = preceding_agent.vehicle.get_world().get_map()
        preceding_agent_location = preceding_agent.vehicle.get_location()
        nearest_waypoint_preceding_agent = _map.get_waypoint(preceding_agent_location, project_to_road=True)
        velocity_df = np.sqrt(ego_agent_df['xVelocity']**2 + ego_agent_df['yVelocity']**2)
        desired_velocity = velocity_df.max() # desired velocity at the beginning of the scenario
        
        local_map = driver_profile['local_map']
        local_map['vehicle_tracking_radius'] = vehicle_tracking_radius
        
        spawn_waypoint = nearest_waypoint_preceding_agent.previous(spawn_distance)[0]
        spawn_transform = spawn_waypoint.transform
        spawn_location = spawn_transform.location
        spawn_location = carla.Vector3D(spawn_location.x, spawn_location.y, 1.0)
        
        destination_transform = preceding_agent.get_destination_transform()
        destination_waypoint = _map.get_waypoint(destination_transform.location, project_to_road=True)
        destination_location = destination_waypoint.transform.location
        
        agent_settings = {'driver_profile': driver_profile,
                          'source': spawn_location,
                          'destination': destination_location}

        lane_following_subtask = agent_settings['driver_profile']['subtasks_parameters']['lane_following']
        lane_following_subtask['desired_velocity'] = desired_velocity
        lane_following_subtask['safe_time_headway'] = 0.5
        lane_following_subtask['max_acceleration'] = 2.9
        lane_following_subtask['comfort_deceleration'] = 1.67
        lane_following_subtask['acceleration_exponent'] = 4
        lane_following_subtask['minimum_distance'] = 2
        lane_following_subtask['vehicle_length'] = 4
        return agent_settings





class ResearchDriverIDMvCogMod(BaseCogModResearch):
    
    def __init__(self,
                 client,
                 simulationMode,
                 map_name,
                 high_d_path,
                 stable_height_path,
                 dataset_ids,
                 pivot,
                 base_distance,
                 idm_profile,
                 cogmod_profile,
                 n_repeat,
                 car_follow_filter,
                 output_dir,
                 log_level,
                 log_file_name,
                 data_file_name):
        
        self.name = "DriverIDMvCogMod"
        
        super().__init__(name=self.name,
                         client=client,
                         mapName=map_name,
                         logLevel=log_level,
                         outputDir=output_dir,
                         simulationMode=simulationMode,
                         showSpawnPoints=True)
                         
        self.high_d_path = high_d_path
        self.stable_height_path = stable_height_path
        self.dataset_ids = dataset_ids
        self.pivot = pivot
        self.base_distance = base_distance
        self.idm_profile = idm_profile
        self.cogmod_profile = cogmod_profile
        self.n_repeat = n_repeat
        self.car_follow_filter = car_follow_filter
        
        self.log_file_name = log_file_name
        self.data_file_name = data_file_name             
        
        self.filtered_follow_scenario_meta_df =  self.filter_dataset(ids=self.dataset_ids,
                                                                     car_follow_filter=self.car_follow_filter)             
        
        
        self.filtered_follow_scenario_meta_df['is_written'] = False
        
        self.filtered_follow_scenario_meta_df = self.filtered_follow_scenario_meta_df.reset_index(drop=True)
        # print('index ', self.filtered_follow_scenario_meta_df.index)
        self.stable_height_dict = read_stable_height_dict(self.stable_height_path)
        self.data_collector = DataCollectorDriverResearch()
        
        self.agent_list = {'ego': None, 'preceding': None}
        self.frame_tracker = 0
        self.scenario_state = None
        self.current_scenario_index = 0 # scenario id
        self.execution_num = 0 # TODO 0 means we simulate the idm first  
        
        self.current_follow_scenario = None
        self.isDriverChanged = False
        
        pass
    
    
    
    def create_simulation(self, scenario_index):
        
        self.current_follow_scenario = self.filtered_follow_scenario_meta_df.iloc[scenario_index]
        # self.logger.info(f"create_simulation {scenario_index}, {self.current_follow_scenario}")
        dataset_id = self.current_follow_scenario['dataset_id']
        ego_id = self.current_follow_scenario['ego_id']
        preceding_id = self.current_follow_scenario['preceding_id']
        start_frame = self.current_follow_scenario['start_frame']
        end_frame = self.current_follow_scenario['end_frame']
        
        all_frames = self.dfs[self.dfs['dataset_id'] == dataset_id]
        all_frames = all_frames[(all_frames['frame'] >= start_frame) & (all_frames['frame'] <= end_frame)]
        
        self.preceding_agent_df = all_frames[(all_frames['id'] == preceding_id)]
        self.ego_agent_df = all_frames[(all_frames['id'] == ego_id)]
        self.trigger_distance = self.current_follow_scenario['start_distance']
        
        self.laneID = get_left_right_lanes(all_frames)
        self.logger.info(f"dataset id {dataset_id}, ego id {ego_id}, preceding id {preceding_id}, start frame {start_frame}, end frame {end_frame}")
        self.logger.info(f"ego agent df {len(self.ego_agent_df)}, preceding agent df {len(self.preceding_agent_df)}")
        # self.logger.info(f"all frames {len(all_frames)}, col {all_frames.columns}")
        
        preceding_agent = self.createTrajectoryFollowerAgent(agent_id=preceding_id,
                                                             trajectory_df=self.preceding_agent_df,
                                                             pivot=self.pivot,
                                                             stable_height_dict=self.stable_height_dict,
                                                             laneID=self.laneID)
        self.world.tick()
        # self.SetSpectator(preceding_agent.get_vehicle().get_location(), height=50)
        distance = self.trigger_distance + self.base_distance
        
        if self.execution_num == 0:
            ego_agent_settings = DriverModifier.change_idm_settings_pending_simulation(preceding_agent=preceding_agent,
                                                                                        spawn_distance=distance,
                                                                                        vehicle_tracking_radius=self.trigger_distance,
                                                                                        driver_profile=self.cogmod_profile,
                                                                                        ego_agent_df=self.ego_agent_df)
            ego_agent = self.createIDMAgent(ego_agent_settings,
                                            loglevel=logging.INFO)
        else:
            ego_agent_settings = DriverModifier.change_cogmod_settings_pending_simulation(preceding_agent=preceding_agent,
                                                                                        spawn_distance=distance,
                                                                                        vehicle_tracking_radius=self.trigger_distance,
                                                                                        driver_profile=self.cogmod_profile,
                                                                                        ego_agent_df=self.ego_agent_df)
            # self.logger.info(f"current driver settings : {ego_agent_settings}")
            ego_agent = self.createCogModAgent(ego_agent_settings,
                                            loglevel=logging.INFO)
        
        self.agent_list = {'ego': ego_agent, 'preceding': preceding_agent}
        
        self.scenario_state = ScenarioState.PENDING
        self.isDriverChanged = False
        
        pass
    
    
    
    def run(self, maxTicks=100):
        self.max_tick = maxTicks
        self.current_follow_scenario = self.filtered_follow_scenario_meta_df.iloc[self.current_scenario_index]
        self.create_simulation(self.current_scenario_index)
        
        self.scenario_state = ScenarioState.PENDING
        onTickers = [self.checkScenarioState, self.dataCollectorOnTick , self.onTick]
        onEnders = [self.onEnd]
        self.simulator = Simulator(client=self.client,
                                   onTickers=onTickers,
                                   onEnders=onEnders,
                                   useThreads=False, 
                                   sleep=0.05,
                                   simulationMode=self.simulationMode)
        
        self.simulator.run(maxTicks)             
        pass
    
    def onTick(self, tick):
        print('scenario status ', self.scenario_state, tick)
        ego_agent = self.agent_list['ego']
        preceding_agent = self.agent_list['preceding']
        
        if ego_agent is None or preceding_agent is None:
            return
        
        ego_vehicle = ego_agent.vehicle
        preceding_vehicle = preceding_agent.vehicle
        
        ego_location = ego_vehicle.get_location()
        preceding_location = preceding_vehicle.get_location()
        
        self.SetSpectator(ego_location, height=200)
        
        # when we start the scneario we let the driver run with starting config (existing settings)
        if self.scenario_state == ScenarioState.START:
            ego_control = ego_agent.run_step(self.time_delta)
            if ego_control is not None:
                self.client.apply_batch_sync([carla.command.ApplyVehicleControl(ego_vehicle.id, ego_control)])
                pass
        
        # when the driver in running scenario mode, we change the driver at the begining of the running state and 
        # change driver settings
        if self.scenario_state == ScenarioState.RUNNING:
            if self.isDriverChanged == False:
                agent_settings = DriverModifier.change_driver_settings_running_simulation(agent_settings=self.cogmod_profile,
                                                                                        ego_agent_df=self.ego_agent_df)
                ego_agent.reset_driver(agent_settings, time_delta=0.04)
                self.isDriverChanged = True
                pass
            
            ego_control = ego_agent.run_step(self.time_delta)
            if ego_control is not None:
                self.client.apply_batch_sync([carla.command.ApplyVehicleControl(ego_vehicle.id, ego_control)])
                pass
            frame = tick - self.frame_tracker + self.current_follow_scenario['start_frame']
            preceding_agent.run_step(frame)
            pass
        
        # when the driver is in pending state we let the driver run with the existing config
        if self.scenario_state == ScenarioState.PENDING:    
            ego_control = ego_agent.run_step(self.time_delta)
            if ego_control is not None:
                self.client.apply_batch_sync([carla.command.ApplyVehicleControl(ego_vehicle.id, ego_control)])
                pass
            
        if self.scenario_state == ScenarioState.END:
            self.onEnd()
            self.execution_num += 1
            self.restart_scenario()
            
        pass
    
    def restart_scenario(self):
        
        if self.execution_num == self.n_repeat:
            self.execution_num = 0
            if self.current_scenario_index < len(self.filtered_follow_scenario_meta_df) - 1:
                self.current_scenario_index += 1
            else:
                self.logger.info(f'follow meta after simulation done {self.filtered_follow_scenario_meta_df}')
                self.data_collector.saveCSV(self.data_file_name, self.outputDir)
                self.logger.info("simulation ending because all execution competed")
                exit()
        
        self.create_simulation(self.current_scenario_index)
        self.logger.info(f"restart scenario {self.execution_num}, {self.current_scenario_index}")
        pass
    
    
    def onEnd(self):
        print('onEnd')
        all_vehicle_actors = self.world.get_actors().filter('vehicle.*')
        id_list = []
        for actor in all_vehicle_actors:
            id_list.append(actor.id)
        print("all vehicle ids : ", id_list)
        command_list = []
        for id in id_list:
            command_list.append(carla.command.DestroyActor(id))
        res = self.client.apply_batch_sync(command_list, True)
        
        for r in res:
            if r.error:
                print('actor ', r.actor_id, r.error)
            else:
                print('actor ', r.actor_id, 'destroyed')
        pass
    
    def dataCollectorOnTick(self, tick):
        
        dataset_id = self.current_follow_scenario['dataset_id']
        ego_id = self.current_follow_scenario['ego_id']
        preceding_id = self.current_follow_scenario['preceding_id']
        
        if tick >= self.max_tick:
            self.data_collector.saveCSV(self.data_file_name, self.outputDir)
            self.logger.info("simulation ending because time ran out")
            exit()
        
        if self.execution_num == 0:
            agent_type = 'idm'
        else:
            agent_type = 'cogmod'
        
        if self.scenario_state == ScenarioState.END:
            if self.execution_num == self.n_repeat:
                df = self.filtered_follow_scenario_meta_df
                df.at[self.current_scenario_index, 'is_written'] = True
    
            self.data_collector.updateTrajectoryDF() 
            return
        
        
        self.data_collector.collectStats(dataset_id=dataset_id,
                                         scenario_id=self.current_scenario_index,
                                         exec_num=self.execution_num,
                                         agent_type=agent_type,
                                         frame=tick,
                                         scenario_status=self.scenario_state,
                                         ego_id=ego_id,
                                         preceding_id=preceding_id,
                                         ego_agent=self.agent_list['ego'],
                                         actor_agent=self.agent_list['preceding'])
        return
        
        
    
    
    def checkScenarioState(self, tick):
        # print('checkScenarioState ', self.scenario_state, tick)
        ego_agent = self.agent_list['ego']
        preceding_agent = self.agent_list['preceding']
        
        ego_vehicle = ego_agent.vehicle
        preceding_vehicle = preceding_agent.vehicle
        
        ego_location = ego_vehicle.get_location()
        preceding_location = preceding_vehicle.get_location()
        
        ego_velocity = ego_vehicle.get_velocity().length()
        target_velocity = self.ego_agent_df['xVelocity'].iloc[0]
        
        distance = ego_location.distance(preceding_location)
        
        self.logger.info(f"ego vel {ego_velocity}, target {target_velocity}, distance {distance}")
        # none of the agent is alive anymore
        if ego_agent is None or preceding_agent is None:
            self.scenario_state = ScenarioState.END
            return 
        
        #scenario about to start as ego reached the target velocity and we are in pending state
        if ego_velocity >= target_velocity  and self.scenario_state == ScenarioState.PENDING:
            self.scenario_state = ScenarioState.START
            return
        
        # sceanrio is in start state and ego is in the target distance so we start moving 
        # the preceding agent (Running state)
        if distance < self.trigger_distance and self.scenario_state == ScenarioState.START:
            self.scenario_state = ScenarioState.RUNNING
            self.frame_tracker = tick
            return
        
        start_frame = self.current_follow_scenario['start_frame']
        end_frame = self.current_follow_scenario['end_frame']
        cur_frame = tick - self.frame_tracker + start_frame
        if cur_frame > end_frame and self.scenario_state == ScenarioState.RUNNING:
            self.scenario_state = ScenarioState.END
            return
        pass
    
    def filter_dataset(self, ids, car_follow_filter):
        
        high_d = HighD(ids=ids, 
                       data_directory=self.high_d_path)
        
        self.dfs = high_d.get_combined_dataframes()
        print("dataframe type = ", len(self.dfs))
        ego_type = car_follow_filter["ego_type"]
        preceding_type = car_follow_filter["preceding_type"]
        time_duration = car_follow_filter["time_duration"]
        distance_threshold = car_follow_filter["distance_threshold"]
        # print("ego_type = ", ego_type, " preceding_type = ", preceding_type, " time_duration = ", time_duration, " min_distance = ", distance_threshold)
        follow_meta = Filter.filter_vehicle_follow_scenario(self.dfs,
                                                            ego_type=ego_type,
                                                            preceding_type=preceding_type,
                                                            minDuration=time_duration,
                                                            minStartDistance=distance_threshold,
                                                            removeStrictDistanceInc=True)
        print("follow_meta = ", len(follow_meta))
        print("follow meta col = ", follow_meta.columns)
        print("follow meta len = ", len(follow_meta))
        return follow_meta
    
    
    
    pass