import logging
import carla
import os
from datetime import datetime as date
from lib import ClientUser, LoggerFactory, MapManager, MapNames, SimulationVisualization, Utils, SimulationMode
from research import *
from research.Research1v1NavPathModel import Research1v1NavPathModel
from research.Research4v4 import Research4v4

from .ResearchDriverIDMvCogMod import ResearchDriverIDMvCogMod
from .ResearchCarFollowRepeat import ResearchCarFollowRepeat


class ResearchFactory:
    def __init__(self, host="127.0.0.1", port=2000, output_dir="logs", map=MapNames.circle_t_junctions) -> None:
        self.map = map
        self.host = host
        self.port = int(port)
        self.output_dir = output_dir
        pass

    @staticmethod
    def createResearch1v1(
                            host="127.0.0.1", 
                            port=2000, 
                            defaultLogLevel=logging.INFO, 
                            output_dir="logs", 
                            map=MapNames.circle_t_junctions,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            settingsId = "setting1",
                            stats=True
                            ) -> Research1v1:

        print(f"research chosen : R1v1 with host: {host}, port: {port}, log level: {defaultLogLevel}, output directory: {output_dir}")
        port = int(port)
        name = "Research1v1"
        logPath = os.path.join(output_dir, f"{name}.log")
        logger = LoggerFactory.getBaseLogger(name, defaultLevel=defaultLogLevel, file=logPath)
        client = Utils.createClient(logger, host, port)
        research = Research1v1(client, 
                         mapName=map, 
                         logLevel=defaultLogLevel, 
                         outputDir=output_dir,
                         simulationMode=simulationMode, 
                         settingsId=settingsId,
                         stats=stats
                         )

        return research
    
    
    @staticmethod
    def createResearch1v1NavPathModel(
                            host="127.0.0.1", 
                            port=2000, 
                            defaultLogLevel=logging.INFO, 
                            output_dir="logs", 
                            map=MapNames.circle_t_junctions,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            settingsId = "setting1",
                            stats=True
                            ) -> Research1v1NavPathModel:

        print(f"research chosen : R1v1 with host: {host}, port: {port}, log level: {defaultLogLevel}, output directory: {output_dir}")
        port = int(port)
        name = "Research1v1NavPathModel"
        logPath = os.path.join(output_dir, f"{name}.log")
        logger = LoggerFactory.getBaseLogger(name, defaultLevel=defaultLogLevel, file=logPath)
        client = Utils.createClient(logger, host, port)
        research = Research1v1NavPathModel(client, 
                         mapName=map, 
                         logLevel=defaultLogLevel, 
                         outputDir=output_dir,
                         simulationMode=simulationMode, 
                         settingsId=settingsId,
                         stats=stats
                         )
        


        return research
    


    @staticmethod
    def createResearch4v4(
                            host="127.0.0.1", 
                            port=2000, 
                            defaultLogLevel=logging.INFO, 
                            output_dir="logs", 
                            map=MapNames.circle_t_junctions,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            settingsId = "setting1",
                            stats=True
                            ) -> Research4v4:

        print(f"research chosen : R4v4 with host: {host}, port: {port}, log level: {defaultLogLevel}, output directory: {output_dir}")
        port = int(port)
        name = "Research4v4"
        logPath = os.path.join(output_dir, f"{name}.log")
        logger = LoggerFactory.getBaseLogger(name, defaultLevel=defaultLogLevel, file=logPath)
        client = Utils.createClient(logger, host, port)
        research = Research4v4(client, 
                         mapName=map, 
                         logLevel=defaultLogLevel, 
                         outputDir=output_dir,
                         simulationMode=simulationMode, 
                         settingsId=settingsId,
                         stats=stats
                         )

        return research
    
    
    
    @staticmethod
    def createResearchDriverIDMvCogMod(maxTicks=100,
                                       host="127.0.0.1",
                                       port=2000,
                                       timeout=10.0,
                                       default_log_level=logging.INFO,
                                       output_dir="logs",
                                       simulation_mode=SimulationMode.SYNCHRONOUS,
                                       map_name=MapNames.HighWay_Ring,
                                       high_d_path=None,
                                       stable_height_path=None,
                                       dataset_ids=[],
                                       pivot=None,
                                       base_distance=800,
                                       idm_profile = None,
                                       cogmod_profile = None,
                                       n_repetitions=10,
                                       car_follow_filter = None):
        print("research chosen : ResearchDriverIDMvCogMod")
        name = "Research_IDM1_COG2_DATA"
        logPath = os.path.join(output_dir, f"{name}.log")
        # data_file_name = date.now().strftime("%Y-%m-%d-%H-%M-%S")
        data_file_name = name
        
        logger = LoggerFactory.getBaseLogger(name, defaultLevel=default_log_level, file=logPath)
        client = Utils.createClient(logger, host, port, timeout=timeout)
        
        research = ResearchDriverIDMvCogMod(client=client,
                                            simulationMode=simulation_mode,
                                            map_name=map_name,
                                            high_d_path=high_d_path,
                                            stable_height_path=stable_height_path,
                                            dataset_ids=dataset_ids,
                                            pivot=pivot,
                                            base_distance=base_distance,
                                            idm_profile=idm_profile,
                                            cogmod_profile=cogmod_profile,
                                            n_repeat=n_repetitions,
                                            car_follow_filter=car_follow_filter,
                                            output_dir=output_dir,
                                            log_level=default_log_level,
                                            log_file_name=name,
                                            data_file_name=data_file_name)
        research.run(maxTicks=maxTicks)
        # research = ResearchCarFollowRepeat(client=client,
        #                                     simulationMode=simulation_mode,
        #                                     map_name=map_name,
        #                                     high_d_path=high_d_path,
        #                                     stable_height_path=stable_height_path,
        #                                     dataset_ids=dataset_ids,
        #                                     pivot=pivot,
        #                                     base_distance=base_distance,
        #                                     idm_profile=idm_profile,
        #                                     cogmod_profile=cogmod_profile,
        #                                     n_repeat=n_repetitions,
        #                                     car_follow_filter=car_follow_filter,
        #                                     output_dir=output_dir,
        #                                     log_level=default_log_level,
        #                                     log_file_name=name,
        #                                     data_file_name=data_file_name)
        # research.run(maxTicks)
        pass
    
    
    