import logging
import carla
import os

from lib import ClientUser, LoggerFactory, MapManager, MapNames, SimulationVisualization, Utils, SimulationMode
from research import *
from research.Research1v1NavPathModel import Research1v1NavPathModel
from research.Research4v4 import Research4v4
from research.ResearchIDM import ResearchIDM
# from research.SimulationMode import SimulationMode


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
                            map=MapNames.varied_width_lanes,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            settingsId = "setting1",
                            stats=True,
                            record=False,
                            scenario = "psi-0002",
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
                         stats=stats,
                         record=record,
                         scenario=scenario
                         )
        


        return research
    


    @staticmethod
    def createResearch4v4(
                            host="127.0.0.1", 
                            port=2000, 
                            defaultLogLevel=logging.INFO, 
                            output_dir="logs", 
                            map=MapNames.varied_width_lanes,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            settingsId = "setting1",
                            stats=True,
                            record=False,
                            scenario = "psi-0002"
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
                         stats=stats,
                         record=record,
                         scenario=scenario
                         )

        return research
    
    
    @staticmethod
    def createResearchIDM(
                            host="127.0.0.1", 
                            port=2000, 
                            defaultLogLevel=logging.INFO, 
                            output_dir="logs", 
                            map=MapNames.circle_t_junctions,
                            simulationMode = SimulationMode.ASYNCHRONOUS,
                            filterSettings = None,
                            stats=True
                            ) -> ResearchIDM:

        print(f"research chosen : ResearchIDM with host: {host}, port: {port}, log level: {defaultLogLevel}, output directory: {output_dir}")
        port = int(port)
        name = "ResearchIDM"
        logPath = os.path.join(output_dir, f"{name}.log")
        logger = LoggerFactory.getBaseLogger(name, defaultLevel=defaultLogLevel, file=logPath)
        client = Utils.createClient(logger, host, port)
        research = ResearchIDM(client, 
                         mapName=map, 
                         logLevel=defaultLogLevel, 
                         outputDir=output_dir,
                         simulationMode=simulationMode, 
                         filterSettings=filterSettings,
                         )
        return research