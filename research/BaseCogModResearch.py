import carla
import logging
from lib.exceptions import NotImplementedInterface
from settings import SourceDestinationPair
from .BaseResearch import BaseResearch
from agents.vehicles import VehicleFactory
from agents.vehicles.CogMod.CogModAgent import CogModAgent


class BaseCogModResearch(BaseResearch):

    def __init__(self,
                 name,
                 client,
                 mapName,
                 logLevel,
                 outputDir,
                 simulationMode,
                 showSpawnPoints=False):
        super().__init__(name,
                         client,
                         mapName,
                         logLevel,
                         outputDir,
                         simulationMode)
        
        # contains all the agents with id as key
        self.agent_list = {}

        self.vehicle_factory =  VehicleFactory(client)
        self.source_destination_pairs = self.createSourceDestinationPairs(distance_from_source=500)
        
        if showSpawnPoints:
            self.visualizer.drawSpawnPoints(dropout=0, life_time=1000)
        pass
    
    def onStart(self):
        raise NotImplementedInterface("onStart")

    def run(self, maxTicks=100):
        raise NotImplementedInterface("run")
    
    # in synchronous mode if we dont tick the vehicle is spawned on origin (0, 0)
    #  right after tick vehicle is shifted at the correct location
    def createCogModAgent(self, source_destination_pair, driver_settings, loglevel=logging.INFO):
        # self.logger.info("createCogModAgent")
        spawn_wp = source_destination_pair.source
        destination_wp = source_destination_pair.destination
    
        driver_profile = driver_settings
        #  get blueprint 
        bpLib = self.world.get_blueprint_library()
        vehicleBps = bpLib.filter('vehicle.audi.*')
        # spawn the vehicle
        # vehicle = self.vehicle_factory.spawn(spawn_transform)
        response = self.client.apply_batch_sync([carla.command.SpawnActor(vehicleBps[0], spawn_wp.transform)], True)[0]
        print('COGMOD actor id ', response.actor_id)
        print('COGMOD has error ', response.has_error())
        vehicle = self.world.get_actor(response.actor_id)
        
        # vehicle = self.vehicle_factory.spawn(spawn_wp)
        if vehicle is None:
            self.logger.error(f"Cannot create vehicle - stopping simulation")
            exit("cannot spawn cogmod vehicle")
        else:
            self.logger.info(f"successfully spawned cogmod actor {vehicle.id}")
            # self.world.tick()
            # self.logger.info(f"vehicle location {vehicle.get_location()}")
            cogmod_agent = CogModAgent(vehicle=vehicle, 
                                       destination_point=destination_wp.transform, 
                                       driver_profile=driver_profile)
            self.agent_list[vehicle.id] = cogmod_agent
            return cogmod_agent
        
            



    def SetSpectator(self, location, rotation=None, height=20):
        spectator = self.world.get_spectator()
        location = carla.Location(x=location.x, y=location.y, z=height)
        if rotation is None:
            rotation = carla.Rotation(pitch=-90, yaw=0)
        
        spectator.set_transform(carla.Transform(location, rotation))
        pass
    
    
    def createSourceDestinationPairs(self, distance_from_source=100):
        ret: list(SourceDestinationPair) = []
        source_points = self.mapManager.spawn_points
        for sp in source_points:
            source = self.map.get_waypoint(sp.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            destination = source.next(distance_from_source)[0]
            ret.append(SourceDestinationPair(source=source, destination=destination))
        return ret    
                 
        
