import carla
import math
from typing import List
from enum import Enum
from .ClientUser import ClientUser
import logging

class MapNames(Enum):
    circle_t_junctions = 'circle_t_junctions'
    t_junction = 't_junction'
    straight_road_with_parking = 'straight'
    varied_width_lanes = 'varied_width_lanes'
    roundabout1 = 'roundabout1'
    # Town01 = 'Town01'
    # Town02 = 'Town02'
    # Town03 = 'Town03'
    # Town04 = 'Town04'
    # Town05 = 'Town05'
    # Town06 = 'Town06'
    # Town07 = 'Town07'
    # Town10 = 'Town10'
    Town11 = 'Town11'
    Town01_Opt = 'Town01_Opt'
    Town02_Opt = 'Town02_Opt'
    Town03_Opt = 'Town03_Opt'
    Town04_Opt = 'Town04_Opt'
    Town05_Opt = 'Town05_Opt'
    Town06_Opt = 'Town06_Opt'
    Town07_Opt = 'Town07_Opt'
    Town10HD_Opt = 'Town10HD_Opt'


class MapManager(ClientUser):

    def __init__(self, client):
        super().__init__(client)
        self.currentMapName = None
        self._waypoints = None

    @property
    def spawn_points(self) -> List[carla.Transform]:
        return self.map.get_spawn_points()

    @property
    def waypoints(self):
        if self._waypoints is None:
            raise Exception("waypoint accessed before loading a map")
        return self._waypoints

    def getMapName(self, map):
        path = map.name
        pathArr = path.split('/')
        return pathArr[-1]

    def generateNavPoints(self, count=20):
        nav_points = []
        for i in range(count):
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                point = carla.Transform(location=loc)
                nav_points.append(point)
        return nav_points

    
    def load(self, mapName: MapNames, layers=carla.MapLayer.NONE, forceReload=False):
        
        currentMapName = self.getMapName(self.map)
        if (mapName.value != currentMapName) or forceReload:
            logging.info(f"MapManager: loading new map {mapName.value}")
            self.client.load_world(mapName.value, map_layers=layers)

        # self.client.load_world(mapName.value, map_layers=layers)
        self.currentMapName = mapName

        self.generateWaypoints()
        self.configureSpectator()

    
    def reload(self):
        self.client.reload_world(False)
        self.generateWaypoints()
        self.configureSpectator()


    def generateWaypoints(self, distance=5.0):
        self._waypoints = self.map.generate_waypoints(distance=distance)


    def configureSpectator(self):

        (x, y, z) = self.getSpectatorPos()
        logging.info(f"MapManager: setting spectator position to ({x}, {y}, {z})")
        transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.circle_t_junctions:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.5), carla.Rotation(pitch=-90)) 
        # elif self.currentMapName == MapNames.t_junction:
        #     transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=-90)) 
        # else:
        #     transform = carla.Transform(carla.Location(x=x, y=y, z=z*3), carla.Rotation(pitch=-90)) 

        if self.currentMapName == MapNames.Town01_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.5), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.Town02_Opt:
            transform = carla.Transform(carla.Location(x=x-15, y=y-8, z=z * 0.15), carla.Rotation(pitch=-88, yaw=2)) 
            # transform = carla.Transform(carla.Location(x=x-20, y=y-10, z=10), carla.Rotation(pitch=-30, yaw=190)) 
        if self.currentMapName == MapNames.Town03_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.3), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.Town04_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.3), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.Town05_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.3), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.Town07_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.5), carla.Rotation(pitch=-90)) 
        if self.currentMapName == MapNames.Town10HD_Opt:
            transform = carla.Transform(carla.Location(x=x, y=y, z=z * 0.5), carla.Rotation(pitch=-90)) 
            
        self.setSpectator(transform)

    
    def setSpectator(self, transform: carla.Transform):
        spectator = self.world.get_spectator()
        spectator.set_transform(transform)

    
    def getSpectatorPos(self):
        minX = 9999999
        maxX = -9999999
        minY = 9999999
        maxY = -9999999

        # for point in self.spawn_points:
        for point in self._waypoints:
            location = point.transform.location
            x = location.x
            y = location.y
            # print(f"MapManager: point: ({x}, {y})" )

            if x < minX:
                minX = x
            if x > maxX:
                maxX = x
            if y < minY:
                minY = y
            if y > maxY:
                maxY = y


        diag = math.sqrt((minX - maxX)**2 + (minY - maxY)**2)
        # print(f"MapManager: Map bb: ({minX}, {minY}) ({maxX, maxY})" )
        # print(f"MapManager: Map bb diag: ({diag})" )

        return ((minX + maxX) / 2, (minY + maxY) / 2, diag)
            
