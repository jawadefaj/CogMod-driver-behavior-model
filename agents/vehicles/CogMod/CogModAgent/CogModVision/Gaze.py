import carla
import math
import numpy as np
from ...CogModEnum import GazeDirection
from ...CogModEnum import ManeuverType

from shapely.geometry import LineString
from shapely import geometry
from shapely.geometry import Point
from numpy import random


class Gaze():
    def __init__(self, vehicle, gaze_settings):

        self.name = 'Gaze Module'
        self.tick_counter = 0
        self.gaze_allocation_tick = 10
        self.time_delta = 0.04

        self.vehicle = vehicle
        self.gaze_settings = gaze_settings

        self.gaze_direction = GazeDirection.CENTER
        self.probabilities = [0.01, 0.05, 0.1, 0.5, 0.1, 0.05, 0.01, 0.04, 0.03, 0]
        self.probabilities[-1] = 1 - sum(self.probabilities[:-1])
        
        pass
    

    def filter_object_inside_gaze_direction(self, object_list, maneuver_type):

        self.gaze_direction = self.gaze_direction_tick(maneuver_type)
        gaze_data = self.gaze_settings[self.gaze_direction]
        triangle_corners = self.find_gaze_triangle_corners(self.vehicle, gaze_data)

        gaze_triangle = geometry.Polygon([[p.x, p.y] for p in triangle_corners])
        self.draw_gaze_triangle()

        actor_list = []
        for actor in object_list:
            location = actor.get_location()
            p = Point(location.x, location.y)
            if gaze_triangle.contains(p):
                actor_list.append(actor)

        return actor_list

    # this fucntion is called every tick 
    # it will return a gaze direction based on the maneuver type if the gaze allocation time elapsed
    # if the current direction is not center then it will return center (accounting for importance of the center gaze direction)
    def gaze_direction_tick(self, maneuver_type):
        self.tick_counter += 1
        gaze_direction = None
        if self.tick_counter == self.gaze_allocation_tick:
            self.tick_counter = 0
            # if driver looking else where then we need to reset the gaze direction to center
            if self.gaze_direction != GazeDirection.CENTER:
                gaze_direction = GazeDirection.CENTER
            else:
                gaze_direction = self.get_gaze_direction(maneuver_type)
            
            prev_gaze_data = self.gaze_settings[self.gaze_direction]
            cur_gaze_data = self.gaze_settings[gaze_direction]

            angle_diff = abs(prev_gaze_data.get_eye_movement_angle() - cur_gaze_data.get_eye_movement_angle())
            self.gaze_allocation_tick = int((313 + 5.8 * angle_diff) * self.time_delta)
        else:
            gaze_direction = self.gaze_direction
        return gaze_direction

    def get_gaze_direction(self, maneuver_type):
        val = self.get_gaze_distribution(maneuver_type)
        gaze_direction = list(self.gaze_settings.keys())[val]
        return gaze_direction

    def sample_multinomial(self, probabilities):
        """
        Sample from a multinomial distribution with 1 trial and probabilities for each GazeDirection enum.
        """
        return random.multinomial(1, probabilities).argmax()
    
    def get_gaze_distribution(self, maneuver_type):
        if maneuver_type == ManeuverType.VEHICLE_FOLLOW:
            val = self.sample_multinomial(self.probabilities)
            return val
        
        elif maneuver_type == ManeuverType.LANEFOLLOW:
            val = self.sample_multinomial(self.probabilities)
            return val

    def find_gaze_triangle_corners(self, vehicle, gaze_data, height = 1):
        area = gaze_data.get_area()
        gaze_direction, view_angle, length = area
        view_angle = math.radians(view_angle)   # convert to radians

        vehicle_transform = vehicle.get_transform()
        vehicle_center = vehicle_transform.location
        vehicle_forward = vehicle_transform.get_forward_vector()

        cos = math.cos(gaze_direction)
        sin = math.sin(gaze_direction)
        newX = vehicle_forward.x * cos - vehicle_forward.y * sin
        newY = vehicle_forward.x * sin + vehicle_forward.y * cos

        direction_vector = carla.Vector3D(newX*length, newY*length, height)

        normalized_direction_vector = direction_vector.make_unit_vector()

        left_point = self.find_corner_point_of_triangle(height, view_angle, length, normalized_direction_vector)
        right_point = self.find_corner_point_of_triangle(height, -view_angle, length, normalized_direction_vector)

        left_point = vehicle_center + left_point
        right_point = vehicle_center + right_point
        end_point = vehicle_center + direction_vector

        return [right_point, left_point, vehicle_center]
    

    
    def find_corner_point_of_triangle(self, height, view_angle, length, normalized_direction_vector):
        cos_left = math.cos(view_angle/2)
        sin_left = math.sin(view_angle/2)
        left_point_x =  normalized_direction_vector.x * cos_left - normalized_direction_vector.y * sin_left
        left_point_y =  normalized_direction_vector.x * sin_left + normalized_direction_vector.y * cos_left
        left_point = carla.Vector3D(left_point_x*length, left_point_y*length, height)
        return left_point
    
    
    def draw_gaze_triangle(self):
        
        gaze_data = self.gaze_settings[self.gaze_direction]
        debug = self.vehicle.get_world().debug
        triangle_corners = self.find_gaze_triangle_corners(self.vehicle, gaze_data)
        for i in range(len(triangle_corners)):
            debug.draw_line(begin=triangle_corners[i], 
                            end=triangle_corners[(i+1)%3],
                            thickness=0.2,
                            color=gaze_data.get_color(),
                            life_time=0.2)
        
        return triangle_corners