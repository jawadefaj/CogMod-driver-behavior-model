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

        self.vehicle = vehicle
        self.gaze_settings = gaze_settings

        self.gaze_directions = list(self.gaze_settings.keys())
        self.gaze_fixation_areas = [self.gaze_settings[direction].get_fixation_area() for direction in self.gaze_directions]
        self.gaze_fixation_probability = [self.gaze_settings[direction].get_fixation_probability() for direction in self.gaze_directions]
        
        pass
    
    def sample_gaze_direction(self):
        val = self.sample_multinomial()
        gaze_direction = self.gaze_directions[val]
        return gaze_direction


    def sample_multinomial(self):
        """
        Sample from a multinomial distribution with 1 trial and probabilities for each GazeDirection enum.
        """
        return random.multinomial(1, self.gaze_fixation_probability).argmax()

    def get_movement_angle_diff(self, prev_gaze_direction, cur_gaze_direction):
        prev_gaze_data = self.gaze_settings[prev_gaze_direction]
        cur_gaze_data = self.gaze_settings[cur_gaze_direction]
        angle_diff = abs(prev_gaze_data.get_movement_factor() - cur_gaze_data.get_movement_factor())
        return angle_diff


    def filter_object_inside_fixation_area(self, object_list, gaze_direction):

        triangle_corners = self.find_gaze_triangle_corners(self.vehicle, gaze_direction)

        gaze_triangle = geometry.Polygon([[p.x, p.y] for p in triangle_corners])
        self.draw_gaze_triangle(gaze_direction)

        actor_list = []
        for actor in object_list:
            location = actor.get_location()
            p = Point(location.x, location.y)
            if gaze_triangle.contains(p):
                actor_list.append(actor)

        return actor_list

    def find_gaze_triangle_corners(self, vehicle, gaze_direction, height = 1):

        index = self.gaze_directions.index(gaze_direction)
        fixation_area = self.gaze_fixation_areas[index]

        gaze_direction, view_angle, length = fixation_area
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
    
    



    def draw_gaze_triangle(self, gaze_direction):
        
        debug = self.vehicle.get_world().debug
        triangle_corners = self.find_gaze_triangle_corners(self.vehicle, gaze_direction)
        for i in range(len(triangle_corners)):
            debug.draw_line(begin=triangle_corners[i], 
                            end=triangle_corners[(i+1)%3],
                            thickness=0.2,
                            color=carla.Color(255, 0, 0, 0),
                            life_time=0.2)
        
        return triangle_corners




    
    # # this fucntion is called every tick 
    # # it will return a gaze direction based on the maneuver type if the gaze allocation time elapsed
    # # if the current direction is not center then it will return center (accounting for importance of the center gaze direction)
    # def gaze_direction_tick(self, maneuver_type):
    #     self.tick_counter += 1
    #     gaze_direction = None
    #     if self.tick_counter == self.gaze_allocation_tick:
    #         self.tick_counter = 0
    #         # if driver looking else where then we need to reset the gaze direction to center
    #         if self.cur_gaze_direction != GazeDirection.CENTER:
    #             gaze_direction = GazeDirection.CENTER
    #         else:
    #             gaze_direction = self.get_gaze_direction(maneuver_type)
            
    #         prev_gaze_data = self.gaze_settings[self.cur_gaze_direction]
    #         cur_gaze_data = self.gaze_settings[gaze_direction]

    #         angle_diff = abs(prev_gaze_data.get_eye_movement_angle() - cur_gaze_data.get_eye_movement_angle())
    #         self.gaze_allocation_tick = int((313 + 5.8 * angle_diff) * self.time_delta)
    #     else:
    #         gaze_direction = self.cur_gaze_direction
    #     return gaze_direction
    


    # def get_gaze_direction(self, maneuver_type):
    #     val = self.get_gaze_distribution(maneuver_type)
    #     gaze_direction = list(self.gaze_settings.keys())[val]
    #     return gaze_direction

    # def sample_multinomial(self, probabilities):
    #     """
    #     Sample from a multinomial distribution with 1 trial and probabilities for each GazeDirection enum.
    #     """
    #     return random.multinomial(1, probabilities).argmax()
    
    # def get_gaze_distribution(self, maneuver_type):
    #     if maneuver_type == ManeuverType.VEHICLE_FOLLOW:
    #         val = self.sample_multinomial(self.gaze_fixation_probability)
    #         return val
        
    #     elif maneuver_type == ManeuverType.LANEFOLLOW:
    #         val = self.sample_multinomial(self.gaze_fixation_probability)
    #         return val
