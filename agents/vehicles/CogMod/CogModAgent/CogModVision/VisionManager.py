
from agents.vehicles.CogMod.CogModEnum import GazeDirection
from .Gaze import Gaze



class VisionManager():

    def __init__(self, vehicle, vision_settings) -> None:
        self.name = 'Vision Manager'
        self.vehicle = vehicle
        self.vision_settings = vision_settings

        self.lane_following_gaze = Gaze(vehicle, vision_settings['gaze']['lane_follow'])
        self.vehicle_following_gaze = Gaze(vehicle, vision_settings['gaze']['vehicle_follow'])

        self.d_saccade = self.vision_settings['d_saccade_const']
        self.d_saccade_mul = self.vision_settings['d_saccade_mul']
        self.l_saccade_const = self.vision_settings['l_saccale_const']
        self.l_saccade_mul = self.vision_settings['l_saccale_mul']
        self.base_saccade_duration = self.vision_settings['base_saccade_duration']
        
        self.current_gaze_direction = GazeDirection.CENTER
        self.time_delta = 0.04
        self.tick_counter = 0
        self.gaze_allocation_tick = 10
        pass


    def tick(self, vehicle, maneuver_type, object_list):
        self.tick_counter += 1
        print('tick vs module', self.tick_counter, self.gaze_allocation_tick)
        seen_objects = []
        if self.tick_counter == self.gaze_allocation_tick:
            self.tick_counter = 0
            new_gaze_direction = self.lane_following_gaze.sample_gaze_direction()
            self.gaze_allocation_tick = self.set_gaze_allocation_tick(new_gaze_direction, self.current_gaze_direction)
            self.current_gaze_direction = new_gaze_direction

        seen_objects = self.lane_following_gaze.filter_object_inside_fixation_area(object_list, self.current_gaze_direction)
        return seen_objects



    def set_gaze_allocation_tick(self, prev_gaze_direction, cur_gaze_direction):

        angle_diff = self.lane_following_gaze.get_movement_angle_diff(prev_gaze_direction, cur_gaze_direction)
        saccade_duration = self.d_saccade + self.d_saccade_mul * angle_diff
        saccade_latency = self.l_saccade_const + self.l_saccade_mul * angle_diff
        total_saccade_duration = saccade_duration + saccade_latency + self.base_saccade_duration
        gaze_allocation_tick = int(total_saccade_duration * self.time_delta)
        return gaze_allocation_tick