import carla
from agents.navigation.basic_agent import BasicAgent
import math

TIME_STEP = 0.04

def r(float):
    return round(float, 2)

class IDMAgent(BasicAgent):
    
    # IDM agents only works for HighD Map
    # TODO: add a check for this
    
    def __init__(self, vehicle, target_speed=20, opt_dict={}):
        super().__init__(vehicle, target_speed, opt_dict)
        
        self.desired_velocity = 45  # m/s
        self.safe_time_headway = 1.5 # s
        self.max_acceleration = 10 # m/s^2
        self.comfort_deceleration = 1.67 # m/s^2
        self.acceleration_exponent = 4
        self.minimum_distance_1 = 2 # m
        self.minimum_distance_2 = 9 # m
        self.vehicle_length = 1 # m
        self.far_distance = 500 # m

        if 'desired_velocity' in opt_dict:
            self.desired_velocity = opt_dict['desired_velocity']
        if 'safe_time_headway' in opt_dict:
            self.safe_time_headway = opt_dict['safe_time_headway']
        if 'max_acceleration' in opt_dict:
            self.max_acceleration = opt_dict['max_acceleration']
        if 'comfort_deceleration' in opt_dict:
            self.comfort_deceleration = opt_dict['comfort_deceleration']
        if 'acceleration_exponent' in opt_dict:
            self.acceleration_exponent = opt_dict['acceleration_exponent']
        if 'minimum_distance_1' in opt_dict:
            self.minimum_distance_1 = opt_dict['minimum_distance_1']
        if 'minimum_distance_2' in opt_dict:
            self.minimum_distance_2 = opt_dict['minimum_distance_2']
        if 'vehicle_length' in opt_dict:
            self.vehicle_length = opt_dict['vehicle_length']
        if 'far_distance' in opt_dict:
            self.far_distance = opt_dict['far_distance']
        
        self.IS_DRIVER_SET = False
        self.other_vehicle = None
        
        self.set_other_agent()
        self.logger.info("IDMAgent: init {opt_dict}")
        pass

    def set_other_agent(self):
        all_actors = self._vehicle.get_world().get_actors().filter('vehicle.*')
        for actor in all_actors:
            if actor.id != self._vehicle.id:
                self.other_vehicle = actor
                pass
            pass
    
    def set_idm_parameters(self, desired_velocity, safe_time_headway, 
                           max_acceleration, comfort_deceleration, 
                           acceleration_exponent, 
                           minimum_distance_1, minimum_distance_2,
                           vehicle_length):
        self.desired_velocity = desired_velocity 
        self.safe_time_headway = safe_time_headway
        self.max_acceleration = max_acceleration
        self.comfort_deceleration = comfort_deceleration
        self.acceleration_exponent = acceleration_exponent
        self.minimum_distance_1 = minimum_distance_1
        self.minimum_distance_2 = minimum_distance_2
        self.vehicle_length = vehicle_length
        
        self.IS_DRIVER_SET = True
        pass
    
    
    
    def run_step(self):
        if not self.IS_DRIVER_SET:
            return super().run_step()
        else:
            distance = self.get_other_vehicle_location_x() - self.get_vehicle_location_x() - self.vehicle_length
            gap = distance
            velocity = self.get_vehicle_velocity_x()
            velocity_diff = self.get_vehicle_velocity_x() - self.get_other_vehicle_velocity_x()
            acceleration = self.calc_acceleration(gap, velocity, velocity_diff)
            updated_velocity = velocity + (acceleration * TIME_STEP) + 0.6
            self.set_target_speed(updated_velocity)
            # self.logger.info(f'run_step: vel {r(velocity)}, acc {r(acceleration)}, up_vel {r(updated_velocity)}')
            control = self._local_planner.run_step(debug=True)
            return control
    
    def get_vehicle_velocity_x(self):
        vehicle_velocity = self._vehicle.get_velocity()
        return vehicle_velocity.x
    
    def get_other_vehicle_velocity_x(self):
        other_vehicle_velocity = self.other_vehicle.get_velocity()
        return other_vehicle_velocity.x
    
    def get_vehicle_location_x(self):
        vehicle_location = self._vehicle.get_location()
        return vehicle_location.x
    
    def get_other_vehicle_location_x(self):
        other_vehicle_location = self.other_vehicle.get_location()
        return other_vehicle_location.x
      
    def calc_acceleration(self, gap, velocity, velocity_diff):
        first_term = 1
        second_term = math.pow((velocity / self.desired_velocity), self.acceleration_exponent)
        third_term = math.pow(self.desired_gap(velocity, velocity_diff) / gap, 2)
                
        return self.max_acceleration * (first_term - second_term - third_term)
    
    def desired_gap(self, velocity, velocity_diff):
        
        first_term = self.minimum_distance_1
        second_term = self.minimum_distance_2 * math.sqrt(velocity / self.desired_velocity)
        third_term = velocity * self.safe_time_headway
        fourth_term = (velocity * velocity_diff) / (2 * math.sqrt(self.max_acceleration * self.comfort_deceleration))
        
        return first_term + second_term + third_term + fourth_term
        
        
        
        
        
    
    
    
    # def calc_acceleration(self):
    #     """
    #     dv(t)/dt = [1 - (v(t)/v0)^4  - (s*(t)/s(t))^2]
    #     """
    #     vehicle_velocity = self.vehicle_velocity_x()

    #     vehicle_location = self._vehicle.get_location()
    #     # vehicle_location = carla.Vector3D(vehicle_location.x, vehicle_location.y, 0)
    #     vehicle_location_x = vehicle_location.x
    #     other_vehicle_location = self.other_vehicle.get_location()
    #     other_vehicle_location_x = other_vehicle_location.x 

    #     distance = other_vehicle_location_x - vehicle_location_x
    #     gap = min(1000, distance)
    #     # print('v_vel: ', vehicle_velocity, 'desired_vel: ', self.desired_velocity, ' gap: ', gap)
    #     acceleration = math.pow((vehicle_velocity / self.desired_velocity), self.acceleration_exponent)
    #     deceleration = math.pow(self.calc_desired_gap() / min(self.far_distance, gap), 2)

    #     ret = float(self.max_acceleration * (1 - acceleration - deceleration))
    #     self.logger.info(f'calc_acc {r(acceleration)}, dec {r(deceleration)}, ret {r(ret)}')
    #     return ret

    # def vehicle_velocity_x(self):
    #     vehicle_velocity = self._vehicle.get_velocity()
    #     # vehicle_velocity = carla.Vector3D(vehicle_velocity.x, 0, 0)
    #     # vehicle_velocity = math.sqrt(vehicle_velocity.squared_length())
    #     return vehicle_velocity.x

    # def calc_desired_gap(self):
    #     pv = self.vehicle_velocity_x()
    #     lpv = self.other_vehicle.get_velocity()
    #     lpv = lpv.x
        
    #     del_v = (pv - lpv)
    #     ab = self.max_acceleration * self.comfort_deceleration
    #     c = ((self.safe_time_headway * pv) + ((pv * del_v) / (2 * math.sqrt(ab))))
    #     ret = float(self.minimum_distance + max(0, c))
    #     self.logger.info(f'desired_gap -> del_v {r(del_v)}, ab {r(ab)}, c {r(c)}, ret {r(ret)}')
    #     return ret


    # def calc_velocity(self, delta_time=-1):
    #     new_velocity = self.calc_raw_velocity(delta_time)
    #     ret = float(max(0, new_velocity))
    #     self.logger.info(f'calc_velocity: {r(ret)}, ')
    #     return ret


    # def calc_raw_velocity(self,delta_time=-1):
    #     vehicle_velocity = self.vehicle_velocity_x()
    #     acceleration = self.calc_acceleration()
    #     if delta_time > 0:
    #         result = float(vehicle_velocity + (acceleration * delta_time))
    #     else:
    #         result = float(vehicle_velocity + (acceleration * TIME_STEP))
    #     self.logger.info(f'raw vel -> vehicle vel {r(vehicle_velocity)}, acce {r(acceleration)}, result: {r(result)} ')
    #     return result
    
    


