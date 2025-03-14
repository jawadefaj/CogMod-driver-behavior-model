# # Copyright (c) # Copyright (c) 2018-2020 CVC.
# #
# # This work is licensed under the terms of the MIT license.
# # For a copy, see <https://opensource.org/licenses/MIT>.

# """ This module contains PID controllers to perform lateral and longitudinal control. """

# from collections import deque
# import math
# import numpy as np
# import carla
# from agents.tools.misc import get_speed
# import logging
# from lib import LoggerFactory


# class VehiclePIDController():
#     """
#     VehiclePIDController is the combination of two PID controllers
#     (lateral and longitudinal) to perform the
#     low level control a vehicle from client side
#     """


#     def __init__(self, vehicle, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
#                  max_steering=0.8):
#         """
#         Constructor method.

#         :param vehicle: actor to apply to local planner logic onto
#         :param args_lateral: dictionary of arguments to set the lateral PID controller
#         using the following semantics:
#             K_P -- Proportional term
#             K_D -- Differential term
#             K_I -- Integral term
#         :param args_longitudinal: dictionary of arguments to set the longitudinal
#         PID controller using the following semantics:
#             K_P -- Proportional term
#             K_D -- Differential term
#             K_I -- Integral term
#         :param offset: If different than zero, the vehicle will drive displaced from the center line.
#         Positive values imply a right offset while negative ones mean a left one. Numbers high enough
#         to cause the vehicle to drive through other lanes might break the controller.
#         """

#         self.max_brake = max_brake
#         self.max_throt = max_throttle
#         self.max_steer = max_steering

#         self._vehicle = vehicle
#         self._world = self._vehicle.get_world()
#         self.past_steering = self._vehicle.get_control().steer
#         self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
#         self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)

#     def run_step(self, target_speed, waypoint):
#         """
#         Execute one step of control invoking both lateral and longitudinal
#         PID controllers to reach a target waypoint
#         at a given target_speed.

#             :param target_speed: desired vehicle speed
#             :param waypoint: target location encoded as a waypoint
#             :return: distance (in meters) to the waypoint
#         """

#         acceleration = self._lon_controller.run_step(target_speed)
#         current_steering = self._lat_controller.run_step(waypoint)
#         control = carla.VehicleControl()
        
#         if acceleration >= 0.0:
#             control.throttle = min(acceleration, self.max_throt)
#             control.brake = 0.0
#         else:
#             control.throttle = 0.0
#             control.brake = min(abs(acceleration), self.max_brake)

#         # Steering regulation: changes cannot happen abruptly, can't steer too much.

#         if current_steering > self.past_steering + 0.1:
#             current_steering = self.past_steering + 0.1
#         elif current_steering < self.past_steering - 0.1:
#             current_steering = self.past_steering - 0.1

#         if current_steering >= 0:
#             steering = min(self.max_steer, current_steering)
#         else:
#             steering = max(-self.max_steer, current_steering)

#         control.steer = steering
#         control.hand_brake = False
#         control.manual_gear_shift = False
#         self.past_steering = steering

#         return control


#     def change_longitudinal_PID(self, args_longitudinal):
#         """Changes the parameters of the PIDLongitudinalController"""
#         self._lon_controller.change_parameters(**args_longitudinal)

#     def change_lateral_PID(self, args_lateral):
#         """Changes the parameters of the PIDLongitudinalController"""
#         self._lon_controller.change_parameters(**args_lateral)


# class PIDLongitudinalController():
#     """
#     PIDLongitudinalController implements longitudinal control using a PID.
#     """

#     def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
#         """
#         Constructor method.

#             :param vehicle: actor to apply to local planner logic onto
#             :param K_P: Proportional term
#             :param K_D: Differential term
#             :param K_I: Integral term
#             :param dt: time differential in seconds
#         """
#         self._vehicle = vehicle
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt
#         self._error_buffer = deque(maxlen=10)
#         self.logger = LoggerFactory.create("Controller", {'LOG_LEVEL':logging.INFO})

#     def run_step(self, target_speed, debug=False):
#         """
#         Execute one step of longitudinal control to reach a given target speed.

#             :param target_speed: target speed in M/S
#             :param debug: boolean for debugging
#             :return: throttle control
#         """
#         current_speed = get_speed(self._vehicle)

#         if debug:
#             print(f'Current speed = {current_speed}')

#         return self._pid_control(target_speed, current_speed)

#     def _pid_control(self, target_speed, current_speed):
#         """
#         Estimate the throttle/brake of the vehicle based on the PID equations

#             :param target_speed:  target speed in Km/h
#             :param current_speed: current speed of the vehicle in Km/h
#             :return: throttle/brake control
#         """

#         error = target_speed - current_speed
#         self._error_buffer.append(error)

#         if len(self._error_buffer) >= 2:
#             _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
#             _ie = sum(self._error_buffer) * self._dt
#         else:
#             _de = 0.0
#             _ie = 0.0
        
        
#         output = (self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie)
#         ret = np.clip(output, -1.0, 1.0)
#         self.logger.info(f'target {round(target_speed,2)}, cur {round(current_speed,2)}, error {round(error,2)}, sum {round(sum(self._error_buffer),2)}, out {round(output,2)}, ret {ret}')
#         return ret

#     def change_parameters(self, K_P, K_I, K_D, dt):
#         """Changes the PID parameters"""
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt


# class PIDLateralController():
#     """
#     PIDLateralController implements lateral control using a PID.
#     """

#     def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
#         """
#         Constructor method.

#             :param vehicle: actor to apply to local planner logic onto
#             :param offset: distance to the center line. If might cause issues if the value
#                 is large enough to make the vehicle invade other lanes.
#             :param K_P: Proportional term
#             :param K_D: Differential term
#             :param K_I: Integral term
#             :param dt: time differential in seconds
#         """
#         self._vehicle = vehicle
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt
#         self._offset = offset
#         self._e_buffer = deque(maxlen=10)

#     def run_step(self, waypoint):
#         """
#         Execute one step of lateral control to steer
#         the vehicle towards a certain waypoin.

#             :param waypoint: target waypoint
#             :return: steering control in the range [-1, 1] where:
#             -1 maximum steering to left
#             +1 maximum steering to right
#         """
#         return self._pid_control(waypoint, self._vehicle.get_transform())

#     def _pid_control(self, waypoint, vehicle_transform):
#         """
#         Estimate the steering angle of the vehicle based on the PID equations

#             :param waypoint: target waypoint
#             :param vehicle_transform: current transform of the vehicle
#             :return: steering control in the range [-1, 1]
#         """
#         # Get the ego's location and forward vector
#         ego_loc = vehicle_transform.location
#         v_vec = vehicle_transform.get_forward_vector()
#         v_vec = np.array([v_vec.x, v_vec.y, 0.0])

#         # Get the vector vehicle-target_wp
#         if self._offset != 0:
#             # Displace the wp to the side
#             w_tran = waypoint.transform
#             r_vec = w_tran.get_right_vector()
#             w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
#                                                          y=self._offset*r_vec.y)
#         else:
#             w_loc = waypoint.transform.location

#         w_vec = np.array([w_loc.x - ego_loc.x,
#                           w_loc.y - ego_loc.y,
#                           0.0])

#         wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
#         if wv_linalg == 0:
#             _dot = 1
#         else:
#             _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
#         _cross = np.cross(v_vec, w_vec)
#         if _cross[2] < 0:
#             _dot *= -1.0

#         self._e_buffer.append(_dot)
#         if len(self._e_buffer) >= 2:
#             _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
#             _ie = sum(self._e_buffer) * self._dt
#         else:
#             _de = 0.0
#             _ie = 0.0

#         return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

#     def change_parameters(self, K_P, K_I, K_D, dt):
#         """Changes the PID parameters"""
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt




























# import random
# from collections import deque
# import math
# import numpy as np
# import carla
# from agents.tools.misc import get_speed
# import logging
# from lib import LoggerFactory

# # Global variables for sticky action behavior
# STICKY_COUNT = 0
# STICKY_ACTION = None
# STICKY_PROB = 0.1     # Probability of sticky action (7% chance)
# STICKY_STEPS = 3      # Number of steps to maintain the sticky action

# class VehiclePIDController():
#     """
#     VehiclePIDController is the combination of two PID controllers
#     (lateral and longitudinal) to perform the
#     low level control a vehicle from client side
#     """
#     def __init__(self, vehicle, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
#                  max_steering=0.8, enable_sticky_actions=True):
#         """
#         Constructor method.
#         :param vehicle: actor to apply to local planner logic onto
#         :param args_lateral: dictionary of arguments to set the lateral PID controller
#         using the following semantics:
#             K_P -- Proportional term
#             K_D -- Differential term
#             K_I -- Integral term
#         :param args_longitudinal: dictionary of arguments to set the longitudinal
#         PID controller using the following semantics:
#             K_P -- Proportional term
#             K_D -- Differential term
#             K_I -- Integral term
#         :param offset: If different than zero, the vehicle will drive displaced from the center line.
#         Positive values imply a right offset while negative ones mean a left one. Numbers high enough
#         to cause the vehicle to drive through other lanes might break the controller.
#         :param enable_sticky_actions: Boolean to enable/disable sticky action behavior
#         """
#         self.max_brake = max_brake
#         self.max_throt = max_throttle
#         self.max_steer = max_steering
#         self.enable_sticky_actions = enable_sticky_actions
#         self._vehicle = vehicle
#         self._world = self._vehicle.get_world()
#         self.past_steering = self._vehicle.get_control().steer
#         self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
#         self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)
        
#         # Initialize logger
#         self.logger = LoggerFactory.create("StickyController", {'LOG_LEVEL': logging.INFO})

#     def run_step(self, target_speed, waypoint):
#         """
#         Execute one step of control invoking both lateral and longitudinal
#         PID controllers to reach a target waypoint
#         at a given target_speed.
#             :param target_speed: desired vehicle speed
#             :param waypoint: target location encoded as a waypoint
#             :return: distance (in meters) to the waypoint
#         """
#         global STICKY_COUNT, STICKY_ACTION, STICKY_PROB, STICKY_STEPS
#         # Calculate the new control
#         acceleration = self._lon_controller.run_step(target_speed)
#         current_steering = self._lat_controller.run_step(waypoint)
#         new_control = carla.VehicleControl()
        
#         if acceleration >= 0.0:
#             new_control.throttle = min(acceleration, self.max_throt)
#             new_control.brake = 0.0
#         else:
#             new_control.throttle = 0.0
#             new_control.brake = min(abs(acceleration), self.max_brake)
#         # Steering regulation: changes cannot happen abruptly, can't steer too much.
#         if current_steering > self.past_steering + 0.1:
#             current_steering = self.past_steering + 0.1
#         elif current_steering < self.past_steering - 0.1:
#             current_steering = self.past_steering - 0.1
#         if current_steering >= 0:
#             steering = min(self.max_steer, current_steering)
#         else:
#             steering = max(-self.max_steer, current_steering)
#         new_control.steer = steering
#         new_control.hand_brake = False
#         new_control.manual_gear_shift = False
#         self.past_steering = steering
#         # Apply sticky action logic if enabled
#         if self.enable_sticky_actions:
#             control = self._apply_sticky_actions(new_control)
#             self.logger.info(f"Sticky status: count={STICKY_COUNT}, active={STICKY_COUNT>0}, " +
#                            f"throttle={control.throttle:.2f}, brake={control.brake:.2f}, steer={control.steer:.2f}")
#         else:
#             control = new_control
#         return control

#     def _apply_sticky_actions(self, new_control):
#         """
#         Apply sticky action logic to possibly reuse a previous control input
        
#         :param new_control: The newly calculated control
#         :return: Either the new control or a sticky (previously used) control
#         """
#         global STICKY_COUNT, STICKY_ACTION, STICKY_PROB, STICKY_STEPS
        
#         # If we are still in a sticky phase, reuse the stored action
#         if STICKY_COUNT > 0:
#             STICKY_COUNT -= 1  # one less step of sticking
#             return STICKY_ACTION
#         else:
#             # Possibly start a new sticky phase
#             if random.random() < STICKY_PROB:
#                 # Begin a sticky phase for the next STICKY_STEPS calls
#                 STICKY_COUNT = STICKY_STEPS
#                 # We will store the new control for future reuse
#                 STICKY_ACTION = new_control
#             else:
#                 # Normal (non-sticky) path: use the new control
#                 STICKY_ACTION = new_control
            
#             return new_control

#     def change_longitudinal_PID(self, args_longitudinal):
#         """Changes the parameters of the PIDLongitudinalController"""
#         self._lon_controller.change_parameters(**args_longitudinal)

#     def change_lateral_PID(self, args_lateral):
#         """Changes the parameters of the PIDLateralController"""
#         self._lat_controller.change_parameters(**args_lateral)
        
#     def set_sticky_params(self, probability=0.07, steps=1):
#         """
#         Change the sticky action parameters
        
#         :param probability: Probability of applying sticky action (0-1)
#         :param steps: Number of steps to maintain the sticky action
#         """
#         global STICKY_PROB, STICKY_STEPS
#         STICKY_PROB = probability
#         STICKY_STEPS = steps
#         self.logger.info(f"Updated sticky params: prob={STICKY_PROB}, steps={STICKY_STEPS}")

# # The rest of the code (PIDLongitudinalController and PIDLateralController) remains unchanged
# class PIDLongitudinalController():
#     """
#     PIDLongitudinalController implements longitudinal control using a PID.
#     """
#     def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
#         """
#         Constructor method.
#             :param vehicle: actor to apply to local planner logic onto
#             :param K_P: Proportional term
#             :param K_D: Differential term
#             :param K_I: Integral term
#             :param dt: time differential in seconds
#         """
#         self._vehicle = vehicle
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt
#         self._error_buffer = deque(maxlen=10)
#         self.logger = LoggerFactory.create("Controller", {'LOG_LEVEL':logging.INFO})

#     def run_step(self, target_speed, debug=False):
#         """
#         Execute one step of longitudinal control to reach a given target speed.
#             :param target_speed: target speed in M/S
#             :param debug: boolean for debugging
#             :return: throttle control
#         """
#         current_speed = get_speed(self._vehicle)
#         if debug:
#             print(f'Current speed = {current_speed}')
#         return self._pid_control(target_speed, current_speed)

#     def _pid_control(self, target_speed, current_speed):
#         """
#         Estimate the throttle/brake of the vehicle based on the PID equations
#             :param target_speed:  target speed in Km/h
#             :param current_speed: current speed of the vehicle in Km/h
#             :return: throttle/brake control
#         """
#         error = target_speed - current_speed
#         self._error_buffer.append(error)
#         if len(self._error_buffer) >= 2:
#             _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
#             _ie = sum(self._error_buffer) * self._dt
#         else:
#             _de = 0.0
#             _ie = 0.0
        
#         output = (self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie)
#         ret = np.clip(output, -1.0, 1.0)
#         self.logger.info(f'target {round(target_speed,2)}, cur {round(current_speed,2)}, error {round(error,2)}, sum {round(sum(self._error_buffer),2)}, out {round(output,2)}, ret {ret}')
#         return ret

#     def change_parameters(self, K_P, K_I, K_D, dt):
#         """Changes the PID parameters"""
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt

# class PIDLateralController():
#     """
#     PIDLateralController implements lateral control using a PID.
#     """
#     def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
#         """
#         Constructor method.
#             :param vehicle: actor to apply to local planner logic onto
#             :param offset: distance to the center line. If might cause issues if the value
#                 is large enough to make the vehicle invade other lanes.
#             :param K_P: Proportional term
#             :param K_D: Differential term
#             :param K_I: Integral term
#             :param dt: time differential in seconds
#         """
#         self._vehicle = vehicle
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt
#         self._offset = offset
#         self._e_buffer = deque(maxlen=10)

#     def run_step(self, waypoint):
#         """
#         Execute one step of lateral control to steer
#         the vehicle towards a certain waypoin.
#             :param waypoint: target waypoint
#             :return: steering control in the range [-1, 1] where:
#             -1 maximum steering to left
#             +1 maximum steering to right
#         """
#         return self._pid_control(waypoint, self._vehicle.get_transform())

#     def _pid_control(self, waypoint, vehicle_transform):
#         """
#         Estimate the steering angle of the vehicle based on the PID equations
#             :param waypoint: target waypoint
#             :param vehicle_transform: current transform of the vehicle
#             :return: steering control in the range [-1, 1]
#         """
#         # Get the ego's location and forward vector
#         ego_loc = vehicle_transform.location
#         v_vec = vehicle_transform.get_forward_vector()
#         v_vec = np.array([v_vec.x, v_vec.y, 0.0])
#         # Get the vector vehicle-target_wp
#         if self._offset != 0:
#             # Displace the wp to the side
#             w_tran = waypoint.transform
#             r_vec = w_tran.get_right_vector()
#             w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
#                                                          y=self._offset*r_vec.y)
#         else:
#             w_loc = waypoint.transform.location
#         w_vec = np.array([w_loc.x - ego_loc.x,
#                           w_loc.y - ego_loc.y,
#                           0.0])
#         wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
#         if wv_linalg == 0:
#             _dot = 1
#         else:
#             _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
#         _cross = np.cross(v_vec, w_vec)
#         if _cross[2] < 0:
#             _dot *= -1.0
#         self._e_buffer.append(_dot)
#         if len(self._e_buffer) >= 2:
#             _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
#             _ie = sum(self._e_buffer) * self._dt
#         else:
#             _de = 0.0
#             _ie = 0.0
#         return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

#     def change_parameters(self, K_P, K_I, K_D, dt):
#         """Changes the PID parameters"""
#         self._k_p = K_P
#         self._k_i = K_I
#         self._k_d = K_D
#         self._dt = dt













































import random
from collections import deque
import math
import numpy as np
import carla
from agents.tools.misc import get_speed
import logging
from lib import LoggerFactory


# Global variables for sticky action behavior
STICKY_COUNT = 0
STICKY_ACTION = None
STICKY_PROB = 0.1     # Probability of sticky action (7% chance)
MAX_STICKY_STEPS = 5   # Maximum possible steps for a sticky action
STICKY_LAMBDA = 1


class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, vehicle, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.8, enable_sticky_actions=True, sticky_lambda=STICKY_LAMBDA):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        :param enable_sticky_actions: Boolean to enable/disable sticky action behavior
        :param sticky_lambda: Lambda parameter for exponential distribution (higher = shorter stickiness)
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering
        self.enable_sticky_actions = enable_sticky_actions
        self._lambda_param = sticky_lambda

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer
        self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)
        
        # Initialize logger
        self.logger = LoggerFactory.create("StickyController", {'LOG_LEVEL': logging.INFO})

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """
        global STICKY_COUNT, STICKY_ACTION, STICKY_PROB, STICKY_STEPS

        # Calculate the new control
        acceleration = self._lon_controller.run_step(target_speed)
        current_steering = self._lat_controller.run_step(waypoint)
        new_control = carla.VehicleControl()
        
        if acceleration >= 0.0:
            new_control.throttle = min(acceleration, self.max_throt)
            new_control.brake = 0.0
        else:
            new_control.throttle = 0.0
            new_control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        new_control.steer = steering
        new_control.hand_brake = False
        new_control.manual_gear_shift = False
        self.past_steering = steering

        # Apply sticky action logic if enabled
        if self.enable_sticky_actions:
            control = self._apply_sticky_actions(new_control)
            self.logger.info(f"Sticky status: count={STICKY_COUNT}, active={STICKY_COUNT>0}, " +
                           f"throttle={control.throttle:.2f}, brake={control.brake:.2f}, steer={control.steer:.2f}")
        else:
            control = new_control

        return control

    def _apply_sticky_actions(self, new_control):
        """
        Apply sticky action logic to possibly reuse a previous control input
        with exponentially distributed sticky steps (more 1s, fewer 2s, very few 4-5s)
        
        :param new_control: The newly calculated control
        :return: Either the new control or a sticky (previously used) control
        """
        global STICKY_COUNT, STICKY_ACTION, STICKY_PROB, MAX_STICKY_STEPS
        
        # If we are still in a sticky phase, reuse the stored action
        if STICKY_COUNT > 0:
            STICKY_COUNT -= 1  # one less step of sticking
            return STICKY_ACTION
        else:
            # Possibly start a new sticky phase
            if random.random() < STICKY_PROB:
                # Generate number of steps using exponential distribution
                # Lambda of 1.5 gives roughly: 77% for 1, 18% for 2, 4% for 3, 1% for 4-5
                # Generate using exponential distribution and take ceiling to get discrete values
                steps = min(MAX_STICKY_STEPS, max(1, int(math.ceil(random.expovariate(self._lambda_param)))))
                
                STICKY_COUNT = steps
                STICKY_ACTION = new_control
                
                self.logger.info(f"Starting new sticky action for {steps} steps")
            else:
                # Normal (non-sticky) path: use the new control
                STICKY_ACTION = new_control
            
            return new_control

    def change_longitudinal_PID(self, args_longitudinal):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_longitudinal)

    def change_lateral_PID(self, args_lateral):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_lateral)
        
    def set_sticky_params(self, probability=0.07, max_steps=5, lambda_param=1):
        """
        Change the sticky action parameters
        
        :param probability: Probability of applying sticky action (0-1)
        :param max_steps: Maximum number of steps to maintain the sticky action
        :param lambda_param: Lambda parameter for exponential distribution (higher = shorter stickiness)
        """
        global STICKY_PROB, MAX_STICKY_STEPS
        STICKY_PROB = probability
        MAX_STICKY_STEPS = max_steps
        self._lambda_param = lambda_param
        self.logger.info(f"Updated sticky params: prob={STICKY_PROB}, max_steps={MAX_STICKY_STEPS}, lambda={lambda_param}")


# The rest of the code (PIDLongitudinalController and PIDLateralController) remains unchanged
class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)
        self.logger = LoggerFactory.create("Controller", {'LOG_LEVEL':logging.INFO})

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in M/S
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print(f'Current speed = {current_speed}')

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0
        
        
        output = (self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie)
        ret = np.clip(output, -1.0, 1.0)
        self.logger.info(f'target {round(target_speed,2)}, cur {round(current_speed,2)}, error {round(error,2)}, sum {round(sum(self._error_buffer),2)}, out {round(output,2)}, ret {ret}')
        return ret

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

            :param waypoint: target waypoint
            :return: steering control in the range [-1, 1] where:
            -1 maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        if self._offset != 0:
            # Displace the wp to the side
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                         y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt