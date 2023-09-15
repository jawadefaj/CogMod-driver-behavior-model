
from .GazeSettings import *

CONST = 500
driver_profile = {
    'updated_driver': { 'vision':{
                            'gaze':
                                { 
                                    'lane_follow': lane_follow_gaze, 
                                    'vehicle_follow': vehicle_follow_gaze
                                },
                            'd_saccade_const': 21,
                            'd_saccade_mul': 2.2,
                            'l_saccale_const': 242,
                            'l_saccale_mul': 3.59,
                            'base_saccade_duration': 50,
                        },
                    },

                                    
    # 'driver1': {
    #     'servers': {
    #         'longterm_memory': {'queue_length': 10, 'tick_frequency': 1,},
    #         'complex_cognition': {'queue_length': 10, 'tick_frequency': 1,},
    #         'motor_control': {'queue_length': 10, 'tick_frequency': 1,},
    #     },
    #     'local_map': {
    #         'vehicle_tracking_radius': 20,
    #         'global_plan_sampling_resolution': 1.0,
    #     },
    #     'gaze': Gaze_Settings1,
    #     'controller': {
    #         'lateral_PID': {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01,},
    #         'longitudinal_PID': {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': 0.01,},
    #         'max_throttle': 0.75,
    #         'max_brake': 0.3,
    #         'max_steering': 0.8,
    #         'offset': 0.0,
    #     },
    #     'subtasks_parameters': {
    #         'lane_following': {
    #             'desired_velocity': 10.56, # m/s
    #             'safe_time_headway': 1.5, # s
    #             'max_acceleration': 2.73, # m/s^2
    #             'comfort_deceleration': 1.67, # m/s^2
    #             'acceleration_exponent': 4, 
    #             'minimum_distance': 4, # m
    #             'vehicle_length': 1, # m
    #         },
    #         'lane_keeping': {
    #             'far_distance': 30.0,
    #         },
    #     },
    # },

    'driver2': {
        'servers': {
            'longterm_memory': {'queue_length': 10, 'tick_frequency': 1,},
            'complex_cognition': {'queue_length': 10, 'tick_frequency': 1,},
            'motor_control': {'queue_length': 10, 'tick_frequency': 1,},
        },
        'local_map': {
            'vehicle_tracking_radius': CONST,                               # important parameter
            'global_plan_sampling_resolution': 1.0,               
        },
        'gaze': lane_follow_gaze,
        'controller': {
            'lateral_PID': {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.04,},
            'longitudinal_PID': {'K_P': 20.0, 'K_I': 5, 'K_D': 0.0, 'dt': 0.04,},
            'max_throttle': 0.99,
            'max_brake': 0.5,
            'max_steering': 0.8,
            'offset': 0.0,
        },
        'subtasks_parameters': {
            'lane_following': {
                'desired_velocity': 50, # m/s
                'safe_time_headway': 0.5, # s
                'max_acceleration': 2.9, # m/s^2
                'comfort_deceleration': 1.67, # m/s^2
                'acceleration_exponent': 4, 
                'minimum_distance': 2, # m
                'vehicle_length': 4, # m
                'far_distance': CONST,                                 # important parameter
            },
            'lane_keeping': {
                'far_distance': CONST,                               # important parameter
            },
        },
    },

    # 'driver3': {
    #     'servers': {
    #         'longterm_memory': {'queue_length': 10, 'tick_frequency': 1,},
    #         'complex_cognition': {'queue_length': 10, 'tick_frequency': 1,},
    #         'motor_control': {'queue_length': 10, 'tick_frequency': 1,},
    #     },
    #     'local_map': {
    #         'vehicle_tracking_radius': 30,
    #         'global_plan_sampling_resolution': 1.0,
    #     },
    #     'controller': {
    #         'lateral_PID': {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01,},
    #         'longitudinal_PID': {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': 0.01,},
    #         'max_throttle': 0.75,
    #         'max_brake': 0.3,
    #         'max_steering': 0.8,
    #         'offset': 0.0,
    #     },
    #     'subtasks_parameters': {
    #         'lane_following': {
    #             'desired_velocity': 5.56, # m/s
    #             'safe_time_headway': 1.5, # s
    #             'max_acceleration': 2.73, # m/s^2
    #             'comfort_deceleration': 1.67, # m/s^2
    #             'acceleration_exponent': 4, 
    #             'minimum_distance': 4, # m
    #             'vehicle_length': 1, # m
    #         },
    #         'lane_keeping': {
    #             'far_distance': 30.0,
    #         },
    #     },
    # },


    # 'driver4': {
    #     'servers': {
    #         'longterm_memory': {'queue_length': 10, 'tick_frequency': 2,},
    #         'complex_cognition': {'queue_length': 10, 'tick_frequency': 2,},
    #         'motor_control': {'queue_length': 10, 'tick_frequency': 1,},
    #     },
    #     'local_map': {
    #         'vehicle_tracking_radius': 30,
    #         'global_plan_sampling_resolution': 1.0,
    #     },
    #     'controller': {
    #         'lateral_PID': {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': 0.01,},
    #         'longitudinal_PID': {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': 0.01,},
    #         'max_throttle': 0.75,
    #         'max_brake': 0.3,
    #         'max_steering': 0.8,
    #         'offset': 0.0,
    #     },
    #     'subtasks_parameters': {
    #         'lane_following': {
    #             'desired_velocity': 5.56, # m/s
    #             'safe_time_headway': 1.5, # s
    #             'max_acceleration': 2.73, # m/s^2
    #             'comfort_deceleration': 1.67, # m/s^2
    #             'acceleration_exponent': 4, 
    #             'minimum_distance': 4, # m
    #             'vehicle_length': 1, # m
    #         },
    #         'lane_keeping': {
    #             'far_distance': 30.0,
    #         },
    #     },
    # }
}