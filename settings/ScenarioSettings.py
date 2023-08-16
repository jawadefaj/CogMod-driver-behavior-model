from lib.MapManager import MapNames
import carla
from agents.vehicles.CogMod.Settings.DriverSettings import driver_profile

scenarios = {
    
    "scenario1": {
        "map": MapNames.HighWay_Ring,
        "high_d_path": f'D:\\highD_data\\highD_dataset',
        "dataset_id": '01',
        "stable_height_path": f"C:\\Users\\abjawad\\Documents\\GitHub\\cogmod-driver-behavior-model\\settings\\stable_height.csv",
        "pivot": carla.Transform(carla.Location(x=0, y=-22, z=0)),
        "car_follow_settings":{
            'ego_type': 'Car',
            'preceding_type': 'Car',
            'time_duration': 5,
            'distance_threshold': 50,
        },
        "base_distance": 800,
        "cogmod_agent": {
            "source": None,
            "destination": None,
            "driver_profile": driver_profile['driver2']
        },
    },
}



