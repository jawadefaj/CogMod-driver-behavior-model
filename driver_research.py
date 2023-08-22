import logging
from lib import SimulationMode

from lib.MapManager import MapNames
from research import ResearchFactory
import carla
from agents.vehicles.CogMod.Settings.DriverSettings import driver_profile

def DriverResearch():

    max_ticks = 100000
    host = "127.0.0.1"
    port = 2000
    timeout=10

    default_log_level = logging.INFO
    output_dir = "logs"
    simulation_mode = SimulationMode.SYNCHRONOUS

    map_name =  MapNames.HighWay_Ring
    highD_path = r'D:\\highD_data\\highD_dataset'
    dataset_ids = ['01']
    stable_height_path = r"C:\\Users\\abjawad\\Documents\\GitHub\\cogmod-driver-behavior-model\\settings\\stable_height.csv"
    pivot = carla.Transform(carla.Location(x=0, y=-22, z=0))
    base_distance = 200
    
    idm_driver_profile = driver_profile['idm1'] # we simulate idm once
    
    cogmod_driver_profile =  driver_profile['cogmod1']
    n_repeat = 2
    
    car_follow_filter = {
                            'ego_type': 'Car',
                            'preceding_type': 'Car',
                            'time_duration': 5,
                            'distance_threshold': 50,
                        }
    
    
    research = ResearchFactory.createResearchDriverIDMvCogMod(maxTicks=max_ticks,
                                                              host=host,
                                                              port=port,
                                                              timeout=timeout,
                                                              default_log_level=default_log_level,
                                                              output_dir=output_dir,
                                                              simulation_mode=simulation_mode,
                                                              map_name=map_name,
                                                              high_d_path=highD_path,
                                                              stable_height_path=stable_height_path,
                                                              dataset_ids=dataset_ids,
                                                              pivot=pivot,
                                                              base_distance=base_distance,
                                                              idm_profile=idm_driver_profile,
                                                              cogmod_profile=cogmod_driver_profile,
                                                              n_repetitions=n_repeat,
                                                              car_follow_filter=car_follow_filter)
if __name__ == '__main__':
    DriverResearch()