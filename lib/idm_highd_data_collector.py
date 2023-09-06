


from enum import Enum
import os
import pandas as pd

from lib import LoggerFactory




class IdmHighDDataCollector():

    def __init__(self):

        self.data_dict = None
        self.logger = LoggerFactory.create("DataCollector")
        self.trajectory_DF = pd.DataFrame()
        self.initDataDict()

        self.logger.info("DataCollector idm-highD initialized")


    def initDataDict(self):
        
        self.statDict = {
            "scenario_id": [], "exec_num": [], "frame": [], "scenario_status": [],

            "ego_id": [], "c_x": [], "c_y": [], 
            "c_speed": [], "c_acceleration": [],
            "c_steer": [], "c_throttle": [], "c_brake": [],

            "preceding_id": [], "a_x": [], "a_y": [], 
            "a_speed": [], "a_acceleration": [],
            "a_steer": [], "a_throttle": [], "a_brake": [],
        }

  
        
    
    def collectStats(self, scenario_id, exec_num, frame, scenario_status,
                     idm_vehicle, actor_vehicle):
        
        self.statDict["scenario_id"].append(scenario_id)
        self.statDict["exec_num"].append(exec_num)
        self.statDict["frame"].append(frame)
        self.statDict["scenario_status"].append(scenario_status)

        self.statDict["ego_id"].append(idm_vehicle.id)
        self.statDict["c_x"].append(idm_vehicle.get_location().x)
        self.statDict["c_y"].append(idm_vehicle.get_location().y)
        
        self.statDict["c_speed"].append(idm_vehicle.get_velocity().length())
        self.statDict["c_acceleration"].append(idm_vehicle.get_acceleration().length())
        
        self.statDict["c_steer"].append(idm_vehicle.get_control().steer)
        self.statDict["c_throttle"].append(idm_vehicle.get_control().throttle)
        self.statDict["c_brake"].append(idm_vehicle.get_control().brake)

        self.statDict["preceding_id"].append(actor_vehicle.id)
        self.statDict["a_x"].append(actor_vehicle.get_location().x)
        self.statDict["a_y"].append(actor_vehicle.get_location().y)
        
        self.statDict["a_speed"].append(actor_vehicle.get_velocity().length())
        self.statDict["a_acceleration"].append(actor_vehicle.get_acceleration().length())
        
        self.statDict["a_steer"].append(actor_vehicle.get_control().steer)
        self.statDict["a_throttle"].append(actor_vehicle.get_control().throttle)
        self.statDict["a_brake"].append(actor_vehicle.get_control().brake)
        pass
    
    def updateTrajectoryDF(self):
        df = pd.DataFrame.from_dict(self.statDict)
        self.trajectory_DF = pd.concat([self.trajectory_DF, df], ignore_index=True)
        self.initDataDict()

    
    def saveCSV(self, filename, filepath):
        filepath = os.path.join(os.getcwd(), filepath, filename)
        print("filepath", filepath)
        self.logger.info(f"Saving CSV file to {filepath + '.csv'}")
        self.trajectory_DF.to_csv(filepath + ".csv", index=False)
        pass

    


    



