from .CogModController import VehiclePIDController
from .CogModVision.Gaze import Gaze
from .CogModVision.VisionManager import VisionManager
from .CognitiveModel.Map import LocalMap 
from .CognitiveModel.Servers import LongTermMemory, ComplexCognition, MotorControl
from ..CogModEnum import ServerType



class AgentIntializer():
    def __init__(self, 
                 vehicle,
                 destination,
                 driver_profile):

        self.driver_profile = driver_profile
        self.vehicle = vehicle
        # self.destination = destination
        
        # print('driver profile ', self.driver_profile, vehicle, destination)

        self.vision_settings = self.driver_profile['vision']

        self.vision_manager = self.set_vision_module()

        pass

    def set_vision_module(self):
        vision_manager = VisionManager(self.vehicle, self.vision_settings)
        return vision_manager
    