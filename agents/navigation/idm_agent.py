import carla
from agents.navigation.basic_agent import BasicAgent


class IDMAgent(BasicAgent):
    
    # IDM agents only works for HighD Map
    # TODO: add a check for this
    
    def __init__(self, vehicle, target_speed=20, opt_dict={}):
        super().__init__(vehicle, target_speed, opt_dict)
        
        self.logger.info("IDMAgent: init")
    pass