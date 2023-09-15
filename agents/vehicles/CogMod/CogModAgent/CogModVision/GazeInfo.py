



class GazeInfo:
    def __init__(self, movement_factor: float, fixation_area: tuple, fixation_probability: float) -> None:
        self.movement_factor = movement_factor
        self.fixation_area = fixation_area
        self.fixation_probability = fixation_probability
    
    def get_movement_factor(self):
        return self.movement_factor
    
    # this area is a derived from the following information:
    # 1. the gaze direction (angle wrt to the vehicle's heading) clockwise/right negetive
    # 2. FOV (field of view) angle the gaze direction bisects the FOV angle
    # 3. length of the gaze direction
    def get_fixation_area(self):
        return self.fixation_area
    
    def get_fixation_probability(self):
        return self.fixation_probability
