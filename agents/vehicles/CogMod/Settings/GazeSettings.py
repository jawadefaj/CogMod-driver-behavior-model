from ..CogModEnum import GazeDirection

from lib import utils



class GazeData:
    def __init__(self, eye_movement_angle: float, area: tuple, vis_color) -> None:
        self.eye_movement_angle = eye_movement_angle
        self.area = area
        self.vis_color = vis_color
    
    def get_eye_movement_angle(self):
        return self.eye_movement_angle
    
    def get_area(self):
        return self.area
    
    def get_color(self):
        return self.vis_color


Gaze_Settings1 = {
    GazeDirection.CENTER: GazeData(0, (0, 30, 100), utils.red),
    GazeDirection.LEFT: GazeData(45, (70, 70, 70), utils.green),
    GazeDirection.RIGHT: GazeData(45, (-70, 70, 70), utils.blue),
    
    GazeDirection.LEFTBLINDSPOT: GazeData(90, (130, 30, 20), utils.green),
    GazeDirection.RIGHTBLINDSPOT: GazeData(90, (-130, 30, 20), utils.blue),
    
    GazeDirection.LEFTMIRROR: GazeData(55, (160, 20, 50), utils.green),
    GazeDirection.RIGHTMIRROR: GazeData(55, (-160, 20, 50), utils.blue),
    GazeDirection.REARMIRROR: GazeData(35, (179, 45, 100), utils.red),
    
    GazeDirection.SPEEDOMETER: GazeData(30, (0, 50, 5), utils.cyan),
    GazeDirection.CENTERCONSOLE: GazeData(30, (0, 10, 5), utils.cyan),
    GazeDirection.OTHER: GazeData(45, (0, 90, 5), utils.cyan),
}