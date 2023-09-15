from ..CogModEnum import GazeDirection

from lib import utils
from agents.vehicles.CogMod.CogModAgent.CogModVision.GazeInfo import GazeInfo



lane_follow_gaze = {

    GazeDirection.CENTER: GazeInfo( movement_factor=0, fixation_area=(0, 30, 100), fixation_probability=0.5),
    GazeDirection.LEFT: GazeInfo( movement_factor=45, fixation_area=(70, 70, 70), fixation_probability=0.1),
    GazeDirection.RIGHT: GazeInfo( movement_factor=45, fixation_area=(-70, 70, 70), fixation_probability=0.1),

    GazeDirection.LEFTBLINDSPOT: GazeInfo( movement_factor=90, fixation_area=(130, 30, 20), fixation_probability=0.05),
    GazeDirection.RIGHTBLINDSPOT: GazeInfo( movement_factor=90, fixation_area=(-130, 30, 20), fixation_probability=0.05),

    GazeDirection.LEFTMIRROR: GazeInfo( movement_factor=55, fixation_area=(160, 20, 50), fixation_probability=0.01),
    GazeDirection.RIGHTMIRROR: GazeInfo( movement_factor=55, fixation_area=(-160, 20, 50), fixation_probability=0.01),
    GazeDirection.REARMIRROR: GazeInfo( movement_factor=35, fixation_area=(179, 45, 100), fixation_probability=0.04),

    GazeDirection.SPEEDOMETER: GazeInfo( movement_factor=30, fixation_area=(0, 50, 5), fixation_probability=0.03),
    GazeDirection.CENTERCONSOLE: GazeInfo( movement_factor=30, fixation_area=(0, 10, 5), fixation_probability=0.03),
    GazeDirection.OTHER: GazeInfo( movement_factor=45, fixation_area=(0, 90, 5), fixation_probability=0.03),
}

vehicle_follow_gaze = {

    GazeDirection.CENTER: GazeInfo( movement_factor=0, fixation_area=(0, 30, 100), fixation_probability=0.6),
    GazeDirection.LEFT: GazeInfo( movement_factor=45, fixation_area=(70, 70, 70), fixation_probability=0.1),
    GazeDirection.RIGHT: GazeInfo( movement_factor=45, fixation_area=(-70, 70, 70), fixation_probability=0.1),

    GazeDirection.LEFTBLINDSPOT: GazeInfo( movement_factor=90, fixation_area=(130, 30, 20), fixation_probability=0.05),
    GazeDirection.RIGHTBLINDSPOT: GazeInfo( movement_factor=90, fixation_area=(-130, 30, 20), fixation_probability=0.05),

    GazeDirection.LEFTMIRROR: GazeInfo( movement_factor=55, fixation_area=(160, 20, 50), fixation_probability=0.01),
    GazeDirection.RIGHTMIRROR: GazeInfo( movement_factor=55, fixation_area=(-160, 20, 50), fixation_probability=0.01),
    GazeDirection.REARMIRROR: GazeInfo( movement_factor=35, fixation_area=(179, 45, 100), fixation_probability=0.04),

    GazeDirection.SPEEDOMETER: GazeInfo( movement_factor=30, fixation_area=(0, 50, 5), fixation_probability=0.03),
    GazeDirection.CENTERCONSOLE: GazeInfo( movement_factor=30, fixation_area=(0, 10, 5), fixation_probability=0.03),
    GazeDirection.OTHER: GazeInfo( movement_factor=45, fixation_area=(0, 90, 5), fixation_probability=0.03),
}

