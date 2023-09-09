from ..CogModEnum import GazeDirection

from lib import utils

Gaze_Settings1 = {
    GazeDirection.CENTER: (0, 30, 100, utils.red),
    GazeDirection.LEFT:  (70, 70, 70, utils.green),
    GazeDirection.RIGHT:  (-70, 70, 70, utils.blue),
    GazeDirection.LEFTBLINDSPOT:  (130, 30, 20, utils.green),
    GazeDirection.RIGHTBLINDSPOT:  (-130, 30, 20, utils.blue),
    GazeDirection.LEFTMIRROR:  (160, 20, 50, utils.green),
    GazeDirection.RIGHTMIRROR:  (-160, 20, 50, utils.blue),
    GazeDirection.REARMIRROR:  (179, 45, 100, utils.red),
    GazeDirection.SPEEDOMETER:  (0, 50, 5, utils.cyan),
    GazeDirection.CENTERCONSOLE:  (0, 10, 5, utils.cyan),
    GazeDirection.OTHER:  (0, 90, 5, utils.cyan),
}