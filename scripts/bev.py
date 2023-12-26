exec(open("sys_path_hack.py").read())

import logging

from research import ResearchFactory
from lib import MapNames, SimulationMode


def researchBEV(max_ticks, map_name, simulationMode):
    research = ResearchFactory.createResearchBEV(
        map=map_name, 
        defaultLogLevel=logging.WARN, 
        simulationMode = simulationMode,
        )
    research.maxStepsPerCrossing = max_ticks
    research.run(maxTicks=max_ticks)


if __name__ == '__main__':
    map_name = MapNames.Town02_Opt
    max_ticks = 1000
    simulationMode = SimulationMode.ASYNCHRONOUS
    researchBEV(max_ticks, map_name, simulationMode)