exec(open("sys_path_hack.py").read())

import logging
from lib import SimulationMode
from lib.MapManager import MapNames
from research.ResearchFactory import ResearchFactory

def main():
    host= "127.0.0.1"
    port = 2000
    output_dir = "logs"
    defaultLogLevel = logging.INFO
    _map = MapNames.HighWay_Ring
    simulationMode = SimulationMode.SYNCHRONOUS
    maxTicks = 2000
    research = ResearchFactory.createResearchCogMod(host=host, 
                                                    port=port, 
                                                    defaultLogLevel=defaultLogLevel, 
                                                    output_dir=output_dir, 
                                                    map=_map,
                                                    simulationMode=simulationMode,)
    research.run(maxTicks=maxTicks)

if __name__ == "__main__":
    main()