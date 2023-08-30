



# ScenarioState with states:
# 1. PENDING
# 2. STARTING
# 3. RUNNING
# 4. ENDED

from enum import Enum


class ScenarioState(Enum):
    PENDING = 1
    STARTING = 2
    RUNNING = 3
    FINISHED = 4