exec(open("sys_path_hack.py").read())

import logging
import click

from research import ResearchFactory
from lib import MapNames

filterSettings = {
    'ego_type': 'car',
    'preceding_type': 'car',
}

@click.command()
@click.option(
    '--max_ticks',
    metavar='number',
    default=1000,
    type=int,
    help='Number of ticks the simulator will run'
    )
def reasearch_idm(max_ticks):
    research = ResearchFactory.createResearchIDM(map=MapNames.HighWay_Ring, defaultLogLevel=logging.WARN, filterSettings=filterSettings)
    research.maxStepsPerCrossing = max_ticks
    research.run(maxTicks=max_ticks)


if __name__ == '__main__':
    reasearch_idm()