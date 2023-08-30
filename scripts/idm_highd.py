# exec(open("sys_path_hack.py").read())

import logging
import click
import random
from research import ResearchFactory
from lib import MapNames
from lib.SimulationMode import SimulationMode

from sys import path
import os
from os.path import dirname as dir
    
filtered_follow_meta_path = "C:\\Users\\abjawad\\Documents\\GitHub\\cogmod-driver-behavior-model\\lib\\highD\\follow_meta.csv"

@click.command()
@click.option(
    '--max_ticks',
    metavar='number',
    default=1000,
    type=int,
    help='Number of ticks the simulator will run'
    )

def reasearch_idm(max_ticks):
    random.seed(52)
    research = ResearchFactory.createResearchIDM(map=MapNames.HighWay_Ring, 
                                                 defaultLogLevel=logging.DEBUG, 
                                                 filterSettings=filtered_follow_meta_path,
                                                 simulationMode=SimulationMode.SYNCHRONOUS)
    research.run(maxTicks=max_ticks)


if __name__ == '__main__':
    
    parentdir = dir(path[0])
    # path.append(parentdir)
    
    path.insert(0, parentdir)
    os.chdir(parentdir)
    __package__ = "scripts"
    
    reasearch_idm()