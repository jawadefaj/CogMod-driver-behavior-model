exec(open("sys_path_hack.py").read())

import logging
import click
import carla
from datetime import date
import time
import os
from research import ResearchFactory
from lib import MapNames, SimulationMode


@click.command()
@click.option(
    '-h', '--host',
    metavar='str',
    default="127.0.0.1",
    type=str,
    help='Host'
    )
@click.option(
    '-p', 'port',
    metavar='number',
    default=2000,
    type=int,
    help='Port'
    )
@click.option(
    '-t', '--duration',
    metavar='number',
    default=60,
    type=int,
    help='Duration in seconds'
    )

@click.option(
    '-r', '--record',
    metavar='boolean',
    default=True,
    type=bool,
    help='Start recording'
    )

@click.option(
    '-p', '--play',
    metavar='boolean',
    default=False,
    type=bool,
    help='Start recording'
    )
@click.option(
    '-a', '--actor',
    metavar='str',
    default=None,
    type=str,
    help='Actor to follow while playing'
    )
@click.option(
    '-f', '--file',
    metavar='str',
    default=None,
    type=str,
    help='File to play'
    )
    

def main(host, port, duration, record, play, actor, file):
    
    try:

        client = carla.Client(host, port)
        client.set_timeout(2.0)

        if play:
            playSession(client, file, duration, actor)
        elif record:
            recordSession(client, duration)
    except Exception as e:
        print(e)



def recordSession(client: carla.Client, duration: int):
    abspath = "C:/AV/Carla/CARLA_0.9.13/WindowsNoEditor/PythonAPI/experiments/logs/recordings"
    # path = os.path.join("../logs/recordings", f"{date.today().strftime('%Y-%m-%d-%H-%M')}.log")
    path = os.path.join(abspath, f"{date.today().strftime('%Y-%m-%d-%H-%M')}.log")
    
    try:
        client.start_recorder(path)
        print(f"Start recording at {path}")
        if (duration > 0):
            time.sleep(duration)
        else:
            while True:
                client.world.wait_for_tick()
                # time.sleep(0.1)
    finally:
        print("Stop recording")
        client.stop_recorder()


def playSession(client, file, duration, actor):
    try:
        print(client.show_recorder_file_info(file, True))
        client.replay_file(file, 0, duration, 24)
        if (duration > 0):
            time.sleep(duration)
        else:
            while True:
                client.world.wait_for_tick()
                # time.sleep(0.1)
    finally:
        print("Stop playing")
        client.stop_replayer(False)

if __name__ == '__main__':
    main()