#!/usr/bin/python
"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

from mission import *
from constant import *

from sys import argv, exit
from timer import Timer


if __name__ == "__main__":

    with Timer('Loading mission file'):
        json_mission = loaded_mission(argv[1])
        mission =  Mission (  json_mission )

    print "Starting Loop !"
    #mission.loop_once('Perception-based TSP')
    #mission.loop(10,True,'Perception-based TSP')

    #mission.decentralized_loop_once()
    mission.decentralized_loop(5,False,'Perception-based TSP')

    print "Updating..."
    mission.update()
    mission.dump_situation()
    #for robot in mission.team:
        #robot.display_weighted_map()
    mission.display_situation()

    mission.print_metrics()

    print "Done."
