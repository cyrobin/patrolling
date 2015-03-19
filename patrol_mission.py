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

    #print "Starting Loop !"
    #mission.loop(20,False,'Perception-based TOP')
    #mission.loop(10,False,'Perception-based SP')

    #mission.decentralized_loop(20,False,'Perception-based TOP')

    mission.parallel_loop(20,False,'Perception-based TOP')

    #mission.sample_objective()
    #mission.sample_all_positions()
    #for robot in mission.team:
        #robot.display_weighted_map()
    #mission.display_situation()

    mission.display_situation()

    #print "Done."
