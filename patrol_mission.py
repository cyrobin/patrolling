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

    #with Timer('Sampling observable points'):
        #mission.sample_objective()
    #with Timer('Sampling positions'):
        #mission.sample_all_positions()

    #print "Solving..."
    #with Timer('Solving'):
        #mission.solve()
        ##mission.solve('Position-based TSP')

    #print "Displaying..."
    #for robot in mission.team:
        #robot.display_weighted_map()
    #mission.display_situation()

    #print "Updating poses and map"
    #mission.update_poses()
    #mission.update_map()


    print "Starting Loop !"
    #mission.loop_once()
    #mission.loop(5,True)
    mission.loop(10)

    mission.update()
    #for robot in mission.team:
        #robot.display_weighted_map()
    mission.display_situation()

    mission.print_metrics()

    print "Done."
