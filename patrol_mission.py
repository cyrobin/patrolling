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
        mission = load_mission(argv[1])
        m =  Mission (  mission )
    with Timer('Sampling observable points'):
        m.sample_objective()
    with Timer('Sampling positions'):
        m.sample_all_positions()

    print "Solving..."
    with Timer('Solving'):
        m.solve()

    print "Displaying..."
    #for r in m.team:
        #r.display_wmap()
    m.display_situation()

    print "Updating pose:"
    m.update_poses()

    print "Done."
