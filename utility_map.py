"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import numpy as np

from geomaps import Geomap
from constant import *

class UtilityMap(Geomap):
    'A Geomap that specifically embodies utility'

    def __init__(self, geofile):
        Geomap.__init__(self,geofile)

        # Use int32 instead of uint8 as utility can grow arbitrarily high
        self.image = np.uint32(self.image)

        # Normalization at init (max = 100)
        max_utility = np.amax(self.image)
        self.image = 100 * self.image / max_utility

        self.max_utility_over_time = [100]

    """ When the geomap embodies utility, update the map value using a sensor
    model and a set of observation (view point)."""
    def update_utility(self, team):

        for (observable, utility) in np.ndenumerate(self.image):
            if utility > 0:
                try:
                    best_view = max(robot.sensor( viewpoint , observable ) \
                      for robot in team for viewpoint in robot.plan )
                except ValueError: # when no plan was computed before
                    best_view = 0

                self.image[ observable ] = utility * ( 1 - best_view ) + UTILITY_GROWTH_BY_PERIOD


        max_utility = np.amax(self.image)
        self.max_utility_over_time = [max_utility]

