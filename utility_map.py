"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import numpy as np
from copy import copy,deepcopy

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

        # Utility growth "mask", used to differenciate various area in the map
        # (the ones we care about, the ones we care less, and the one we do not
        # care at all
        # TODO load this from a separate file (independant from init value of
        # the utility
        self.utility_growth_mask = self.image.astype(np.float, copy=True) / 100

        # Performance metrics
        self.size = sum( [1 for (q,w) in enumerate(self.utility_growth_mask.flat) if w >0 ] )
        if VERBOSITY_LEVEL > 2:
            print "[UtilityMap] {} observable positions are monitored.".format(self.size)

        max_utility = 100
        sum_utility = np.sum( self.image, dtype = np.int64 )
        average_diff_utility = 0
        average_utility = sum_utility / self.size

        self.past_max_utilities     = [ max_utility     ]
        self.past_sum_utilities     = [ sum_utility     ]
        self.past_average_diff_utilities    = [ average_diff_utility ]
        self.past_average_utilities = [ average_utility ]

    """ deep copy a UtilityMap -- shared the Geomap attributes but keep its own
    metrics and its own image (=values) """
    def __deepcopy__(self,memo):
        my_copy = copy(self)

        # Copy of UtilityMap attributes
        my_copy.utility_growth_mask         = deepcopy(self.utility_growth_mask)
        my_copy.size                        = deepcopy(self.size)
        my_copy.past_max_utilities          = deepcopy(self.past_max_utilities)
        my_copy.past_sum_utilities          = deepcopy(self.past_sum_utilities)
        my_copy.past_average_utilities      = deepcopy(self.past_average_utilities)
        my_copy.past_average_diff_utilities = deepcopy(self.past_average_diff_utilities)

        my_copy.image = np.copy(self.image)

        return my_copy

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

                self.image[ observable ] = utility * ( 1 - best_view ) \
                    + self.utility_growth_mask[observable] * UTILITY_GROWTH_BY_PERIOD

        # Update performance metrics
        max_utility     = np.amax(self.image)
        sum_utility     = np.sum( self.image, dtype = np.int64 )
        average_diff_utility    = (sum_utility - np.int64(self.past_sum_utilities[-1])) / self.size
        average_utility = sum_utility / self.size

        self.past_max_utilities.append( max_utility )
        self.past_sum_utilities.append( sum_utility )
        self.past_average_diff_utilities.append( average_diff_utility )
        self.past_average_utilities.append( average_utility )

    """ Display various metrics """
    # TODO TO BE COMPLETED (standard deviation, etc.)
    def print_metrics(self):

        # TODO log this
        #if VERBOSITY_LEVEL > 0:

        if VERBOSITY_LEVEL > 1:
            print "[UtilityMap:metrics] Max utility over time is {}, out of:".format( \
                max(self.past_max_utilities))
            print self.past_max_utilities

            print "[UtilityMap:metrics] Max average of utilities over time is {}, out of:".format( \
                max(self.past_average_utilities))
            print self.past_average_utilities

            print "[UtilityMap:metrics] Worst average utility evolution over time is {}, out of:".format( \
                    max(self.past_average_diff_utilities[1:]))
            print self.past_average_diff_utilities

            print "[UtilityMap:metrics] Best average utility evolution over time is {}, out of:".format( \
                    min(self.past_average_diff_utilities[1:]))
            print self.past_average_diff_utilities


