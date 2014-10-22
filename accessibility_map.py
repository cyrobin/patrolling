"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import numpy as np
from copy import copy,deepcopy

from geomaps import Geomap
from constant import *

class AccessibilityMap(Geomap):
    'A Geomap that specifically embodies robot accessibility'

    """ deep copy an AccessibilityMap -- shared the Geomap attributes but keep
    its own metrics and its own image (=values) """
    def __deepcopy__(self,memo):
        my_copy = copy(self)

        # Real (deep) copy of the image
        my_copy.image = np.copy(self.image)

        return my_copy

    """  Compute a <weighted_map> by weighting the AccessibilityyMap using the
    robot sensor, the UtilityMap <utility_map> and the sampled observable
    points given as arguments."""
    def built_weighted_map( self, sensor, utility_map, points ) :

        self.check_coherence(utility_map)

        weighted_map = deepcopy(self)

        # local references
        utilities = utility_map.image
        weights = weighted_map.image.astype(np.float, copy=False)

        # weight self.image
        for (p,w) in np.ndenumerate(weights):
            if w > 0:
                # FIXME 0.01 is a magic number which avoid to simply discard
                # positions that are not in range of an observable points (instead
                # they have a very low value)
                weights[p] = w * ( 0.01 + sum( sensor(p,q) for q in points ) )

        # Normalization
        weighted_map.image = (255 * weights / weights.max() ).astype(np.uint8, copy=False)

        return weighted_map

    """ Return the existing connections between the <points> in the <self>.
    <paths> refers to the positions in self.points and may be seen as a sparse
    matrix indicating the connections between the accessible points. One may set a
    limit for the number of connections by points, setting a maximum branching
    factor. The choice of the connexions are related to their cost, computed through
    the <f_cost> function."""
    # TODO It currently uses the distance, which obviously not reliable (and
    # costly?) => use a djikstra one to all, which stops after the first three (=>
    # get path !)
    # TODO one may also use a cost_table which precompute every thing
    def computed_paths( self, points, cost_function, branching_factor = 3 ):
        paths = {}

        #The first element of <points> is considered as 'non-return' position
        for p in points:
            links = sorted(points, key=lambda x: cost_function(p,x))
            paths[p] = links[1:branching_factor+1]

        return paths



