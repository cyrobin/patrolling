"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import numpy as np
from math import sqrt,log

from geomaps import Geomap
from constant import *

class VisibilityMap(Geomap):
    'A Geomap that specifically embodies robot visibility (sensing)'

    """ This function return an appropriate sensor model defined by its name,
    its coefficient, its range, and the map in use. A sensor model is a
    function that takes as argument the sensor position and the position of
    the sensed object / area, to return the quality of the observation, which
    is a float between 0 and 1, 1 being perfectly observed."""
    # TODO The models currently only consider obstacles( and no height, etc) :
    # one want to consider viewline in a nearby future !
    def built_sensor_model(geomap, name, coef, sensor_range):

        """ Here follows sensor models of various quality. """
        def constant_sensor(coef):
            def _sensor_model(p,q):
                d = geomap.euclidian_distance_pix2meters(p,q)
                if d > sensor_range :
                    return 0
                else:
                    return 1
            return  _sensor_model

        def linear_sensor(coef):
            def _sensor_model(p,q):
                d = geomap.euclidian_distance_pix2meters(p,q)
                if d == 0:
                    return 1
                elif d > sensor_range :
                    return 0
                else:
                    return min( 1, coef / d )
            return  _sensor_model

        def square_sensor(coef):
            def _sensor_model(p,q):
                d = geomap.euclidian_distance_pix2meters(p,q)
                if d == 0:
                    return 1
                elif d > sensor_range :
                    return 0
                else:
                    return min( 1, coef / sqrt(d) )
            return  _sensor_model

        def log_sensor(coef):
            def _sensor_model(p,q):
                d = geomap.euclidian_distance_pix2meters(p,q)
                if d <= 1:
                    return 1
                elif d > sensor_range :
                    return 0
                else:
                    return min( 1, coef / log(d) )
            return  _sensor_model

        def quadratic_sensor(coef):
            def _sensor_model(p,q):
                d = geomap.euclidian_distance_pix2meters(p,q)
                if d == 0:
                    return 1
                elif d > sensor_range :
                    return 0
                else:
                    return min( 1, coef / d**2 )
            return  _sensor_model

        """ This dictionnary lists available sensor models."""
        available_sensor_models = { \
                'constant'  : constant_sensor,  \
                'linear'    : linear_sensor,    \
                'square'    : square_sensor,    \
                'log'       : log_sensor,       \
                'quadratic' : quadratic_sensor, \
                }

        try:
            sensor_model = available_sensor_models[name](coef)
        except KeyError:
            raise ValueError('[VisibilityMap] Unknown sensor name. Please choose another model.')

        return sensor_model


