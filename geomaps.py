"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import gdal
import numpy as np
from math import sqrt,log
import time

from wrg import WeightedRandomGenerator
from constant import *

class Geomap:
    'Class storing geofile data'

    global VERBOSE

    """ Load a geofile (tiff + xml description) """
    def __init__(self, geofile):
        # get GeoTiff for scale info
        self.geotiff = gdal.Open(geofile)
        self.meta    = self.geotiff.GetMetadata()

        if not 'CUSTOM_X_ORIGIN' in self.meta:
            print('error: %s.aux.xml missing' % self.geofile)
            exit(1)

        self.tf      = self.geotiff.GetGeoTransform()
        self.band    = self.geotiff.GetRasterBand(1)
        self.image   = self.band.ReadAsArray() # numpy array
        self.width   = self.geotiff.RasterXSize
        self.height  = self.geotiff.RasterYSize
        self.scale_x = self.tf[1]
        self.scale_y = self.tf[5]
        self.utm_x   = self.tf[0]
        self.utm_y   = self.tf[3]
        self.custom_x_origin = float(self.meta['CUSTOM_X_ORIGIN'])
        self.custom_y_origin = float(self.meta['CUSTOM_Y_ORIGIN'])

        if VERBOSE:
            print "Image is {}x{}".format(self.width,self.height)

    """ Check the scale, origin and size coherence between two geomaps.
    If it failt, throw an exception (ValueError)."""
    def check_coherence(self, geomap):
        if   self.tf              != geomap.tf              \
          or self.width           != geomap.width           \
          or self.height          != geomap.height          \
          or self.custom_x_origin != geomap.custom_x_origin \
          or self.custom_y_origin != geomap.custom_y_origin :
              raise ValueError("Geomaps does not correspond to each other.")

    """ Sample <n> points in the <geomap>.
    Consider the geomap has a discrete distribution of probability used for the
    sampling. One can set a minimal distance to respect between the sampled points,
    and constrained them to a given area."""
    def sampled_points( self, n, min_dist = 0, area = None ):
        points = []

        if area:
            [(xmin,ymin),(xmax,ymax)] = area

            # Bounded the area to the current map
            xmin = max( 0, xmin )
            ymin = max( 0, ymin )
            xmax = min(self.height + 1, xmax )
            ymax = min(self.width  + 1, ymax )

            h = min( int(xmax-xmin), self.height)
            w = min( int(ymax-ymin), self.width )

            wrg = WeightedRandomGenerator(self.image[xmin:xmax,ymin:ymax])
        else:
            wrg = WeightedRandomGenerator(self.image)

        """ Auxiliary function that check the distance of a given point <_p> to a
        list <_points> : if _p is not too close from others sampled points (>dist),
        then the function return True (ie one can keep <_p> as a valid sample> """
        def _not_too_close (_p, _points):
            for q in points:
                if euclidian_distance(_p,q) < min_dist:
                    return False
            return True

        i = 0
        t_start = time.time()
        while i < n and (time.time() - t_start) < SAMPLING_TIME_OUT :
            idx = wrg()
            # Beware of the order (height,width) (set empirically...)
            if area:
                try:
                    (x,y) = np.unravel_index( idx, (h, w) )
                except ValueError:
                    continue
                p = (x+xmin, y+ymin)
                if _not_too_close(p, points):
                    points.append(p)
                    i+=1
            else:
                p= np.unravel_index( idx, (self.height, self.width ) )
                if _not_too_close(p, points):
                    points.append(p)
                    i+=1

        #TODO handle this with exceptions
        if (time.time() - t_start) > SAMPLING_TIME_OUT :
            print "!WARNING! Sampling timed out ({} / {} points sampled)".format( \
                    len(points), n )

        return points

    """ Return the distance beween two 2D points.
    Currently use the euclidian distance. Keep the length unit. """
    def dist( self, p, q ):
        return euclidian_distance(p,q)

    """ Translate pixel coordinates into utm """
    def point_pix2utm(self, x, y):
        return [ x * self.scale_x + self.utm_x ,
                 y * self.scale_y + self.utm_y ]

    """ Translate pixel coordinates into utm with custom origin """
    def point_pix2custom(self, x, y):
        p = self.point_pix2utm(x, y)
        return [p[0] - self.custom_x_origin,
                p[1] - self.custom_y_origin]

    """ Transform pixels coordinates into one-liner index """
    #def point_pix2idx(self, x, y):
        #return y*self.width + x
        #TODO use flat / ravel instead ? zith coords ?
        #return image.ravel(

    """ Transform one-liner index into image pixels coordinates """
    def point_idx2pix( self, idx ):
        return np.unravel_index( idx, (self.height,self.width) )

    """ Translate pixel length into meters length """
    def length_pix2meter(self, d):
        if ( abs(self.scale_x) == abs(self.scale_y) ):
            return d * abs(self.scale_x)
        else:
            print self.scale_x
            print self.scale_y
            raise RuntimeError("Trying to scale a map that has different axis scales")

    """ Translate meters length into pixel length """
    def length_meter2pix(self, d):
        if (abs(self.scale_x) == abs(self.scale_y) ):
            return d / abs(self.scale_x)
        else:
            print self.scale_x
            print self.scale_y
            raise RuntimeError("Trying to scale a map that has different axis scales")

""" Return the euclidian distance beween two 2D points.
Keep the length unit. """
def euclidian_distance( (x1,y1), (x2,y2) ):
    return sqrt( ( (x1-x2) )**2 \
               + ( (y1-y2) )**2 )

""" This function return an appropriate sensor function defined by its name,
its coefficient, its range, and the map in use. A sensor function takes as
argument  the sensor position and the position of the sensed object / area,
to return the quality of the observation, which is a float between 0 and 1, 1
being perfectly observed."""
def built_sensor_function(geomap, name, coef, sensor_range):
    """ Here follows sensor functions of various quality. """
    def linear_sensor(coef):
        def _function(p,q):
            d = euclidian_distance(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix( sensor_range ):
                return 0
            else:
                return min( 1, coef / d )
        return  _function

    def square_sensor(coef):
        def _function(p,q):
            d = euclidian_distance(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix( sensor_range ):
                return 0
            else:
                return min( 1, coef / sqrt(d) )
        return  _function

    def log_sensor(coef):
        def _function(p,q):
            d = euclidian_distance(p,q)
            if d <= 1:
                return 1
            elif d > geomap.length_meter2pix( sensor_range ):
                return 0
            else:
                return min( 1, coef / log(d) )
        return  _function

    def quadratic_sensor(coef):
        def _function(p,q):
            d = euclidian_distance(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix( sensor_range ):
                return 0
            else:
                return min( 1, coef / d**2 )
        return  _function

    """ This dictionnary lists available sensors function. """
    available_sensor_models = {\
            'linear': linear_sensor, \
            'square': square_sensor, \
            'log': log_sensor, \
            'quadratic': quadratic_sensor, \
            }

    try:
        sensor_model = available_sensor_models[name](coef)
    except KeyError:
        raise ValueError('Unknown sensor name. Please choose another model.')

    return sensor_model

