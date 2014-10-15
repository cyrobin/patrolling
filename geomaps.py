"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import gdal
import numpy as np
from math import sqrt,log
from copy import copy
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
                self.image[ observable ] = min( 255, \
                        utility * ( 1 - best_view ) + UTILITY_GROWTH_BY_PERIOD )
                # TODO have a specific geomap class for utility ?
                # TODO do not limit utility to 255 ? (use newimage = np.uint64(image) )

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

"""  Compute a <wmap> by weighting pos_map using the robot sensor,
the utility map <u_map> and the sampled observable points given as
arguments."""
# FIXME ensure that all geomaps are concistant ??
# (or deal with it ?!)
def built_weighted_map( pos_map, sensor, u_map, points ) :

    # Copy pos_map
    weight_map = copy(pos_map)

    # local references
    image = u_map.image
    wimage = weight_map.image.astype(np.float, copy=False)

    # weight pos_map.image
    for (p,w) in np.ndenumerate(wimage):
        if w > 0:
            # FIXME 0.01 is a magic number which avoid to simply discard
            # positions that are not in range of an observable points (instead
            # they have a very low value)
            wimage[p] = w * ( 0.01 + sum( sensor(p,q) for q in points ) )

    # Normalization
    weight_map.image = (255 * wimage / wimage.max() ).astype(np.uint8, copy=False)

    return weight_map

""" Sample <n> points in the <geomap>.
Consider the geomap has a discrete distribution of probability used for the
sampling. One can set a minimal distance to respect between the sampled points,
and constrained them to a given area."""
def sampled_points( geomap, n, min_dist = 0, area = None ):
    points = []

    if area:
        [(xmin,ymin),(xmax,ymax)] = area

        # Bounded the area to the current map
        xmin = max( 0, xmin )
        ymin = max( 0, ymin )
        xmax = min(geomap.height + 1, xmax )
        ymax = min(geomap.width  + 1, ymax )

        h = min( int(xmax-xmin), geomap.height)
        w = min( int(ymax-ymin), geomap.width )

        wrg = WeightedRandomGenerator(geomap.image[xmin:xmax,ymin:ymax])
    else:
        wrg = WeightedRandomGenerator(geomap.image)

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
            p= np.unravel_index( idx, (geomap.height, geomap.width ) )
            if _not_too_close(p, points):
                points.append(p)
                i+=1

    #TODO handle this with exceptions
    if (time.time() - t_start) > SAMPLING_TIME_OUT :
        print "!WARNING! Sampling timed out ({} / {} points sampled)".format( \
                len(points), n )

    return points

""" Return the existing connections between the <points> in the <geomap>.
<paths> refers to the positions in self.points and may be seen as a sparse
matrix indicating the connections between the accessible points. One may set a
limit for the number of connections by points, setting a maximum branching
factor. The choice of the connexions are related to their cost, computed through
the <f_cost> function."""
def computed_paths( geomap, points, cost_function, branching_factor = 3 ):
    paths = {}

    # TODO It currently use the distance, which obviously not reliable (and
    # costly?) => use a djikstra 1 to all, which stop after the first three (=>
    # get path !)

    #The first element of <points> is considered as 'non-return' position
    for p in points:
        links = sorted(points, key=lambda x: cost_function(p,x))
        paths[p] = links[1:branching_factor+1]

    return paths

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

