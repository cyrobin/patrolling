"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import gdal
import random
import bisect
import operator as operator
import numpy as np
from math import sqrt,log
from copy import copy
import time

from constant import *

def accumulate(iterable, func=operator.add):
    'Return running totals (itertools.accumulate() in Python2)'
    # accumulate([1,2,3,4,5]) --> 1 3 6 10 15
    # accumulate([1,2,3,4,5], operator.mul) --> 1 2 6 24 120
    it = iter(iterable)
    total = next(it)
    yield total
    for element in it:
        total = func(total, element)
        yield total

class WeightedRandomGenerator(object):
    'Class of random generator on 2D numpy array distribution'

    """ Init the weighted random generator
    by computing the flat 1D array of cumulative weights. This allows to quickly
    generate weight random value. """
    def __init__(self, weights):
        self.totals = []
        running_total = 0

        #for w in weights.reshape(-1): # flattening (1D)
            #running_total += w
            #self.totals.append(running_total)

        # OR
        # TODO tic toc
        weights_uint64 = np.uint64(weights)
        self.totals=list( accumulate(weights_uint64.flat, func=operator.add) )


    """ Generate one weighted random item """
    def next(self):
        rnd = random.random() * self.totals[-1]
        idx = bisect.bisect_right(self.totals, rnd)
        return idx

    def __call__(self):
        return self.next()

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

    """ Return the distance beween two 2D points.
    Use the Euclidian distance and keep the length unit. """
    def dist( self, (x1,y1), (x2,y2) ):
        return sqrt( ( (x1-x2) )**2 \
                   + ( (y1-y2) )**2 )

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

"""  Compute a <wmap> by weighting pos_map using the robot sensor,
the utility map <u_map> and the sampled observable points given as
arguments."""
# FIXME ensure that all geomaps are concistant ??
# (or deal with it ?!)
def make_weighted_map( pos_map, sensor, u_map, points ) :

    # Copy pos_map
    wmap = copy(pos_map)

    # local references
    image = u_map.image
    wimage = wmap.image.astype(np.float, copy=False)

    # weight pos_map.image
    for (p,w) in np.ndenumerate(wimage):
        if w > 0:
            #FIXME 0.5 is a magic number
            wimage[p] = w * ( 0.01 + sum( sensor(p,q) for q in points ) )

    # Normalization
    wmap.image = (255 * wimage / wimage.max() ).astype(np.uint8, copy=False)

    return wmap

""" Sample <n> points in the <geomap>.
Consider the geomap has a discrete distribution of probability used for the
sampling. One can set a minimal distance to respect between the sampled points,
and constrained them to a given area."""
def sample_points( geomap, n, min_dist = 0, area = None ):
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
            if geomap.dist(_p,q) <  min_dist:
                return False
        return True

    i = 0
    tstart = time.time()
    while i < n and (time.time() - tstart) < SAMPLING_TIME_OUT :
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
    if (time.time() - tstart) > SAMPLING_TIME_OUT :
        print "!!! WARNING !!! Sampling timed out: only {} points sampled  \
instead of {} required.".format(len(points),n)

    return points

""" Return the existing connections between the <points> in the <geomap>.
<paths> refers to the positions in self.points and may be seen as a sparse
matrix indicating the connections between the accessible points. One may set a
limit for the number of connections by points, setting a maximum branching
factor. The choice of the connexions are related to their cost, computed through
the <f_cost> function."""
def compute_paths( geomap, points, f_cost, branching_factor = 3 ):
    paths = {}

    # TODO It currently use the distance, which obviously not reliable (and
    # costly?) => use a djikstra 1 to all, which stop after the first three (=>
    # get path !)

    #The first element of <points> is considered as 'non-return' position
    for p in points:
        links = sorted(points, key=lambda x: f_cost(p,x))
        paths[p] = links[1:branching_factor+1]

    return paths

""" This function return an appropriate sensor function defined by its name,
its coefficient, its range, and the map in use. A sensor function takes as
argument  the sensor position and the position of the sensed object / area,
to return the quality of the observation, which is a float between 0 and 1, 1
being perfectly observed."""
def make_sensor_function(geomap, name, coef, srange):
    """ Here follows sensor functions of various quality. """
    def linear_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix(srange):
                return 0
            else:
                return coef / d
        return  _function

    def square_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix(srange):
                return 0
            else:
                return coef / sqrt(d)
        return  _function

    def log_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d <= 1:
                return 1
            elif d > geomap.length_meter2pix(srange):
                return 0
            else:
                return coef / log(d)
        return  _function

    def quadratic_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d == 0:
                return 1
            elif d > geomap.length_meter2pix(srange):
                return 0
            else:
                return coef / d**2
        return  _function

    """ This dictionnary lists available sensors function. """
    sensors = {\
            'linear': linear_sensor, \
            'square': square_sensor, \
            'log': log_sensor, \
            'quadratic': quadratic_sensor, \
            }

    try:
        function = sensors[name](coef)
    except KeyError:
        raise ValueError('invalid input')

    return function

