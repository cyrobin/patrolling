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

VERBOSE = False


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

        # TODO it seems this is not necessary
        #negative = np.vectorize( lambda x : 255 - x )
        #self.image = negative( self.image )

        if VERBOSE:
            print "Image is {}x{}".format(self.width,self.height)

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

    """ Return the distance beween two 2D points, on the map.
    Currently use the Euclidian distance. """
    def dist( self, (x1,y1), (x2,y2) ):
        return sqrt( ( (x1-x2) * self.scale_x )**2 \
                   + ( (y1-y2) * self.scale_y )**2 )

""" Sample <n> points in the <geomap>.
Consider the geomap has a discrete distribution of probability used for the
sampling."""
def sample_points( geomap, n ):
    points = []

    wrg = WeightedRandomGenerator(geomap.image)

    for i in range(n):
        idx = wrg()
        # Beware of the order (height,width) (set empirically...)
        points.append( np.unravel_index( idx, \
            (geomap.height, geomap.width ) ) )

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
            elif d > srange:
                return 0
            else:
                return coef / d
        return  _function

    def square_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d == 0:
                return 1
            elif d > srange:
                return 0
            else:
                return coef / sqrt(d)
        return  _function

    def log_sensor(coef):
        def _function(p,q):
            d = geomap.dist(p,q)
            if d < 1:
                return 1
            elif d > srange:
                return 0
            else:
                return coef / log(d)
        return  _function

    """ This dictionnary lists available sensors function. """
    sensors = {\
            'linear': linear_sensor, \
            'square': square_sensor, \
            'log': log_sensor, \
            }

    try:
        function = sensors[name](coef)
    except KeyError:
        raise ValueError('invalid input')

    return function

