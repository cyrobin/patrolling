"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import gdal
import numpy as np
from math import sqrt
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
            print('!ERROR! [Geomap] %s.aux.xml missing' % self.geofile)
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

        if VERBOSITY_LEVEL > 2:
            print "[Geomap] Image is {}x{}".format(self.width,self.height)

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
    sampling. One can set a minimal distance (in meters!) to respect between
    the sampled points, and constrained them to a given bounded area. There is
    a time out for the sampling, which may thus return an incomplete result."""
    def sampled_points( self, n, min_dist = 0, area = None ):
        points = []

        if area:
            [(xmin,ymin),(xmax,ymax)] = area

            # Bounded the area to the current map
            xmin = max( 0, xmin )
            ymin = max( 0, ymin )
            xmax = min(self.height + 1, xmax )
            ymax = min(self.width  + 1, ymax )

            (h,w) = self.image[xmin:xmax,ymin:ymax].shape

            wrg = WeightedRandomGenerator(self.image[xmin:xmax,ymin:ymax])
        else:
            wrg = WeightedRandomGenerator(self.image)

        """ Auxiliary function that check the distance of a given point <_p> to a
        list <_points> : if _p is not too close from others sampled points (>dist),
        then the function return True (ie one can keep <_p> as a valid sample> """
        def _not_too_close (_p, _points):
            for q in points:
                #if euclidian_distance(_p,q) < min_dist:
                if self.euclidian_distance_pix2meters(_p,q) < min_dist:
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
                    if VERBOSITY_LEVEL > 2:
                        print "!WARNING! [Geomap:sampled_points] Index value out of bounds."
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

        if VERBOSITY_LEVEL > 0 and \
                (time.time() - t_start) > SAMPLING_TIME_OUT :
            print "!WARNING! [Geomap:sampled_poinst] Sampling timed out ({} / {} points sampled)".format( \
                len(points), n )

        return points

    """ Return the euclidian distance beween two pixels. """
    def euclidian_distance_pixels( self, (x1,y1), (x2,y2) ):
        return sqrt( ( (x1-x2) )**2 \
                   + ( (y1-y2) )**2 )

    """ Return the euclidian distance in meters beween two 2D points given as
    pixels. """
    def euclidian_distance_pix2meters( self, (x1,y1), (x2,y2) ):
        return sqrt( ( (x1-x2) * self.scale_x )**2 \
                   + ( (y1-y2) * self.scale_y )**2 )

    """ Translate pixel coordinates into utm """
    def point_pix2utm(self, x, y):
        return [ x * self.scale_x + self.utm_x ,
                 y * self.scale_y + self.utm_y ]

    """ Translate pixel coordinates into utm with custom origin """
    def point_pix2custom(self, x, y):
        p = self.point_pix2utm(x, y)
        return [p[0] - self.custom_x_origin,
                p[1] - self.custom_y_origin]

    """ Transform one-liner index into image pixels coordinates """
    def point_idx2pix( self, idx ):
        return np.unravel_index( idx, (self.height,self.width) )

    """ Translate pixel length into meters length (x axis)"""
    def length_pix2meter_x(self, d):
        return d * abs(self.scale_x)

    """ Translate pixel length into meters length (y axis)"""
    def length_pix2meter_y(self, d):
        return d * abs(self.scale_y)

    """ Translate meter length into pixel length (x axis)"""
    def length_meter2pix_x(self, d):
        return d / abs(self.scale_x)

    """ Translate meter length into pixel length (y axis)"""
    def length_meter2pix_y(self, d):
        return d / abs(self.scale_y)

