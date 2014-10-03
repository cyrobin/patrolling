#!/usr/bin/python
#
#   Cyril Robin -- LAAS-CNRS -- 2014
#
# TODO Descriptif
#

import json
import gdal
import matplotlib.pyplot as plt
import random
import bisect
import operator as operator
import time
import numpy as np
from sys import argv, exit
from math import sqrt
from pprint import pprint

VERBOSE=False
FSIZE = (15,15) # plot (figure size)

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
    def point_pix2utm(x, y):
        return [ x * self.scale_x + self.utm_x ,
                 y * self.scale_y + self.utm_y ]

    """ Translate pixel coordinates into utm with custom origin """
    def point_pix2custom(x, y):
        p = point_pix2utm(x, y)
        return [p[0] - self.custom_x_origin,
                p[1] - self.custom_y_origin]

    """ Transform pixels coordinates into one-liner index """
    #def point_pix2idx(x, y):
        #return y*self.width + x
        #TODO use flat / ravel instead ? zith coords ?
        #return image.ravel(

    """ Transform one-liner index into image pixels coordinates """
    def point_idx2pix( idx ):
        return np.unravel_index( idx, (self.height,self.width) )

class Robot:
    'Class embodying a given robot'

    """ Init a robot upon a descriptive dictionary <robot> """
    def __init__(self, robot):

        self.name        = robot[u'name']
        self.desc        = robot[u'description']
        self.map_file    = robot[u'pos_map']
        self.mass        = robot[u'platform'][u'mass']
        self.radius      = robot[u'platform'][u'radius']
        self.velocity    = robot[u'platform'][u'velocity']
        self.range       = robot[u'sensor'][u'range']
        self.fov         = robot[u'sensor'][u'fov']
        self.sensor_pose = robot[u'sensor'][u'pose']

        self.pos         = (robot[u'start_pose'][u'x'], \
                            robot[u'start_pose'][u'y'], \
                            robot[u'start_pose'][u'z'], \
                            robot[u'start_pose'][u't'] )

        self.pos_map     = Geomap(self.map_file)

        self.points = []

    """ Sample the accessible positions (= the potential plans)."""
    def sample_positions(self):
        self.points = []

        # TODO have a better mapping approach ? (grids ?)
        # what size for the gridcell ? (range /2 ? other ? )
        wrg = WeightedRandomGenerator(self.pos_map.image)

        # TODO fix magic number
        for i in range(10):
            idx = wrg()
            # Beware of the order (height,width) (set empirically...)
            self.points.append( np.unravel_index( idx, \
                (self.pos_map.height, self.pos_map.width ) ) )

    # TODO update pose

    # TODO store / compute plans

    # TODO display map (and pos/points)?

    """end"""

class Mission:
    'Class describing and planning a mission'

    """ Init a mission upon a descriptive dictionary <mission>.
    This dictionary can be obtain through the load_mission function. """
    def __init__(self, _mission):

        self.mission    = _mission
        self.map_file   = self.mission[u'map']
        self.sampling   = self.mission[u'sampling']

        self.map        = Geomap(self.map_file)

        self.team = []
        for robot in self.mission[u'team']:
            self.team.append( Robot( robot ) )

        self.points = [] # the sampled points to be observed

    """ Sample the observable positions (= the objective).
    Assume that the map is a distribution of probabilily,
    e.g the value are the utility. """
    def sample_objective(self):
        self.points = []

        wrg = WeightedRandomGenerator(self.map.image)

        for i in range(self.sampling):
            idx = wrg()
            # Beware of the order (height,width) (set empirically...)
            self.points.append( np.unravel_index( idx, (self.map.height, self.map.width ) ) )

    # Sample accessible positions each robot of the team
    def sample_all_positions(self):
        for r in self.team:
            r.sample_positions()

    # TODO Solve / glpk

    # TODO Update sensing

    # TODO Update robot's poses

    """ Display map, robots and sampled positions """
    def display_situation(self):
        global FSIZE
        plt.subplots(figsize = FSIZE)
        imgplot = plt.imshow(self.map.image)
        imgplot.set_cmap('gray') # color

        # sampled points
        if self.points:
            # Beware of the order (y,x) (set empirically...)
            y,x = zip(*self.points)
            plt.plot(x, y, 'o', c='green')

        # TODO Robots positions

        # TODO Robots accessible positions

        # TODO Visibility links

        # TODO Caption

        plt.show()
        print "Display done"

    # TODO Dump map (as a distribution of probability)

    """ end """

def load_mission(mission_file):
    """ Load the mission file which has a json format.
    Return a python dictionnary equivalent to this json file.

    Here is an instance of expected json file:
    {
        "map": "./path_to_the_observation_map.png",
        "sampling": Nbr_of_sampling_points_(eg: 50),
        "team":
            [ {
                "name": "robot_1",
                "description": "./agv.json",
                "pos_map": "./pos_map_agv.png",
                "start_pose":{"x":0.1,"y":0.2,"z":0.7,"t":0.0}
                },
              {
                "name": "robot_2",
                "description": "./agv.json",
                "pos_map": "./pos_map_agv.png",
                "start_pose":{"x":1.1,"y":1.2,"z":0.7,"t":0.0}
                },
              {
                "name": "robot_3",
                "description": "./aav.json",
                "pos_map": "./pos_map_aav.png",
                "start_pose":{"x":0.7,"y":0.7,"z":10.7,"t":0.0}
                }
        ]
    }

    Robot descrition files look like this:
    {
        "platform":{
            "mass":1.0,
            "radius":1.0,
            "velocity":1.0
        },
        "sensor":{
            "range":20.0,
            "fov":6.28,
            "pose":{"x":0.1,"y":0.2,"z":0.7,"t":0.0}
        }
    }

    The returned mission dictionnary is the init entry for the Mission class,
    and has, for instance, the following format :

    {u'map': u'./obsvertion_map.png',
     u'sampling': 50,
     u'team': [{u'description': u'./agv.json',
                u'name': u'robot_1',
                u'pos_map': u'./pos_map_agv.png',
                u'robot': {u'mass': 1.0, u'radius': 1.0, u'velocity': 1.0},
                u'sensor': {u'fov': 6.28,
                            u'pose': {u't': 0.0,
                                      u'x': 0.1,
                                      u'y': 0.2,
                                      u'z': 0.7},
                            u'range': 20.0},
                u'start_pose': {u't': 0.0, u'x': 0.1, u'y': 0.2, u'z': 0.7}},
               {u'description': u'./agv.json',
                u'name': u'robot_2',
                u'pos_map': u'./pos_map_agv.png',
                u'robot': {u'mass': 1.0, u'radius': 1.0, u'velocity': 1.0},
                u'sensor': {u'fov': 6.28,
                            u'pose': {u't': 0.0,
                                      u'x': 0.1,
                                      u'y': 0.2,
                                      u'z': 0.7},
                            u'range': 20.0},
                u'start_pose': {u't': 0.0, u'x': 1.1, u'y': 1.2, u'z': 0.7}},
               {u'description': u'./aav.json',
                u'name': u'robot_3',
                u'pos_map': u'./pos_map_aav.png',
                u'robot': {u'mass': 1.0, u'radius': 1.0, u'velocity': 1.0},
                u'sensor': {u'fov': 6.28,
                            u'pose': {u't': 0.0,
                                      u'x': 0.1,
                                      u'y': 0.2,
                                      u'z': 0.7},
                            u'range': 20.0},
                u'start_pose': {u't': 0.0, u'x': 0.7, u'y': 0.7, u'z': 10.7}}]}


    """
    global VERBOSE

    with open(mission_file) as json_data:
        mission = json.load(json_data)
        json_data.close()

    # 'team' is a list of dictionnaries
    for robot in mission['team']:
        with open(robot['description']) as robot_json:
            robot.update( json.load( robot_json ) )
            robot_json.close()

    if VERBOSE:
        pprint(mission)

    return mission



if __name__ == "__main__":

    VERBOSE=True

    mission = load_mission(argv[1])
    m =  Mission (  mission )
    m.sample_objective()
    m.display_situation()

    print "Done."

