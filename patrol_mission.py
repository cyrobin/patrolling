#!/usr/bin/python
"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

# FIXME ensure that all geomaps are concistant ??
# (or deal with it ?!)

import json
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import time
import numpy as np
from sys import argv, exit
from pprint import pprint

from geomaps import *
from glpk_solver import *
from robot import Robot
from timer import Timer

VERBOSE=False
FSIZE = (12,12) # plot (figure size)
COLORS = ('green','cyan' ,'firebrick' ,'yellow' ,'blue' , \
        'purple','darkgoldenrod' ,'red')

class Mission:
    'Class describing and planning a mission'

    """ Init a mission upon a descriptive dictionary <mission>.
    This dictionary can be obtain through the load_mission function. """
    def __init__(self, _mission):

        self.mission    = _mission
        self.map_file   = self.mission[u'map']
        self.sampling   = self.mission[u'sampling']
        self.period     = self.mission[u'period']

        self.map        = Geomap(self.map_file)

        self.team = []
        for robot in self.mission[u'team']:
            self.team.append( Robot( robot ) )

        self.points = [] # the sampled points to be observed

        self.solver = GLPKSolver(self)

    """ Sample the observable positions (= the objective).
    Assume that the map is a distribution of probabilily,
    e.g the value are the utility. The sampled points are far apart, with a
    minimal distance between them. """
    def sample_objective(self):

        #FIXME magic number...
        self.points = sample_points( self.map, self.sampling, \
                min_dist =  self.map.length_meter2pix( 5 ) )

    """ Sample accessible positions each robot of the team """
    def sample_all_positions(self):
        for r in self.team:
            r.sample_positions( self.map, self.points )

    """ Solve / glpk """
    def solve(self):
        self.solver.solve_top()

    # TODO Update sensing

    """ Update the robots poses according to <p>, a vector of positions. If no
    <p> is provided, then use the current robot's plan and set the pose to the
    last position of this plan."""
    def update_poses(self, p = None):
        if p:
            for i,r in enumerate(self.team):
                r.update_pose(p[i])
        else:
            for r in self.team:
                r.update_pose()

    """ Display map, robots, sampled positions and current plans """
    def display_situation(self):
        # Beware of the axis-inversion (y,x) spotted empirically
        # when plotting points, axis, and so on
        global FSIZE
        global COLORS

        c = 0 ; # colors

        fig,ax = plt.subplots( figsize = FSIZE )
        imgplot = plt.imshow(self.map.image)
        imgplot.set_cmap('gray')
        plt.colorbar() #  Utility

        marks,labels = [],[]

        # sampled points
        if self.points:
            x,y = zip(*self.points)
            mark, = plt.plot(y, x, 'o', c=COLORS[c] )
            label = "Observable positions"

            marks.append(mark)
            labels.append(label)

        # Robots positions and paths
        for r in self.team:
            # Each robot has a specific color
            c += 1
            if len(COLORS) == c:
                c = 0

            # starting position
            mark, = plt.plot(r.pose[1],r.pose[0], '^', c=COLORS[c])
            label= "Starting position ({})".format(r.name)

            if r.points:
                # Accessible positions
                x,y = zip(*r.points)

                if not r.paths:
                    mark, = plt.plot(y, x, 'v', c=COLORS[c])
                    label = "Accessible positions ({})".format(r.name)

                    marks.append(mark)
                    labels.append(label)

                # Visibility (sensed areas)
                for xp,yp in zip(x,y):
                    sensor_rays = Ellipse((yp,xp),  \
                        width  = 2*self.map.length_meter2pix( r.srange ), \
                        height = 2*self.map.length_meter2pix( r.srange ), \
                        angle = 0, color=COLORS[c], alpha = 0.15)
                    ax.add_artist(sensor_rays)

                marks.append( sensor_rays )
                labels.append( "Sensing ({})".format(r.name) )

            # Path links are drawn as staight segments
            if r.paths:
                segments = []
                for (p,l) in r.paths.iteritems():
                    (x1,y1) = p
                    for (x2,y2) in l:
                        segments.extend([(y1,y2),(x1,x2),COLORS[c]])
                mark = plt.plot(*segments,linestyle='--')

                marks.append( mark[0] )
                labels.append( "Path links ({})".format(r.name) )

            # Plan -- links are drawn as staight segments
            if r.plan:
                x,y = zip(*r.plan)
                mark = plt.plot(y,x, color = COLORS[c], linewidth = 2.0)

                marks.append( mark[0] )
                labels.append( "Plan({})".format(r.name) )

        # Caption
        ax.legend(marks,labels,bbox_to_anchor=(-.1,0.9), loc=0 )
        #ax.legend(marks,labels, bbox_to_anchor=(0., 1.02, 1., .102), loc=3,\
           #ncol=2, mode="expand", borderaxespad=0.)
        plt.axis([0,self.map.width,self.map.height,0])

        plt.show()
        print "Display done."

    # TODO Dump map (as a distribution of probability)

    """ end """

def load_mission(mission_file):
    """ Load the mission file which has a json format.
    Return a python dictionnary equivalent to this json file.

    Here is an instance of expected json file:
    {
        "map": "./path_to_the_observation_map.png",
        "sampling": Nbr_of_sampling_points_(eg: 50),
        "period": max_cost_by_period_(eg: 99),
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
            "name":'linear'
            "range":20.0,
            "quality":3.5,
            "fov":6.28,
            "pose":{"x":0.1,"y":0.2,"z":0.7,"t":0.0}
        }
    }

    The returned mission dictionnary is the init entry for the Mission class,
    and has, for instance, the following format :

    {u'map': u'./obsvertion_map.png',
     u'sampling': 50,
     u'period': 99,
     u'team': [{u'description': u'./agv.json',
                u'name': u'robot_1',
                u'pos_map': u'./pos_map_agv.png',
                u'robot': {u'mass': 1.0, u'radius': 1.0, u'velocity': 1.0},
                u'sensor': {u'fov': 6.28,
                            u'pose': {u't': 0.0,
                                      u'x': 0.1,
                                      u'y': 0.2,
                                      u'z': 0.7},
                            u'quality': 3.5},
                            u'range': 20.0},
                            u'name': linear},
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
    #VERBOSE=True

    with Timer('Loading mission file'):
        mission = load_mission(argv[1])
        m =  Mission (  mission )
    with Timer('Sampling observable points'):
        m.sample_objective()
    with Timer('Sampling positions'):
        m.sample_all_positions()

    print "Solving..."
    with Timer('Solving'):
        m.solve()

    print "Displaying..."
    #for r in m.team:
        #r.display_wmap()
    #m.display_situation()

    print "Updating pose:"
    m.update_poses()

    print "Done."

