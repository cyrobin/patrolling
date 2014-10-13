"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import json
import matplotlib.pyplot as plt
import numpy as np
from geomaps import *

VERBOSE=False
FSIZE = (12,12) # plot (figure size)
COLORS = ('green','cyan' ,'firebrick' ,'yellow' ,'blue' , \
        'purple','darkgoldenrod' ,'red')

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

        self.sname       = robot[u'sensor'][u'name']
        self.squality    = robot[u'sensor'][u'quality']
        self.srange      = robot[u'sensor'][u'range']
        self.fov         = robot[u'sensor'][u'fov']
        self.sensor_pose = robot[u'sensor'][u'pose']

        self.pose         = (robot[u'start_pose'][u'x'], \
                            robot[u'start_pose'][u'y'], \
                            robot[u'start_pose'][u'z'], \
                            robot[u'start_pose'][u't'] )

        self.pos_map     = Geomap(self.map_file)

        # FIXME use a different geomap for the sensor
        self.sensor = make_sensor_function( self.pos_map, \
                                            self.sname,
                                            self.squality,
                                            self.srange)

        self.points = []
        self.paths  = {}
        self.plan   = []

    """ Sample the accessible positions (= the potential plans).
    To do so, optionnaly compute a intermediary <wpos_map> weighted by the
    robot sensor, the utility map <u_map> and the sampled observable points
    given as arguments. The points are far apart with a minimal distance
    proportional to the robot's sensor range"""
    def sample_positions(self, u_map, points, weight = False):

        # Compute the intermediary self.wpos_map : it embodies the utility of
        # the position to be sampled while taking into account the
        # accessibility criteria given by self.pos_map
        if weight:
            self.wpos_map = make_weighted_map( self.pos_map, \
                                               self.sensor, \
                                               u_map, \
                                               points)
        else:
            self.wpos_map = self.pos_map

        # FIXME fix magic number
        self.points = sample_points( self.wpos_map, 5, \
                min_dist = self.wpos_map.length_meter2pix( 0.5*self.srange ) )

        # Add the current position (which is obviously valid !)
        self.points.append(self.pose[0:2])

        # Compute the path links between positions
        # <paths> is a dictionary using the positions in self.points as entry
        # and may be seen as a sparse matrix indicating the connections between
        # the accessible points.
        self.paths = compute_paths( self.pos_map, self.points, self.cost )

        #dummy position, for test purpose
        self.points.append((0,0))

    """ Return the cost (time) for a robot to travel from p to q.
    Use the euclidian distance and the speed of the robot. """
    def cost(self,p,q):
        d = self.pos_map.length_pix2meter( self.pos_map.dist(p,q) )
        return d / self.velocity

    """ Update the robot pos according to <p> : if no <p> is provided, then use
    the current robot's plan and set the pose to the last position of this
    plan."""
    def update_pose(self, p=None):
        print "{}: {}".format(self.name,self.pose)
        if p:
            print "Yes !"
            self.pose = p
        else:
            (x,y) = self.plan[-1] # plan is 2D
            (z,t) = self.pose[2:4] # plan is 2D
            self.pose = (x,y,z,t)

        print "{}: {}".format(self.name,self.pose)

    # TODO store / compute plans

    """ Display the wmap """
    def display_wmap(self):
        # Beware of the axis-inversion (y,x) spotted empirically
        # when plotting points, axis, and so on
        global FSIZE
        global COLORS

        c = 0 ; # colors

        fig,ax = plt.subplots( figsize = FSIZE )
        imgplot = plt.imshow(self.wpos_map.image)
        imgplot.set_cmap('gray')
        plt.colorbar() #  Utility

        marks,labels = [],[]

        # sampled accessible positions
        if self.points:
            x,y = zip(*self.points)
            mark, = plt.plot(y, x, 'o', c=COLORS[c] )
            label = "Accessible positions ({})".format(self.name)

            marks.append(mark)
            labels.append(label)

        # starting position
        c += 1
        mark, = plt.plot(self.pose[1],self.pose[0], '^', c=COLORS[c])
        label= "Starting position ({})".format(self.name)

        marks.append(mark)
        labels.append(label)

        # Caption
        ax.legend(marks,labels,bbox_to_anchor=(-.1,0.9), loc=0 )
        plt.axis([0,self.wpos_map.width,self.wpos_map.height,0])

        plt.show()

    """end"""

