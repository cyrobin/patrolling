"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

import json
import matplotlib.pyplot as plt
import numpy as np
from geomaps import *

from constant import *

class Robot:
    'Class embodying a given robot'

    """ Init a robot upon a descriptive dictionary <robot> """
    def __init__(self, robot):

        self.name        = robot[u'name']
        self.descrition  = robot[u'description']
        self.map_file    = robot[u'pos_map']

        self.mass        = robot[u'platform'][u'mass']
        self.radius      = robot[u'platform'][u'radius']
        self.velocity    = robot[u'platform'][u'velocity']

        self.sensor_name       = robot[u'sensor'][u'name']
        self.sensor_quality    = robot[u'sensor'][u'quality']
        self.sensor_range      = robot[u'sensor'][u'range']
        self.sensor_fov         = robot[u'sensor'][u'fov']
        self.sensor_pose = robot[u'sensor'][u'pose']

        self.pose         = (robot[u'start_pose'][u'x'], \
                            robot[u'start_pose'][u'y'], \
                            robot[u'start_pose'][u'z'], \
                            robot[u'start_pose'][u't'] )

        self.pos_map     = Geomap(self.map_file)

        # FIXME use a different geomap for the sensor
        self.sensor = built_sensor_function( self.pos_map, \
                                            self.sensor_name,
                                            self.sensor_quality,
                                            self.sensor_range)

        self.points     = []
        self.paths      = {}
        self.plan       = []
        self.old_plans  = []

    """ Sample the accessible positions (= the search space for potential
    plans).  To do so, optionnaly compute a intermediary <weighted_pos_map>
    weighted by the robot sensor, the utility map <u_map> and the sampled
    observable points given as arguments. The points are far apart with a
    minimal distance proportional to the robot's sensor range, and are
    accessible for at most a cost <max_cost>. """
    def sample_positions(self, u_map, points, max_cost, use_weight = False):

        # Compute the intermediary self.weighted_pos_map : it embodies the
        # utility of the position to be sampled while taking into account the
        # accessibility criteria given by self.pos_map
        if use_weight:
            self.weighted_pos_map = built_weighted_map( self.pos_map, \
                                                     self.sensor, \
                                                     u_map, \
                                                     points)
        else:
            self.weighted_pos_map = self.pos_map

        apply_scale = self.weighted_pos_map.length_meter2pix

        # Compute the accessible area to constraints the sampling to useful
        # positions only (e.g accessible for a cost inferior to <max_cost>)
        max_dist = apply_scale( max_cost / self.velocity )
        xmin = self.pose[0] - max_dist
        ymin = self.pose[1] - max_dist
        xmax = self.pose[0] + max_dist + 1
        ymax = self.pose[1] + max_dist + 1

        self.points = sampled_points( self.weighted_pos_map, N_SAMPLED_POS, \
                min_dist = apply_scale( 0.5*self.sensor_range ), \
                area = [(xmin,ymin),(xmax,ymax)] )

        # Add the current position (which is obviously valid !)
        self.points.append(self.pose[0:2])

        # Compute the path links between positions
        # <paths> is a dictionary using the positions in self.points as entry
        # and may be seen as a sparse matrix indicating the connections between
        # the accessible points.
        self.paths = computed_paths( self.pos_map, self.points, self.cost )

    """ Return the cost (time) for a robot to travel from p to q.
    Use the euclidian distance and the speed of the robot. """
    def cost(self,p,q):
        apply_scale = self.pos_map.length_pix2meter
        d = apply_scale( self.pos_map.dist(p,q) )
        return d / self.velocity

    """ Update the robot pos according to <p> : if no <p> is provided, then use
    the current robot's plan and set the pose to the last position of this
    plan. Either ways, the plan is tagged as 'old' and is wipped, just as the
    sampled positions."""
    def update_pose(self, p=None):
        if p:
            print "Yes !"
            self.pose = p
        elif self.plan:
            (x,y) = self.plan[-1] # plan is 2D
            (z,t) = self.pose[2:4] # plan is 2D
            self.pose = (x,y,z,t)
        # else do nothing

        # Deal with (no longer valid) past path and sampled positions
        self.old_plans.append(self.plan)
        self.plan   = []
        self.points = []
        self.paths  = []

        if VERBOSE:
            print "{} new pose is {}".format(self.name,self.pose)

    # TODO store / compute plans

    """ Display the wmap """
    def display_weighted_map(self):
        # Beware of the axis-inversion (y,x) spotted empirically
        # when plotting points, axis, and so on
        global FSIZE
        global COLORS

        color = 0 ; # colors

        fig,ax = plt.subplots( figsize = FSIZE )
        imgplot = plt.imshow(self.weighted_pos_map.image)
        imgplot.set_cmap('gray')
        plt.colorbar() #  Utility

        marks,labels = [],[]

        # sampled accessible positions
        if self.points:
            x,y   = zip(*self.points)
            mark, = plt.plot(y, x, 'o', c=COLORS[color] )
            label = "Accessible positions ({})".format(self.name)

            labels.append( label )
            marks.append(  mark  )

        # starting position
        color += 1
        mark, = plt.plot(self.pose[1],self.pose[0], '^', c=COLORS[color])
        label = "Starting position ({})".format(self.name)

        labels.append( label )
        marks.append(  mark  )

        # Caption
        ax.legend(marks,labels,bbox_to_anchor=(-.1,0.9), loc=0 )
        plt.axis([0,self.weighted_pos_map.width,self.weighted_pos_map.height,0])
        ax.xaxis.set_label_position('top')

        plt.show()

    """end"""

