"""
Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif
"""

# FIXME ensure that all geomaps are concistant ??
# (or deal with it ?!)

import json
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
from pprint import pprint

from geomaps import *
from utility_map import UtilityMap
from glpk_solver import *
from robot import Robot

from constant import *
from timer import Timer

class Mission:
    'Class describing and planning a mission'

    """ Init a mission upon a descriptive dictionary <mission>.
    This dictionary can be obtain through the load_mission function. """
    def __init__(self, _mission):

        self.mission     = _mission
        self.name        = self.mission[u'name']
        self.time_stamps = time.strftime("%Y-%m-%d_%H:%M")
        self.sampling    = N_SAMPLED_OBS
        self.period      = self.mission[u'period']
        self.utility_map_file = self.mission[u'map']

        self.utility_map = UtilityMap( self.utility_map_file )

        self.team = []
        for robot in self.mission[u'team']:
            self.team.append( Robot( robot ) )

        for robot in self.team:
            self.utility_map.check_coherence( robot.accessibility_map )
            self.utility_map.check_coherence( robot.visibility_map )

        self.points = [] # the sampled points to be observed

        self.solver = GLPKSolver()

        self.loop_step = 0

        self.fig,self.ax = plt.subplots( figsize = FSIZE )

    """ Sample the observable positions (= the objective).
    Assume that the map is a distribution of probabilily,
    e.g the value are the utility. The sampled points are far apart, with a
    minimal distance (in meters) between them. """
    def sample_objective(self):

        self.points = self.utility_map.sampled_points( self.sampling, \
                min_dist = MIN_SAMPLING_DIST )

    """ Sample accessible positions each robot of the team """
    def sample_all_positions(self):
        for robot in self.team:
            robot.sample_positions( self.utility_map, self.points, self.period )

    """ Solve (using glpk) """
    def solve(self, milp_formulation = 'Perception-based TSP'):
        available_milp_formulation = {
            'Perception-based TSP': self.solver.solve_perception_tsp,
            'Position-based TSP'  : self.solver.solve_position_tsp,
            'Perception-based TOP': self.solver.solve_ptop,
        }

        try:
            solver = available_milp_formulation[ milp_formulation ]
        except KeyError:
            print "!ERROR! '{}' is not a known MILP formulation. Problem remains unsolved.".format(milp_formulation)
        else:
            solver(self.team, self.utility_map, self.points, self.period)

    """ Perform one whole planning loop, <n> times. """
    def loop(self, n, DISPLAY = False, milp_formulation='Perception-based TSP'):

        for i in range(n):
            self.loop_once(DISPLAY, milp_formulation)

    """ Perform one whole planning loop. """
    def loop_once(self, DISPLAY=False, milp_formulation='Perception-based TSP'):

        self.loop_step += 1
        print "Patrolling -- loop #{}".format(self.loop_step)

        with Timer('Updating poses and utility map'):
            self.update()

        with Timer('Sampling observable points'):
            self.sample_objective()
        with Timer('Sampling positions'):
            self.sample_all_positions()

        with Timer('Solving'):
            self.solve(milp_formulation)

        if DISPLAY:
            if VERBOSE:
                for robot in self.team:
                    robot.display_weighted_map()
            self.display_situation()

        self.dump_situation()

    """ Update the mission according to current plan (utility map and robots's
    poses) """
    def update(self):
        self.update_map()
        self.update_poses()

    """ Update the robots poses according to <p>, a vector of positions. If no
    <p> is provided, then use the current robot's plan and set the pose to the
    last position of this plan."""
    def update_poses(self, p = None):
        if p:
            for i,robot in enumerate(self.team):
                robot.update_pose(p[i])
        else:
            for robot in self.team:
                robot.update_pose()

    """ Update the utility map according to robots'plans and natural growth due
    to idleness (= absence of observation for a while)"""
    def update_map(self):
        self.utility_map.update_utility( self.team )

    """ Display various metrics """
    # TODO TO BE COMPLETED (data about robots ?!)
    def print_metrics(self):
        self.utility_map.print_metrics()

    """ Dump map, robots, sampled positions and current plans """
    def dump_situation(self):
        self.display_situation(True)

    """ Display map, robots, sampled positions and current plans """
    def display_situation(self, DUMP = False):
        # Beware of the axis-inversion (y,x) spotted empirically
        # when plotting points, axis, and so on
        global FSIZE
        global COLORS

        color = 0 ;

        imgplot = plt.imshow(self.utility_map.image)
        imgplot.set_cmap('gray')
        plt.colorbar() #  Utility

        marks,labels = [],[]

        # sampled points
        if self.points:
            x,y   = zip(*self.points)
            mark, = plt.plot(y, x, 'o', c=COLORS[color] )
            label = "Observable positions"

            labels.append( label )
            marks.append( mark )

        # Robots positions and paths
        for robot in self.team:

            # Each robot has a specific color
            color += 1
            if len(COLORS) == color:
                color = 0

            # starting position
            mark, = plt.plot(robot.pose[1],robot.pose[0], '^', c=COLORS[color])
            label = "Starting position ({})".format(robot.name)

            labels.append( label )
            marks.append( mark )

            if robot.points:
                # Accessible positions
                x,y = zip(*robot.points)

                if not robot.paths:
                    mark, = plt.plot(y, x, 'v', c=COLORS[color])
                    label = "Accessible positions ({})".format(robot.name)

                    labels.append( label )
                    marks.append(  mark )

                # Visibility (sensed areas)
                sensor_x_range = self.utility_map.length_meter2pix_x( robot.sensor_range )
                sensor_y_range = self.utility_map.length_meter2pix_y( robot.sensor_range )

                for xp,yp in zip(x,y):
                    sensor_rays = Ellipse( (yp,xp),  \
                        width  = 2 * sensor_x_range, \
                        height = 2 * sensor_y_range, \
                        angle  = 0, \
                        color  = COLORS[color], \
                        alpha  = 0.15 )
                    self.ax.add_artist( sensor_rays )

                labels.append( "Sensing ({})".format(robot.name) )
                marks.append(  sensor_rays )

            # Path links are drawn as staight segments
            if robot.paths:
                segments = []
                for (p,l) in robot.paths.iteritems():
                    (x1,y1) = p
                    for (x2,y2) in l:
                        segments.extend([(y1,y2),(x1,x2),COLORS[color]])
                mark = plt.plot(*segments,linestyle='--')

                labels.append( "Path links ({})".format(robot.name) )
                marks.append(  mark[0] )

            # Plan -- links are drawn as staight segments
            if robot.plan:
                x,y = zip(*robot.plan)
                mark = plt.plot(y,x, color = COLORS[color], linewidth = 2.0)

                labels.append( "Plan({})".format(robot.name) )
                marks.append(  mark[0] )

            # Past Plans -- links are drawn as staight segments
            elif robot.old_plans:
                for plan in robot.old_plans:
                    if plan:
                        x,y = zip(*plan)
                        mark = plt.plot(y,x, color = COLORS[color], linewidth = 1.0)

                if mark:
                    labels.append( "Old Plan({})".format(robot.name) )
                    marks.append(  mark[0] )

        # Caption
        self.ax.legend(marks,labels,bbox_to_anchor=(-.1,0.9), loc=0 )
        #ax.legend(marks,labels, bbox_to_anchor=(0., 1.02, 1., .102), loc=3,\
           #ncol=2, mode="expand", borderaxespad=0.)
        plt.axis([0,self.utility_map.width,self.utility_map.height,0])
        self.ax.xaxis.set_label_position('top')

        if DUMP:
            figname = "{}_{}-{}.svg".format(self.name, self.time_stamps, self.loop_step)
            plt.savefig(figname,bbox_inches='tight')
            print "Figure save as {}".format(figname)
            plt.clf()
            plt.cla()
        else:
            plt.show()
            print "Display done."

    # TODO Dump map (as a distribution of probability)

    """ end """

def loaded_mission(mission_file):
    """ Load the mission file which has a json format.
    Return a python dictionnary equivalent to this json file.

    Here is an instance of expected json file:
    {
        "map": "./path_to_the_observation_map.png",
        "period": max_cost_by_period_(eg: 99),
        "team":
            [ {
                "name": "robot_1",
                "description": "./agv.json",
                "accessibility_map": "./accessibility_map_agv.png",
                "start_pose":{"x":0.1,"y":0.2,"z":0.7,"t":0.0}
                },
              {
                "name": "robot_2",
                "description": "./agv.json",
                "accessibility_map": "./accessibility_map_agv.png",
                "start_pose":{"x":1.1,"y":1.2,"z":0.7,"t":0.0}
                },
              {
                "name": "robot_3",
                "description": "./aav.json",
                "accessibility_map": "./accessibility_map_aav.png",
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
     u'period': 99,
     u'team': [{u'description': u'./agv.json',
                u'name': u'robot_1',
                u'accessibility_map': u'./accessibility_map_agv.png',
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
                u'accessibility_map': u'./accessibility_map_agv.png',
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
                u'accessibility_map': u'./accessibility_map_aav.png',
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
    for robot_data in mission['team']:
        with open(robot_data['description']) as json_robot:
            robot_data.update( json.load( json_robot ) )
            json_robot.close()

    if VERBOSE:
        pprint(mission)

    return mission

