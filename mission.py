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
from copy import deepcopy

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
        self.time_stamps = time.strftime("%Y-%m-%d_%Hh%Mm%S")
        self.logfile     = "{}_{}.log".format(self.name, self.time_stamps)

        self.sampling    = N_SAMPLED_OBS
        self.period      = self.mission[u'period']
        self.utility_map_file = self.mission[u'map']

        self.utility_map = UtilityMap( self.utility_map_file, self.period )

        self.team = []
        for robot in self.mission[u'team']:
            self.team.append( Robot( robot ) )

        for robot in self.team:
            self.utility_map.check_coherence( robot.accessibility_map )
            self.utility_map.check_coherence( robot.visibility_map )

        self.points = [] # the sampled points to be observed

        self.solver = GLPKSolver()

        self.count_periods = 0

        self.fig,self.ax = plt.subplots( figsize = FSIZE )

        if VERBOSITY_LEVEL > 0:
            with open(self.logfile,"a") as log:
                log.write( "log: {}\n".format(self.logfile) )
                pprint( self.mission, log )
                log.write( "Sampling observation: {}\n".format(self.sampling) )

            if VERBOSITY_LEVEL > 1:
                print "[Mission] logfile is {}".format(self.logfile)

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

    """ Update the mission according to current plan (utility map and robots's
    poses) """
    def update(self):
        if self.count_periods > 0:
            if VERBOSITY_LEVEL > 2:
                print "[Mission] Updating..."

            if VERBOSITY_LEVEL > 0:
                with open(self.logfile,"a") as log:
                    log.write("Current situation - period #{}\n".format( self.count_periods ) )
                    for robot in self.team:
                        log.write( "{}: {}, with plan :\n".format(robot.name,robot.pose) )
                        log.write( "{}\n".format(robot.plan ) )

            self.update_map()
            self.update_poses()

        self.count_periods += 1

        if VERBOSITY_LEVEL > 1:
            print "[Mission] Patrolling -- Period #{} begins.".format(self.count_periods)

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

    """ Update the utility map according to robots' plans and natural growth due
    to idleness (= absence of observation for a while)"""
    def update_map(self):
        self.utility_map.update_utility( self.team )

    """ Solve (using glpk) """
    def solve(self, milp_formulation = 'Perception-based TOP'):
        available_milp_formulation = {
            'Perception-based TOP': self.solver.solve_perception_top,
            'Position-based TOP'  : self.solver.solve_position_top,
            'SP': self.solver.solve_sp,
        }

        try:
            solver = available_milp_formulation[ milp_formulation ]
        except KeyError:
            print "!ERROR! [Mission:solve] '{}' is not a known MILP formulation. Problem remains unsolved.".format(milp_formulation)
        else:
            solver(self.team, self.utility_map, self.points, self.period)

    """ Perform one whole planning loop, <n> times. """
    def loop(self, n, DISPLAY = False, milp_formulation='Perception-based TOP'):
        for i in range(n):
            self.loop_once(DISPLAY, milp_formulation)

        self.update()
        self.dump_situation()

        if VERBOSITY_LEVEL > 0:
            with open(self.logfile,"a") as log:
                log.write("Global plan after {} periods:\n".format( self.count_periods-1 ) )
                for robot in self.team:
                    log.write( "{}: {}, with global plan :\n".format(robot.name,robot.pose) )
                    log.write( "{}\n".format( robot.old_plans ) )

            self.print_metrics(self.logfile)
            self.utility_map.save()

    """ Perform one whole planning loop. """
    def loop_once(self, DISPLAY=False, milp_formulation='Perception-based TOP'):
        with Timer('Updating poses and utility map'):
            self.update()

        with Timer('Sampling observable points'):
            self.sample_objective()
        with Timer('Sampling positions'):
            self.sample_all_positions()

        with Timer('Solving'):
            self.solve(milp_formulation)

        if DISPLAY:
            if VERBOSITY_LEVEL > 2:
                for robot in self.team:
                    robot.display_weighted_map()
            self.display_situation()

        if VERBOSITY_LEVEL > 0:
            self.dump_situation()

    """ Decentrally solve (using glpk) """
    def decentrally_solve(self, milp_formulation = 'Perception-based TOP',
             available_comlinks = None ):
        available_milp_formulation = {
            'Perception-based TOP': self.solver.solve_perception_top,
            'Position-based TOP'  : self.solver.solve_position_top,
            'SP': self.solver.solve_sp,
        }
        try:
            solver = available_milp_formulation[ milp_formulation ]
        except KeyError:
            print "!ERROR! [Mission:solve] '{}' is not a known MILP formulation. Problem remains unsolved.".format(milp_formulation)
        else:
            solver(self.virtual_team, self.virtual_utility_map, \
                    self.points, self.period, available_comlinks)

    """ Perform one whole decentralized planning loop, <n> times. """
    def decentralized_loop(self, n, DISPLAY = False, \
            milp_formulation='Perception-based TOP'):

        for i in range(n):
            self.decentralized_loop_once(DISPLAY, milp_formulation)

        self.update()
        self.dump_situation()

        if VERBOSITY_LEVEL > 0:
            with open(self.logfile,"a") as log:
                log.write("Global plan after {} periods:\n".format( self.count_periods-1 ) )
                for robot in self.team:
                    log.write( "{}: {}, with global plan :\n".format(robot.name,robot.pose) )
                    log.write( "{}\n".format( robot.old_plans ) )

            self.print_metrics(self.logfile)
            self.utility_map.save()

    """ Perform one whole planning loop in a decentralized manner. """
    def decentralized_loop_once(self, DISPLAY = False, \
            milp_formulation = 'Perception-based TOP'):

        with Timer('Updating poses and utility map'):
            self.update()

        # copy map into virtual map which is use to simulate the impact of
        # other robots while planning for a specific one
        self.virtual_utility_map = deepcopy(self.utility_map)

        # Plan for one robot at a time, considering the impact of the robots
        # that have already planned something for this period
        # TODO change the planning order to a more smart way
        with Timer('Planning the whole decentralized loop'):
            self.virtual_team = []
            available_comlinks= []
            for robot in self.team:
                with Timer("Planning for {}".format(robot.name)):
                    # update the virtual map (ie compute what map would be
                    # according to the plan of the precedent virtual team)
                    self.virtual_utility_map.update_utility( self.virtual_team )

                    if VERBOSITY_LEVEL > 2:
                        print "[Planning] Virtual map updated. Sampling and Solving..."

                    # Update virtual team
                    self.virtual_team = [robot]

                    # sample observable position according to the virtual map
                    self.points = self.virtual_utility_map.sampled_points( \
                        self.sampling, min_dist = MIN_SAMPLING_DIST )

                    # sample position for <robot>
                    robot.sample_positions( self.virtual_utility_map, self.points, \
                            self.period )

                    # find a suitable plan for <robot>
                    self.decentrally_solve(milp_formulation, available_comlinks)

                    if robot.plan:
                        available_comlinks.append( (robot, robot.plan[-1]) )

        # gather plans
        if DISPLAY:
            if VERBOSITY_LEVEL > 2:
                for robot in self.team:
                    robot.display_weighted_map()
            self.display_situation()

        if VERBOSITY_LEVEL > 0:
            self.dump_situation()

    """ Perform one whole decentralized parallely planning loop, <n> times. """
    def parallel_loop(self, n, DISPLAY = False, \
            milp_formulation='Perception-based TOP'):

        for i in range(n):
            self.parallel_loop_once(DISPLAY, milp_formulation)

        self.update()
        self.dump_situation()

        if VERBOSITY_LEVEL > 0:
            with open(self.logfile,"a") as log:
                log.write("Global plan after {} periods:\n".format( self.count_periods-1 ) )
                for robot in self.team:
                    log.write( "{}: {}, with global plan :\n".format(robot.name,robot.pose) )
                    log.write( "{}\n".format( robot.old_plans ) )

            self.print_metrics(self.logfile)
            self.utility_map.save()

    """ Perform one whole parallelly planning loop in a decentralized manner. """
    def parallel_loop_once(self, DISPLAY = False, \
            milp_formulation = 'Perception-based TOP'):

        with Timer('Updating poses and utility map'):
            self.update()

        # Plan parallel for all robots at a time, ignoring the others teammates.
        with Timer('Planning the whole decentralized parallel loop'):
            #TODO multi-thread this ! See http://www.tutorialspoint.com/python/python_multithreading.htm for instance
            for robot in self.team:

                # copy map into virtual map which is use to simulate the impact
                # of other robots while planning for a specific one
                self.virtual_utility_map = deepcopy(self.utility_map)
                available_comlinks = []
                self.virtual_team = []

                with Timer("Planning for {}".format(robot.name)):
                    # update the virtual map (ie compute what map would be
                    # according to the plan of the precedent virtual team)
                    self.virtual_utility_map.update_utility( self.virtual_team )

                    # Update virtual team
                    self.virtual_team = [robot]

                    # sample observable position according to the virtual map
                    self.points = self.virtual_utility_map.sampled_points( \
                        self.sampling, min_dist = MIN_SAMPLING_DIST )

                    # sample position for <robot>
                    robot.sample_positions( self.virtual_utility_map, \
                        self.points, self.period )

                    # find a suitable plan for <robot>
                    self.decentrally_solve(milp_formulation, available_comlinks)

        # TODO optimize the plan a posteriori ? (to avoid conflict or limit redundancy ?)

        # gather plans
        if DISPLAY:
            if VERBOSITY_LEVEL > 2:
                for robot in self.team:
                    robot.display_weighted_map()
            self.display_situation()

        if VERBOSITY_LEVEL > 0:
            self.dump_situation()

    """ Display various metrics """
    def print_metrics(self,logfile = None):
        self.utility_map.print_metrics(logfile)

    """ Dump map, robots, sampled positions and current plans """
    def dump_situation(self):
        self.display_situation(True)

    """ Display map, robots, sampled positions and current plans """
    def display_situation(self, DUMP = False):
        # Beware of the axis-inversion (y,x) spotted empirically
        # when plotting points, axis, and so on
        imgplot = plt.imshow(self.utility_map.image)
        imgplot.set_cmap('gray')
        plt.colorbar() #  Utility

        marks,labels = [],[]

        # sampled points
        if self.points:
            x,y   = zip(*self.points)
            mark, = plt.plot( y, x, 'o', c=OBS_COLOR )
            label = "Observable positions"

            labels.append( label )
            marks.append( mark )

        color = 0 ;

        # Robots positions and paths
        for robot in self.team:

            # Each robot has a specific color
            if len(ROBOT_COLORS) == color:
                color = 0

            # starting position
            mark, = plt.plot(robot.pose[1],robot.pose[0], '^', c=ROBOT_COLORS[color])
            label = "Starting position ({})".format(robot.name)

            labels.append( label )
            marks.append( mark )

            if robot.points:
                # Accessible positions
                x,y = zip(*robot.points)

                if not robot.paths:
                    mark, = plt.plot(y, x, 'v', c=ROBOT_COLORS[color])
                    label = "Accessible positions ({})".format(robot.name)

                    labels.append( label )
                    marks.append(  mark )

                # Visibility (sensed areas)
                if VERBOSITY_LEVEL > 2:
                    sensor_x_range = self.utility_map.length_meter2pix_x( robot.sensor_range )
                    sensor_y_range = self.utility_map.length_meter2pix_y( robot.sensor_range )

                    for xp,yp in zip(x,y):
                        sensor_rays = Ellipse( (yp,xp),  \
                            width  = 2 * sensor_x_range, \
                            height = 2 * sensor_y_range, \
                            angle  = 0, \
                            color  = ROBOT_COLORS[color], \
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
                        segments.extend([(y1,y2),(x1,x2),ROBOT_COLORS[color]])
                mark = plt.plot(*segments,linestyle='--')

                labels.append( "Path links ({})".format(robot.name) )
                marks.append(  mark[0] )

            # Plan -- links are drawn as staight segments
            if robot.plan:
                x,y = zip(*robot.plan)
                mark = plt.plot(y,x, color = ROBOT_COLORS[color], linewidth = 2.0)

                labels.append( "Plan({})".format(robot.name) )
                marks.append(  mark[0] )

            # Past Plans -- links are drawn as staight segments
            elif robot.old_plans:
                for plan in robot.old_plans:
                    if plan:
                        x,y = zip(*plan)
                        mark = plt.plot(y,x, color = ROBOT_COLORS[color], linewidth = 1.0)

                if mark:
                    labels.append( "Old Plan({})".format(robot.name) )
                    marks.append(  mark[0] )

            color += 1

        # Display comlinks
        mark = None

        if DISPLAY_COM_LINKS:
            for robot in self.team:
                for partner in self.team:
                    if robot == partner:
                        continue

                    p = robot.pose[0:2]
                    q = partner.pose[0:2]
                    if robot.comlink(p,q)*partner.comlink(q,p):
                        x,y = zip(*[p,q])
                        mark = plt.plot(y,x, color = COM_COLOR, linestyle=":", linewidth = 3.0)

        if mark:
            labels.append( "Com links" )
            marks.append(  mark[0] )

        # Caption
        self.ax.legend(marks,labels,bbox_to_anchor=(-.1,0.9), loc=0 )
        #ax.legend(marks,labels, bbox_to_anchor=(0., 1.02, 1., .102), loc=3,\
           #ncol=2, mode="expand", borderaxespad=0.)
        plt.axis([0,self.utility_map.width,self.utility_map.height,0])
        self.ax.xaxis.set_label_position('top')

        if DUMP:
            #figname = "{}_{}-{:03d}.svg".format(self.name, self.time_stamps, self.count_periods)
            figname = "{}_{}-{:03d}.png".format(self.name, self.time_stamps, self.count_periods)
            plt.savefig(figname,bbox_inches='tight')
            if VERBOSITY_LEVEL > 1:
                print "[Mission] Figure save as {}".format(figname)
            if VERBOSITY_LEVEL > 0:
                with open(self.logfile,"a") as log:
                    log.write( "[Mission] Figure save as {}\n".format(figname) )
            plt.clf()
            plt.cla()
        else:
            plt.show()
            if VERBOSITY_LEVEL > 1:
                print "Display done."

    # TODO Dump geomap (as a distribution of probability)

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
    with open(mission_file) as json_data:
        mission = json.load(json_data)
        json_data.close()

    # 'team' is a list of dictionnaries
    for robot_data in mission['team']:
        with open(robot_data['description']) as json_robot:
            robot_data.update( json.load( json_robot ) )
            json_robot.close()

    if VERBOSITY_LEVEL > 2:
        print "[Mission] Job description :"
        pprint(mission)

    return mission

