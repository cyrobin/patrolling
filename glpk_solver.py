"""
Use Python Math Programming (pympl) to use glpk through python
See:
    /home/crobin/dev/src/pympl-4.2
    ~/dev/lib/python2.7/site-packages

Cyril Robin -- LAAS-CNRS -- 2014

TODO Descriptif

"""

from pymprog import *
from pprint import pprint
import itertools

from constant import *

class GLPKSolver:
    'Class embedding the GLPK solvers'

    """ Init the solver. """
    def __init__(self):

        self.cost_penalty = COST_PENALTY

    """ Solve the problem as a position-based multi-TSP.
    Update the plan of the team of Robots according to the best solution found
    before the time out, and return the solver status ('undef' or 'feas' or
    'opt' for 'no solution found', 'found one feasible solution', and 'found
    optimal solution' respectively. One may provide available comlinks,
    otherwise the solver will not consider the necessity of communications
    while planning."""
    def solve_position_tsp(self, team, utility_map, observable_points, period,
            available_comlinks = None ):

        # Utility = do not take sensor model into account
        # Solely consider the current position utility
        def computed_utility( robot, position):
            return utility_map.image[position]

        return self._solve_tsp(computed_utility, team, period, available_comlinks)

    """ Solve the problem as a perception-based multi-TSP.
    Update the plan of the team of Robots according to the best solution found
    before the time out, and return the solver status ('undef' or 'feas' or
    'opt' for 'no solution found', 'found one feasible solution', and 'found
    optimal solution' respectively. One may provide available comlinks,
    otherwise the solver will not consider the necessity of communications
    while planning."""
    def solve_perception_tsp(self, team, utility_map, observable_points, period,
            available_comlinks = None ):

        # Utility = weighted sum of observed areas
        def computed_utility( robot, position):
            return sum( utility_map.image[observed] * \
                    robot.sensor(position,observed) \
                    for observed in observable_points )

        return self._solve_tsp(computed_utility, team, period, available_comlinks)

    """ Solve a multi-tsp-like problem as a flow formulation. The utility
    function is given as an argument (mandatory).  Update the plan of the team
    of Robots according to the best solution found before the time out, and
    return the solver status ('undef' or 'feas' or 'opt' for 'no solution
    found', 'found one feasible solution', and 'found optimal solution'
    respectively. One may provide available comlinks, otherwise the solver will
    not consider the necessity of communications while planning."""
    def _solve_tsp(self, computed_utility, team, period, \
            available_comlinks = None ):

        # DATA: define useful sets
        R = team
        N = {r:r.points for r in R}
        M = [(r,p) for r in R for p in N[r]]
        E = [(r,p,q) for r in R for p in N[r] for q in N[r] ]
        T =  period # Maximal cost allowed

        # Utility = weighted sum of observed areas
        u = { (r,p): computed_utility(r,p) for r in R for p in N[r] }

        # Comlink model (Boolean-like, == 1 if there is an effective comlink)
        if available_comlinks:
            """ There is a comlink if both ends can etablished a link. At least
            one is needed."""
            def check_comlink(r,p):
                res = [ r.comlink(p,q) * partner.comlink(q,p) \
                        for (partner,q) in available_comlinks ]
                res.append(1)
                return  min(res)

            g = { (r,p): check_comlink(r,p) for r in R for p in N[r] }

        else:
            # When there is no available comlink, assume global com is available
            if VERBOSITY_LEVEL > 0:
                print "!WARNING! [Planning] No comlink available: assume full communication links instead."
            g = { (r,p): 1 for r in R for p in N[r] }

        # Pymprog init and option
        pb = model('Team Orienteering Problem through flow formulation')

        # see http://pymprog.sourceforge.net/solvopt.html for options
        # or  http://www.cnd.mcgill.ca/~ivan/it_ineq_script/python%20LP%20solvers/pympl.4.2/Doc/solvopt.rst
        #pb.solvopt(method='exact', verbosity=2) # seems less efficient
        pb.solvopt(tm_lim=SOLVER_TIME_OUT,verbosity=2)
        if VERBOSITY_LEVEL > 2:
            print "Solver options: {}".format( pb.solvopt() )

        # DECISION VARIABLES
        # x[r,p,q] is a boolean variable that indicates if robot r goes from p to q
        x = pb.var(E, 'x', bool)

        # NETWORK CONSTRAINTS (INCLUDING FINISH CONSTRAINTS)
        # Only use valid path links
        pb.st( [ x[r,p,q] == 0 for r,p,q in E \
            if not p in r.paths or not q in r.paths[p] ], 'check path validity')

        # each robot leaves its own starting position (once!)
        pb.st( [ sum( x[r,r.pose[0:2],q] for q in N[r] ) == 1 for r in R ], \
                'leave starting postion' )

        # nw define the last position, which is unique
        nw = pb.var( M, 'nw', bool) # == 1 iff final node, 0 otherwise
        pb.st( [ sum( nw[r,p] for p in N[r] ) == 1 for r in R ], 'unique final pose' )
        pb.st( [ nw[r,p] <=  g[r,p] for r in R for p in N[r] ], 'Must communicate at the final pose' )

        # robots entering an accessible node must leave the same node but the final one
        pb.st( [ sum( x[r,q,p] for q in N[r] ) \
               - sum( x[r,p,q] for q in N[r] ) \
               - nw[r,p] \
               == 0 for r,p in M  if p != r.pose[0:2] ], 'enter' )

        # Go to the position only once
        pb.st( [ sum( x[r,q,p] for q in N[r] ) <= 1 for r in R for p in N[r] ], 'enter once' )

        # MTZ Subtour Eliminating Constraints
        z = pb.var( M, 'z', int) #integer >=0
        pb.st( [ z[r,p] >= 0 for r,p in M ])
        pb.st( [ z[r,r.pose[0:2]] == 0 for r in R ])
        pb.st( [ z[r,p] <= len(N[r]) for r,p in M ])
        pb.st( [ z[r,p] - z[r,q] + 1 <= ( len(N[r]) ) * (1 - x[r,p,q]) for r,p,q in E ] )

        # Cost limit
        plan_cost = pb.var(R, 'plan cost', float)
        pb.st( [ plan_cost[r] <= T for r in R ], 'maximal cost allowed')
        pb.st( [ sum( r.cost(p,q)*x[r,p,q] for p in N[r] for q in N[r]) <= plan_cost[r] for r in R ], 'compute plan cost')

        if VERBOSITY_LEVEL > 2:
            print "[Planning] GLPK: init done. Solving..."

        # OBJECTIVE
        pb.max( sum( u[r,j]*x[r,i,j] - self.cost_penalty*plan_cost[r] for r,i,j in E), 'utility' )

        pb.solve() #solve the TOP problem

        if VERBOSITY_LEVEL > 1:
            print "[Planning] GLPK Solver status:",pb.status()
        # Report Karush-Kuhn-Tucker optimality conditions (= error bounds)
        if VERBOSITY_LEVEL > 2:
            print pb.reportKKT()

        # Retrieve solution
        for r in R:
            r.plan = []
            curr = r.pose[0:2]
            r.plan.append(curr)
            for s in N[r]:
                for p,q in itertools.product(N[r],N[r]):
                    if p == curr and x[r,p,q].primal ==  1:
                        curr = q
                        r.plan.append(curr)
            if VERBOSITY_LEVEL > 2:
                print "[Planning] {} ({} chekpoints) : {}".format(r.name,len(r.plan),r.plan)
                print r.plan

        if VERBOSITY_LEVEL > 0 and pb.status() == 'undef':
            print "!WARNING! NO SOLUTION found by the GLPK solver so far."
        elif VERBOSITY_LEVEL > 1:
            print "[Planning] Gathered utility = %.2f" % sum( u[r,j]*x[r,i,j].primal for r,i,j in E)
            print "[Planning] for a Global cost = %.2f " % sum(plan_cost[r].primal for r in R )

        # Return status:
        # - feas = solution found (but no necessary optimal)
        # - undef = no solution so far
        return pb.status()


    """ Solve a perception TOP problem as a flow formulation.
    Update the plan of the team of Robots according to the best solution found
    before the time out, and return the solver status ('undef' or 'feas' or
    'opt' for 'no solution found', 'found one feasible solution', and 'found
    optimal solution' respectively.
    One may provide available comlinks, otherwise the solver will not consider
    the necessity of communications while planning."""
    def solve_ptop(self, team, utility_map, observable_points, period,
            available_comlinks = None ):

        # DATA: define useful sets
        R = team
        N = { r:r.points for r in R }
        M = [ (r,p) for r in R for p in N[r] ]
        E = [ (r,p,q) for r in R for p in N[r] for q in N[r] ]
        Q = [ q for q in observable_points ]
        V = [ (q,m) for q in Q for m in M ]
        T = period # Maximal cost allowed

        # Utility of the observable areas
        def get_utility( observed ):
            return int(utility_map.image[observed]) # int is needed for pymprog
        u = { q: get_utility(q) for q in Q }

        # Comlink model (Boolean-like, == 1 if there is an effective comlink)
        if available_comlinks:
            """ There is a comlink if both ends can etablished a link. At least
            one is needed."""
            def check_comlink(r,p):
                res = [ r.comlink(p,q) * partner.comlink(q,p) \
                        for (partner,q) in available_comlinks ]
                res.append(1)
                return  min(res)

            g = { (r,p): check_comlink(r,p) for r in R for p in N[r] }

        else:
            # When there is no available comlink, assume global com is available
            if VERBOSITY_LEVEL > 0:
                print "!WARNING! [Planning] No comlink available: assume full communication links instead."
            g = { (r,p): 1 for r in R for p in N[r] }

        # Pymprog init and option
        pb = model('Perception Team Orienteering Problem through flow formulation')

        # see http://pymprog.sourceforge.net/solvopt.html for options
        # or  http://www.cnd.mcgill.ca/~ivan/it_ineq_script/python%20LP%20solvers/pympl.4.2/Doc/solvopt.rst
        #pb.solvopt(method='exact', verbosity=2) # seems less efficient
        pb.solvopt(tm_lim=SOLVER_TIME_OUT,verbosity=2)
        if VERBOSITY_LEVEL > 2:
            print "Solver options: {}".format( pb.solvopt() )

        # DECISION VARIABLES

        # x[r,p,q] is a boolean variable that indicates if robot r goes from p to q
        x = pb.var(E, 'x', bool)

        # y[q] is a variable in [0,1] that indicates the quality of the best
        # observation of q by the team
        y = pb.var(Q, 'y', float)
        pb.st( [ 0 <= y[q] <= 1 for q in Q ], 'observations')

        # NETWORK CONSTRAINTS (INCLUDING FINISH CONSTRAINTS)

        # Only use valid path links
        pb.st( [ x[r,p,q] == 0 for r,p,q in E \
            if not p in r.paths or not q in r.paths[p] ], 'check path validity')

        # each robot leaves its own starting position (once!)
        pb.st( [ sum( x[r,r.pose[0:2],q] for q in N[r] ) == 1 for r in R ], \
                'leave starting postion' )

        # nw define the last position, which is unique
        nw = pb.var( M, 'nw', bool) # == 1 iff final node, 0 otherwise
        pb.st( [ sum( nw[r,p] for p in N[r] ) == 1 for r in R ], 'unique final pose' )
        pb.st( [ nw[r,p] <=  g[r,p] for r in R for p in N[r] ], 'Must communicate at the final pose' )

        # robots entering an accessible node must leave the same node but the final one
        pb.st( [ sum( x[r,q,p] for q in N[r] ) \
               - sum( x[r,p,q] for q in N[r] ) \
               - nw[r,p] \
               == 0 for r,p in M  if p != r.pose[0:2] ], 'enter' )

        # Go to the position only once
        pb.st( [ sum( x[r,q,p] for q in N[r] ) <= 1 for r in R for p in N[r] ], 'enter once' )

        # MTZ Subtour Eliminating Constraints
        z = pb.var( M, 'z', int) #integer >=0
        pb.st( [ z[r,p] >= 0 for r,p in M ])
        pb.st( [ z[r,r.pose[0:2]] == 0 for r in R ])
        pb.st( [ z[r,p] <= len(N[r]) for r,p in M ])
        pb.st( [ z[r,p] - z[r,q] + 1 <= ( len(N[r]) ) * (1 - x[r,p,q]) for r,p,q in E ] )

        # Cost limit
        plan_cost = pb.var(R, 'plan cost', float)
        pb.st( [ plan_cost[r] <= T for r in R ], 'maximal cost allowed')
        pb.st( [ sum( r.cost(p,q)*x[r,p,q] for p in N[r] for q in N[r]) <= plan_cost[r] for r in R ], 'compute plan cost')

        # OBJECTIVE = BEST OBSERVATIONS

        # Find the best observation of the observables points (aka compute y)
        # One wants to maximize the y[q], and to be equal to the best observation of q so far
        # Below is a linearization of this max function

        # A "Big C" used to linearize the original pb (max function with max objective)
        C = 9999

        # New binary variables reguired by for the linearization, that, for a
        # given observable areas q, indicates the best observation so far (= a
        # couple (robot r, from position p) )
        v = pb.var( V, 'v', bool)

        pb.st( [ sum( v[q,m] for m in M) == 1 for q in Q ], 'Best observation is unique')

        # = determine which v is equal to one (= which one  the best observation)
        pb.st( [ y[q] <= ( sum(x[r,p2,p] * r.sensor(p,q) for p2 in N[r] )  + C*(1 - v[q,(r,p)]) ) for q in Q for (r,p) in M ], 'Define best observation')

        if VERBOSITY_LEVEL > 2:
            print "[Planning] GLPK: init done. Solving..."

        # OBJECTIVE
        # Maximize the utility gathered along the path
        pb.max( sum( u[q]*y[q] for q in Q) - sum( self.cost_penalty*plan_cost[r] for r in R), 'utility' )

        pb.solve() #solve the TOP problem

        if VERBOSITY_LEVEL > 1:
            print "[Planning] GLPK Solver status:",pb.status()
        # Report Karush-Kuhn-Tucker optimality conditions (= error bounds)
        if VERBOSITY_LEVEL > 2:
            print pb.reportKKT()

        # Retrieve solution
        for r in R:
            r.plan = []
            curr = r.pose[0:2]
            r.plan.append(curr)
            for s in N[r]:
                for p,q in itertools.product(N[r],N[r]):
                    if p == curr and x[r,p,q].primal ==  1:
                        curr = q
                        r.plan.append(curr)
            if VERBOSITY_LEVEL > 2:
                print "[Planning] {} ({} chekpoints) : {}".format(r.name,len(r.plan),r.plan)
                print r.plan

        if VERBOSITY_LEVEL > 0 and pb.status() == 'undef':
            print "!WARNING! NO SOLUTION found by the GLPK solver so far."
        elif VERBOSITY_LEVEL > 1:
            print "[Planning] Gathered utility = %.2f (out of %.2f)" %  ( sum(u[q]*y[q].primal for q in Q) , sum( u[q] for q in Q) )
            print "[Planning] for a Global cost = %.2f " % sum(plan_cost[r].primal for r in R )

        # Return status:
        # - opt = an optimal integer solution has been found !
        # - feas = solution found (but no necessary optimal)
        # - undef = no solution so far
        return pb.status()

