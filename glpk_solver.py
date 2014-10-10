"""
Use Python Math Programming (pympl) to use glpk through python
See:
    /home/crobin/dev/src/pympl-4.2
    ~/dev/lib/python2.7/site-packages
"""

from pymprog import *
from pprint import pprint
import itertools

class GLPKSolver:
    'Class embedding the GLPK solvers'

    """ Init the solver according to the mission. """
    def __init__(self, mission):

        self.mission = mission
        self.cost_penalty = 0.0001 # FIXME MAGIC NUMBER

    def solve_top(self):

        # DATA: define useful sets
        R = self.mission.team
        N = {r:r.points for r in R}
        M = [(r,p) for r in R for p in N[r]]
        E = [(r,p,q) for r in R for p in N[r] for q in N[r] ]
        T =  self.mission.period # Maximal cost allowed

        # Utility = weighted sum of observed areas
        u = { (r,p): sum( self.mission.map.image[q] * r.sensor(p,q) for q in self.mission.points) \
                for r in R for p in N[r] }

        # DECISION VARIABLES
        pb = model('Team Orienteering Problem through flow formulation')

        # x[r,p,q] is a boolean variable that indicates if robot r goes from p to q
        x = pb.var(E, 'x', bool)

        # NETWORK CONSTRAINTS (INCLUDING FINISH CONSTRAINTS)
        # Only use valid path links
        pb.st( [ x[r,p,q] == 0 for r,p,q in E \
            if not p in r.paths or not q in r.paths[p] ], 'check path validity')

        # each robot leaves its own starting position (once!)
        pb.st( [ sum( x[r,r.pos[0:2],q] for q in N[r] ) == 1 for r in R ], \
                'leave starting postion' )

        # nw define the last position, which is unique
        nw = pb.var( M, 'nw', bool) # == 1 iff final node, 0 otherwise
        pb.st( [ sum( nw[r,p] for p in N[r] ) == 1 for r in R ], 'unique final pose' )

        # robots entering an accessible node must leave the same node but the final one
        pb.st( [ sum( x[r,q,p] for q in N[r] ) \
               - sum( x[r,p,q] for q in N[r] ) \
               - nw[r,p] \
               == 0 for r,p in M  if p != r.pos[0:2] ], 'enter' )

        # Go to the position only once
        pb.st( [ sum( x[r,q,p] for q in N[r] ) <= 1 for r in R for p in N[r] ], 'enter once' )

        # MTZ Subtour Eliminating Constraints
        z = pb.var( M, 'z', int) #integer >=0
        pb.st( [ z[r,p] >= 0 for r,p in M ])
        pb.st( [ z[r,r.pos[0:2]] == 0 for r in R ])
        pb.st( [ z[r,p] <= len(N[r]) for r,p in M ])
        pb.st( [ z[r,p] - z[r,q] + 1 <= ( len(N[r]) ) * (1 - x[r,p,q]) for r,p,q in E ] )

        # Cost limit
        plan_cost = pb.var(R, 'plan cost', float)
        pb.st( [ plan_cost[r] <= T for r in R ], 'maximal cost allowed')
        pb.st( [ sum( r.cost(p,q)*x[r,p,q] for p in N[r] for q in N[r]) <= plan_cost[r] for r in R ], 'compute plan cost')

        print "GLPK: init done. Solving..."

        # OBJECTIVE
        pb.max( sum( u[r,j]*x[r,i,j] - self.cost_penalty*plan_cost[r] for r,i,j in E), 'utility' )

        pb.solve() #solve the TOP problem

        # TODO clean up
        #c = 0
        #for r in R:
            #for p in N[r]:
                #print "{} : {} : {} ({}) : # {}".format(r,p,nw[r,p].primal,sum( x[r,q,p].primal for q in N[r] ) - sum( x[r,p,q].primal for q in N[r] ), z[r,p].primal )
                #for q in N[r]:
                    #if x[r,p,q].primal ==  1:
                        #print "{} : {} ------> {}".format(r,p,q)
                        #c+=1
                    #else :
                        #print "{} : {} xxxxxxx {}".format(r,p,q)
                #print ""
            #print "-----------------------------------------"

        #print c

        # Retrieve solution
        for r in R:
            r.plan = []
            curr = r.pos[0:2]
            r.plan.append(curr)
            for s in N[r]:
                for p,q in itertools.product(N[r],N[r]):
                    if p == curr and x[r,p,q].primal ==  1:
                        #print (p,q)
                        curr = q
                        r.plan.append(curr)
            print(r.plan)

        print "Gathered utility = %.2f :-) " % sum( u[r,j]*x[r,i,j].primal for r,i,j in E)
        print "vs Global cost = %.2f " % sum(plan_cost[r].primal for r in R )

