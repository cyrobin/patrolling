/* Team Orienteering Problem

   Based on the document "patrolling formalism"

    2014

    TODO change text below
 A set of airplanes are initially distributed among a set of
 starting locations. They are to be assigned routes to collectively
 visit a specified set of customers then return the the planes to
 designated finishing locations. The optimization objective is to
 minimize the total great circle distance.

 The data consists of a set of locations with latitude and longitude
 information, a list of customers and their respective locations, and
 a set of aircraft and their starting and finishing locations. The
 aircraft must start and finish at different locations (if needed,
 dummy locations with the same latitude and longitude can be 
 included in the list of locations).

 Jeffrey Kantor
 March, 2013
*/


# DATA SETS (TO BE GIVEN IN THE DATA SECTION)
###############################################################################

# ROBOTS is a set of (name, start_location) pairs
set ROBOTS dimen 2;

# POSITIONS is a set of (robot,location) pairs meaning that location is accessible to robot
set POSITIONS dimen 2;

# set of locations (x_coordinate, y_coordinate, utility_of_the_location)
set LOCATIONS;
param px{LOCATIONS};
param py{LOCATIONS};
param u{LOCATIONS};

# DATA PREPROCESSING
###############################################################################

# set of robots
set R := setof {(r,sLoc) in ROBOTS} r;

# create a complete graph of nodes as (robot, location) pairs
set START := setof {(r,sLoc) in ROBOTS} (r,sLoc); 
# NB: START is not useless: ROBOTS can easily be added some new features
set N := POSITIONS union START;
# ALTERNATIVE: ensure a FINISH position for each robot
/*set FINISH := setof {(r,sLoc,fLoc) in ROBOTS} (r,fLoc);*/
/*set N := POSITIONS union (START union FINISH);*/

# Cost of the path from a to b, for robot r
# currently, use euclidian distances between locations
param cost{r in R, a in LOCATIONS, b in LOCATIONS} :=
    sqrt( (px[a]-px[b])**2 + (py[a]-py[b])**2 );
# ALTERNATIVE: use external data or other function (like robot speed?)

# DECISION VARIABLES
###############################################################################

# x[r,r,aLoc,r,bLoc] = 1 if robot r flies from (r,aLoc) to (r,bLoc)
# NB: a robot can only use (r,LOC) nodes (ie node that are accessible)
var x{R, N, N} binary;

# NETWORK CONSTRAINTS (INCLUDING FINISH CONSTRAINTS)
###############################################################################

var d{R,POSITIONS} binary; # = 1 iff final node, 0 otherwise

# robots cannot use the nodes that belong to other robots
s.t. nw0a {r in R, (a,aLoc) in N, (b,bLoc) in N : r != a} : x[r,a,aLoc,b,bLoc] = 0;
s.t. nw0b {r in R, (a,aLoc) in N, (b,bLoc) in N : r != b} : x[r,a,aLoc,b,bLoc] = 0;
s.t. nw0c {r in R, (a,aLoc) in N, (b,bLoc) in N : a != b} : x[r,a,aLoc,b,bLoc] = 0;

# each robot leaves its own start node (once!)
s.t. nw1 {r in R, (a,aLoc) in START : r = a} :
        sum { (b,bLoc) in POSITIONS } x[r,a,aLoc,b,bLoc] = 1;

# robots entering an accessible node must leave the same node but the final one
s.t. nw2 {r in R, (a,aLoc) in POSITIONS: r = a} :
    sum {(b,bLoc) in (POSITIONS union START) : r = b} x[r,b,bLoc,a,aLoc]
    - sum {(c,cLoc) in (POSITIONS) : r = c} x[r,a,aLoc,c,cLoc]
    = d[r,a,aLoc]; # = 1 if a final, 0 otherwise

# there is only one final node for each robot
s.t. nw3 {r in R} :
    sum {(b,bLoc) in (POSITIONS)} d[r,b,bLoc] = 1;

# no self loops
s.t. nw4 {r in R, (a,aLoc) in N, (b,bLoc) in N : (a=b) && (aLoc=bLoc)} :
    x[r,a,aLoc,b,bLoc] = 0;

# SUBTOUR ELIMINATION CONSTRAINTS
###############################################################################

# MTZ Subtour Eliminating Constraints

var z{R,N} integer, >= 1;

# n(r) = number of positions accessible to the robot r
param n{r in R} :=
    card( setof { (a,aLoc) in N: r = a} (a,aLoc) ) ;

s.t. sb1 {r in R, (a,aLoc) in START: r = a} :
    z[r,a,aLoc] = 1;

s.t. sb2 {r in R, (a,aLoc) in N: r = a} :
    z[r,a,aLoc] <= n[r];

s.t. sb3 {r in R, (a,aLoc) in N, (b,bLoc) in N: r = a && r = b} :
    z[r,a,aLoc] - z[r,b,bLoc] + 1 <= (n[r] - 1) * (1 - x[r,a,aLoc,b,bLoc]);

/*# OBJECTIVE*/
/*###############################################################################*/

# Cost of the route of each robot
var routeCost{R} >= 0;

s.t. ob1 {r in R} : routeCost[r]
        = sum{(a,aLoc) in N, (b,bLoc) in N: r = a && r = b} cost[r,aLoc,bLoc]*x[r,a,aLoc,b,bLoc];

# Max 'cost' allowed for the route of each robot
param T, >= 0;

s.t. ob2 {r in R} : routeCost[r] <= T;

# Is the location visited or not (by at least one robot) ?
var y{LOCATIONS} binary; # = 1 iff the place is visited, 0 otherwise

# y[l] is to maximize, but = 1 only if visited (there exists one x[..., l ,...] = 1 )
# (equivalent to the linearization of the min function)
s.t. ob5 {l in LOCATIONS} : y[l]
        <= sum{r in R, (a,aLoc) in N, (b,bLoc) in N: r = a && r = b && l = aLoc } x[r,a,aLoc,b,bLoc];

# Maximize the utility gathered along the path
maximize util : sum{l in LOCATIONS} u[l]*y[l];
/*maximize util : sum{l in LOCATIONS} y[l];*/
/*maximize util : sum{r in R, (a,aLoc) in N, (b,bLoc) in N} x[r,a,aLoc,b,bLoc];*/
/*minimize totalCost : sum{r in R} routeCost[r];*/

solve;

/*# OUTPUT POST-PROCESSING*/
/*###############################################################################*/


printf "\nUtility gathered = %s\n-------------------\n", util;

for {r in R} {
    printf "\nRouting for %s\n-------------------\n", r;
    printf "%-20s  %-20s  %10s   \n", 'Depart','Arrive','Cost.';
    /*for {k in routeLegs[p]..0 by -1} {*/
    for {k in 0..n[r] by 1} {
       printf {(a,aLoc) in N, (b,bLoc) in N :
           (x[r,a,aLoc,b,bLoc] = 1) && (z[r,a,aLoc]=k)}
           "%-12s  %-5s   %-12s  %-5s   %10.1f km\n",a,aLoc,b,bLoc,cost[r,aLoc,bLoc];
    }
    printf "%42s  %13s\n", '', '---------';
    printf "%42s  %10.1f  %10.1f\n\n", 'Global Cost:', routeCost[r],T;
}

/*for {r in R, (a,aLoc) in N, (b,bLoc) in N} {*/
    /*printf "%1.0f %10s  %-12s  %-5s   %-12s  %-5s   %10.1f km  %10.1f\n",*/
        /*x[r,a,aLoc,b,bLoc], r,a,aLoc,b,bLoc,cost[r,aLoc,bLoc],u[aLoc];*/
/*}*/

/*for {r in R} {*/
    /*printf "\nRouting for %s\n-------------------\n", r;*/
    /*printf "%-20s  %-20s  %10s   \n", 'Depart','Arrive','Cost.';*/
    /*printf {(a,aLoc) in N, (b,bLoc) in N : (x[r,a,aLoc,b,bLoc] = 1)}*/
        /*"%-12s  %-5s   %-12s  %-5s   %10.1f km %10.1f\n",a,aLoc,b,bLoc,cost[r,aLoc,bLoc],u[aLoc];*/
    /*printf "%42s  %13s\n", '', '---------';*/
    /*printf "%42s  %10.1f  %10.1f\n\n", 'Global Cost:', routeCost[r],T;*/
/*}*/

/*printf "%-12s  %-5s  %10s   \n", 'robot','place','#Visits';*/
/*printf {(a,aLoc) in N: (y[aLoc] >= 1)}*/
    /*"%-12s  %-5s %8.0f\n",a,aLoc,y[aLoc];*/

/*# DATA SECTION*/
/*###############################################################################*/

data;

/*param T := 9999.0;*/
/*param T := 99.0;*/
param T := 50.0;

set POSITIONS :=
       ( 'robot 1', ATL )
       ( 'robot 1', BOS )
       ( 'robot 1', DEN )
       ( 'robot 1', DFW )
       ( 'robot 1', JFK )
       ( 'robot 1', LAX )
       ( 'robot 1', ORD )
       ( 'robot 1', STL )
       ( 'robot 2', ATL )
       ( 'robot 2', BOS )
       ( 'robot 2', DEN )
       ( 'robot 2', DFW )
       ( 'robot 2', JFK )
       ( 'robot 2', LAX )
       ( 'robot 2', ORD )
       ( 'robot 2', STL )
       ( 'robot 3', ATL )
       ( 'robot 3', BOS )
       ( 'robot 3', DEN )
       ( 'robot 3', DFW )
       ( 'robot 3', JFK )
       ( 'robot 3', LAX )
       ( 'robot 3', ORD )
       ( 'robot 3', STL )
;

set ROBOTS :=
       ( 'robot 1', ORD_ )  # use a duplicate location
       ( 'robot 2', DFW_ )  # use a duplicate location
       ( 'robot 3', JFK_ )  # use a duplicate location
;

param : LOCATIONS : px          py      u :=
        ATL   33.6366995   -84.4278639      5
        BOS   42.3629722   -71.0064167      1
        DEN   39.8616667  -104.6731667      3
        DFW   32.8968281   -97.0379958      5
        DFW_  32.8968281   -97.0379958      0 # duplicate ROBOTS
        JFK   40.6397511   -73.7789256      8
        JFK_  40.6397511   -73.7789256      8 # duplicate ROBOTS
        LAX   33.9424955  -118.4080684      7
        ORD   41.9816486   -87.9066714      3
        ORD_  41.9816486   -87.9066714      0 # duplicate ROBOTS
        STL   38.7486972   -90.3700289      4
;

end;
