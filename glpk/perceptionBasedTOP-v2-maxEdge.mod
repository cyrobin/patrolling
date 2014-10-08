/* Perceptionn-based patrolling Team Orienteering Problem

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
#TODO EVERTHING !

# ROBOTS is a set of (name, start_location, sensor quality) length
/*set ROBOTS dimen 2;*/
set ROBOTS dimen 3;

# POSITIONS is a set of (robot,location) pairs meaning that location is accessible to robot
set POSITIONS dimen 2;

# set of locations (x_coordinate, y_coordinate)
set LOCATIONS;
param px{LOCATIONS};
param py{LOCATIONS};

# set of Areas (x_coordinate, y_coordinate, utility_of_observing_the_area)
set AREAS;
param ax{AREAS};
param ay{AREAS};
param u{AREAS};

# DATA PREPROCESSING
###############################################################################

# set of robots
/*set R := setof {(r,sLoc) in ROBOTS} r;*/
set R := setof {(r,sLoc,sQ) in ROBOTS} r;

# create a complete graph of nodes as (robot, location) pairs
/*set START := setof {(r,sLoc) in ROBOTS} (r,sLoc);*/
set START := setof {(r,sLoc,sQ) in ROBOTS} (r,sLoc);
# NB: START is not useless: ROBOTS can easily be added some new features
# (eg sensor quality)
set N := POSITIONS union START;
# ALTERNATIVE: ensure a FINISH position for each robot
/*set FINISH := setof {(r,sLoc,fLoc) in ROBOTS} (r,fLoc);*/
/*set N := POSITIONS union (START union FINISH);*/

# Cost of the path from a to b, for robot r
# currently, use euclidian distances between locations
param cost{r in R, a in LOCATIONS, b in LOCATIONS} :=
    sqrt( (px[a]-px[b])**2 + (py[a]-py[b])**2 );
# ALTERNATIVE: use external data or other function (like robot speed?)

# SENSORS
# Quality of the observation of a from b by the robot r
# currently, use euclidian distances between locations
# there is a minimal quality under which sensing is not allowed
# transitional virtual vphi params are used for the thresholding process
param minPhi >= 0;
param vphi{ r in R, a in AREAS, b in LOCATIONS } :=
     20 * (sum { (_r,sLoc,sQ) in ROBOTS : r = _r } sQ ) / sqrt((ax[a]-px[b])**2 + (ay[a]-py[b])**2);
param phi{ r in R, a in AREAS, b in LOCATIONS } :=
    if vphi[r,a,b] > minPhi then vphi[r,a,b] else 0;
     # TODO : make max(phi-min,0) * phi / (phi-min)

# DECISION VARIABLES
###############################################################################

# x[r,r,aLoc,r,bLoc] = 1 if robot r flies from (r,aLoc) to (r,bLoc)
# NB: a robot can only use (r,LOC) nodes (ie node that are accessible)
var x{R, N, N} binary;

# Max 'cost' allowed for an edge to be traversed
param maxEdgeCost >= 0;

param XX{r in R, a in LOCATIONS, b in LOCATIONS} :=
    0 + (if cost[r,a,b] <= maxEdgeCost then 1);

# NETWORK CONSTRAINTS (INCLUDING FINISH CONSTRAINTS)
###############################################################################

var d{R,POSITIONS} binary; # = 1 iff final node, 0 otherwise

# robots cannot use the nodes that belong to other robots
s.t. nw0a {r in R, (a,aLoc) in N, (b,bLoc) in N : r != a} : x[r,a,aLoc,b,bLoc] = 0;
s.t. nw0b {r in R, (a,aLoc) in N, (b,bLoc) in N : r != b} : x[r,a,aLoc,b,bLoc] = 0;
s.t. nw0c {r in R, (a,aLoc) in N, (b,bLoc) in N : a != b} : x[r,a,aLoc,b,bLoc] = 0;

s.t. nw0d {r in R, (a,aLoc) in N, (b,bLoc) in N : XX[r,aLoc,bLoc] != 1} : x[r,a,aLoc,b,bLoc] = 0;

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

# no need to get somewhere more than once (?!)
# NOT NEEDED: ENSURED BY THE MTZ ALREADY !!
/*s.t. nw5 {r in R, (b,bLoc) in POSITIONS : r = b} :*/
        /*sum { (a,aLoc) in POSITIONS } x[r,a,aLoc,b,bLoc] <= 1;*/

# robots entering an accessible node must leave the same node but the final one
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
param T >= 0;

s.t. ob2 {r in R} : routeCost[r] <= T;

# Has the areas been observed ? (and how good was the observation ?)
var y{AREAS} >= 0 ; # = 1 iff the place is perfectly observed, 0 if not observed, in between otherwise

s.t. ob3 {l in AREAS} : y[l] <= 1; # not needed ?

# A "Big M" used to linearize the original pb (max function with max objective)
param M >= 0;

# New binary variables reguired by for the linearization
# indicate the best observation of the given area
var v{R,AREAS,LOCATIONS} binary ;

# There is only one best observation
/*s.t. ob4 {l in AREAS}: sum{ r in R, aLoc in LOCATIONS } v[r,l,aLoc] = 1 ;*/
s.t. ob4 {l in AREAS}: sum{ r in R,  (b,bLoc) in POSITIONS: r = b } v[r,l,bLoc] = 1 ;

# y[l] is to maximize, but = to the best observation so far (if observed!)
# (equivalent to the linearization of the max function)
s.t. ob5 {l in AREAS, r in R, (b,bLoc) in N: r = b } : y[l]
    <= ( sum{ (a,aLoc) in N: r = a} x[r,a,aLoc,b,bLoc] ) * phi[r,l,bLoc] + (1 - v[r,l,bLoc])*M ;

# Discard obsrevations which are not good enough
/*s.t. ob6 {l in AREAS, r in R,  bLoc in LOCATIONS : phi[r,l,bLoc] < minPhi }: v[r,l,bLoc] = 0 ;*/
#TODO: find the bug ; why does it discard everything?
# NB: replace by the phi_ and phi truncation

# Maximize the utility gathered along the path
maximize util : sum{l in AREAS} u[l]*y[l];
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

# What a robot visits
/*printf "%-12s  %-5s  %10s   \n", 'robot','place','#Visits';*/
/*printf {(a,aLoc) in N: (y[aLoc] >= 1)}*/
    /*"%-12s  %-5s %8.0f\n",a,aLoc,y[aLoc];*/

printf "%-12s  %-10s  %12s   \n", 'place','utility','quality (y)';
printf {l in AREAS: (y[l] > 0)}
    "%-12s  %-3.3f %7s %-3.3f\n",l,u[l],' ',y[l];
printf {l in AREAS: (y[l] = 0)}
    "%-12s  %-3.3f %7s %-3.3f\n",l,u[l],' ',y[l];

# What is observed from where
/*printf "\n \n";*/
/*printf "%-12s  %-5s %-7s %10s  %10s  %-6s \n", 'place','v','phi','robot','from', 'y';*/
/*[>for {l in AREAS, r in R, (b,bLoc) in POSITIONS: v[r,l,bLoc] = 1 && r = b } {<]*/
/*for {l in AREAS, r in R, (b,bLoc) in POSITIONS: v[r,l,bLoc] = 0 && r = b } {*/
    /*printf "%-12s %-5.0f %-2.5f %10s  %10s  %-2.4f %2.1f\n",*/
        /*l,v[r,l,bLoc], phi[r,l,bLoc], r,bLoc,y[l], (phi[r,l,bLoc] < minPhi) ;*/
/*}*/

# display constraints on the y[l]
/*printf "\n \n";*/
/*for {l in AREAS, r in R, (b,bLoc) in N: r = b }*/
    /*printf "%3.5f %5s %3.2f %2.5f %-5.0f\n",*/
  /*y[l],'<=', sum{ (a,aLoc) in N: r = a} x[r,a,aLoc,b,bLoc], phi[r,l,bLoc], 1 - v[r,l,bLoc] ;*/

# display  the phi value
printf "\n \n minPhi = %-2.3f \n", minPhi;
printf { r in R, a in AREAS, b in LOCATIONS } :
    "%-8s %-8s %-8s %-2.3f\n",r,a,b,phi[r,a,b];

/*# DATA SECTION*/
/*###############################################################################*/

data;

param M:= 100;

/*param T := 9999.0;*/
/*param T := 99.0;*/
param T := 50.0;

/*param maxEdgeCost := 15;*/
param maxEdgeCost := 5;
/*param maxEdgeCost := 99;*/

/*param minPhi := 0.25;*/
/*param minPhi := 1.00;*/
/*param minPhi := 0.00;*/
param minPhi := 0.00;

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
       /*( 'robot 3', ATL )*/
       /*( 'robot 3', BOS )*/
       /*( 'robot 3', DEN )*/
       /*( 'robot 3', DFW )*/
       /*( 'robot 3', JFK )*/
       /*( 'robot 3', LAX )*/
       /*( 'robot 3', ORD )*/
       /*( 'robot 3', STL )*/
       /*( 'robot 4', ATL )*/
       /*( 'robot 4', BOS )*/
       /*( 'robot 4', DEN )*/
       /*( 'robot 4', DFW )*/
       /*( 'robot 4', JFK )*/
       /*( 'robot 4', LAX )*/
       /*( 'robot 4', ORD )*/
       /*( 'robot 4', STL )*/
;

set ROBOTS :=
       ( 'robot 1', ORD_, 1.0 )  # use a duplicate location
       ( 'robot 2', DFW_, 1.0 )  # use a duplicate location
       /*( 'robot 3', JFK_, 1.0 )  # use a duplicate location*/
       /*( 'robot 4', BOS_, 1.0 )  # use a duplicate location*/
;

param : LOCATIONS : px          py      :=
        ATL   33.6366995   -84.4278639
        BOS   42.3629722   -71.0064167
        BOS_  42.3629722   -71.0064167       # duplicate ROBOTS
        DEN   39.8616667  -104.6731667
        DFW   32.8968281   -97.0379958
        DFW_  32.8968281   -97.0379958       # duplicate ROBOTS
        JFK   40.6397511   -73.7789256
        JFK_  40.6397511   -73.7789256       # duplicate ROBOTS
        LAX   33.9424955  -118.4080684
        ORD   41.9816486   -87.9066714
        ORD_  41.9816486   -87.9066714       # duplicate ROBOTS
        STL   38.7486972   -90.3700289
;

param : AREAS : ax          ay      u :=
        a_ATL   33   -84      5
        a_BOS   42   -71      1
        a_DEN   39  -104      3
        a_DFW   32   -97      5
        a_JFK   40   -73      8
        a_LAX   33  -118      7
        a_ORD   41   -87      3
        a_STL   38   -90      4
        b_ATL   33   -84      5
        b_BOS   42   -71      1
        b_DEN   39  -104      3
        b_DFW   32   -97      5
        b_JFK   40   -73      8
        b_LAX   33  -118      7
        b_ORD   41   -87      3
        b_STL   38   -90      4
        c_ATL   33   -84      5
        c_BOS   42   -71      1
        c_DEN   39  -104      3
        c_DFW   32   -97      5
        c_JFK   40   -73      8
        c_LAX   33  -118      7
        c_ORD   41   -87      3
        c_STL   38   -90      4
;

end;
