"""
Cyril Robin -- LAAS-CNRS -- 2014

Gather the shared constant and the empirically set parameters
(and also some magic numbers)
"""

# Display
VERBOSE=False
#VERBOSE=True

FSIZE = (12,12) # plot (figure size)
COLORS = ('green','cyan' ,'firebrick' ,'yellow' ,'blue' , \
        'purple','darkgoldenrod' ,'red')

# Sampling option
SAMPLING_TIME_OUT = 5 # 5s
#N_SAMPLED_POS = 15 # Number of accessible position sampled for each robot (see robot.py)
#N_SAMPLED_OBS = 100 # Number of obsevable points sampled (see mission.py)
N_SAMPLED_POS = 8 # Number of accessible position sampled for each robot (see robot.py)
N_SAMPLED_OBS = 20 # Number of obsevable points sampled (see mission.py)
MIN_SAMPLING_DIST = 5 # see mission.py

# SOLVER SETTINGS
COST_PENALTY = 0.0001
SOLVER_TIME_OUT = 300000 # 5min (300K ms)

# Utility growth rate
UTILITY_GROWTH_BY_PERIOD = 20

