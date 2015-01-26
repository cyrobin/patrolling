"""
Cyril Robin -- LAAS-CNRS -- 2014

Gather the shared constant and the empirically set parameters
(and also some magic numbers)
"""

# Display, dump/log, and verbosity level:
#   0 = no output but errors (no warnings)
#   1 = dumped files + log (no out on std::out ; no warnings)
#   2 = additional textual output (warnings, timers, main steps, etc.)
#   3 = talkactive level
VERBOSITY_LEVEL = 2

DISPLAY_COM_LINKS =  False

FSIZE = (12,12) # plot (figure size)
OBS_COLOR = 'green'
COM_COLOR = 'red'
ROBOT_COLORS = ('cyan','magenta','yellow','purple','darkgoldenrod', \
        'lightgreen','firebrick','blue')

# Sampling option
SAMPLING_TIME_OUT = 5 # 5s
N_SAMPLED_POS=20 # Number of accessible positions sampled for each robot (see robot.py)
N_SAMPLED_OBS=100 # Number of obsevable points sampled (see mission.py)
#N_SAMPLED_POS=6 # Number of accessible positions sampled for each robot (see robot.py)
#N_SAMPLED_OBS=40 # Number of obsevable points sampled (see mission.py)
#N_SAMPLED_POS=40 # Number of accessible positions sampled for each robot (see robot.py)
#N_SAMPLED_OBS=100 # Number of obsevable points sampled (see mission.py)
MIN_SAMPLING_DIST = 5 # see mission.py
USE_WEIGHTED_MAP = False
USE_WEIGHTED_MAP = True
FAST_WEIGHTED_SAMPLING = False
FAST_WEIGHTED_SAMPLING = True
FAST_WEIGHTED_SAMPLING_FACTOR = 10

# SOLVER SETTINGS
COST_PENALTY = 0.0001
#SOLVER_TIME_OUT = 300000 # 5min (300K ms)
SOLVER_TIME_OUT = 30000 # 30s (300K ms)

# Utility growth rate
UTILITY_GROWTH_BY_PERIOD = 20

