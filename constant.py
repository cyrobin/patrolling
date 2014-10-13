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
N_SAMPLED_POS = 15 # see Robot.py

# SOLVER SETTINGS
COST_PENALTY = 0.0001
SOLVER_TIME_OUT = 300000 # 5min (300K ms)

