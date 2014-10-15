"""
Inspired by :
http://eli.thegreenplace.net/2010/01/22/weighted-random-generation-in-python

Cyril Robin -- LAAS-CNRS -- 2014

The WeightedRandomGenerator class aims at efficiently select a random element
from some kind of container, with the chances of each element to be selected
not being equal, but defined by relative "weights" (or probabilities). This is
called weighted random selection.

"""

import random
import bisect
import operator as operator
import numpy as np

def accumulate(iterable, func=operator.add):
    'Return running totals (taken from itertools.accumulate() in Python3.2)'
    # accumulate([1,2,3,4,5]) --> 1 3 6 10 15
    # accumulate([1,2,3,4,5], operator.mul) --> 1 2 6 24 120
    it = iter(iterable)
    total = next(it)
    yield total
    for element in it:
        total = func(total, element)
        yield total

class WeightedRandomGenerator(object):
    'Class of random generator on 2D numpy array distribution'

    """ Init the weighted random generator
    by computing the flat 1D array of cumulative weights. This allows to quickly
    generate weight random value. """
    def __init__(self, weights):
        self.totals = []
        running_total = 0

        #for w in weights.reshape(-1): # flattening (1D)
            #running_total += w
            #self.totals.append(running_total)

        # OR
        weights_uint64 = np.uint64(weights)
        self.totals=list( accumulate(weights_uint64.flat, func=operator.add) )


    """ Generate one weighted random item """
    def next(self):
        rnd = random.random() * self.totals[-1]
        idx = bisect.bisect_right(self.totals, rnd)
        return idx

    def __call__(self):
        return self.next()

