""" Timer

inspired by
http://stackoverflow.com/questions/5849800/tic-toc-functions-analog-in-python

Usage :

    with Timer('foo_stuff'):
       # do some foo
       # do some stuff
"""

import time

class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '[%s]' % self.name,
        print 'Elapsed: %s s' % (time.time() - self.tstart)

