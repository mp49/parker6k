#!/usr/bin/python

import sys
import time

import cothread
from cothread.catools import *

sys.path.append('/home/controls/epics/motor/proj_delay/examples/tests/')

from motor_lib import motor_lib
from motor_globals import motor_globals

def main():

    pv1 = sys.argv[1]

    print "Test post move delay handling."

    lib = motor_lib()
    g = motor_globals()

    caput(pv1+":DelayTime", 0, wait=True, timeout=g.TIMEOUT)
    stat = lib.move(pv1, 0, g.TIMEOUT)
    if (stat == g.FAIL):
        sys.exit(lib.testComplete(g.FAIL))

    delays = [0, 0.1, 0.2, 0.5, 1, 2, 5, 20]
    positions = range(0,10)
    for delay in delays:
        caput(pv1+":DelayTime", delay, wait=True, timeout=g.TIMEOUT)
        for pos in positions:
            print "Move to " + str(float(pos/10.0)) + " with post move delay " + str(delay)
            stat = lib.move(pv1, float(pos/10.0), g.TIMEOUT)
            if (stat == g.FAIL):
                sys.exit(lib.testComplete(g.FAIL))
    positions = range(0,20,2)
    for delay in delays:
        caput(pv1+":DelayTime", delay, wait=True, timeout=g.TIMEOUT)
        for pos in positions:
            print "Move to " + str(pos) + " with post move delay " + str(delay)
            stat = lib.move(pv1, pos, g.TIMEOUT)
            if (stat == g.FAIL):
                sys.exit(lib.testComplete(g.FAIL))

    sys.exit(lib.testComplete(g.SUCCESS))
   

if __name__ == "__main__":
        main()
