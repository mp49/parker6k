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

    print "Test drive power control"
    
    on = 1
    off = 0

    lib = motor_lib()
    g = motor_globals()

    #Enable auto drive control
    caput(pv1+":AutoEnable", on, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisable", on, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoEnableDelay", 0, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisableDelay", 0, wait=True, timeout=g.TIMEOUT)

    stat = lib.move(pv1, 0, g.TIMEOUT)
    if (stat == g.FAIL):
        sys.exit(lib.testComplete(g.FAIL))

    #Disable auto drive control
    caput(pv1+":AutoEnable", off, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisable", off, wait=True, timeout=g.TIMEOUT)

    for i in range(10):

        print "Cyling drive power " + str(i)

        #Enable power
        caput(pv1+".CNEN", on, wait=True, timeout=g.TIMEOUT)
        power = caget(pv1+".CNEN")
        if (power != on):
            print "Failed to enable drive."
            sys.exit(lib.testComplete(g.FAIL))
    
        time.sleep(2)

        #Disable power
        caput(pv1+".CNEN", off, wait=True, timeout=g.TIMEOUT)
        power = caget(pv1+".CNEN")
        if (power != off):
            print "Failed to disable drive."
            sys.exit(lib.testComplete(g.FAIL))
    
        time.sleep(2)
    
    #Enable power and do a move
    caput(pv1+".CNEN", on, wait=True, timeout=g.TIMEOUT)    
    stat = lib.move(pv1, 10, g.TIMEOUT)
    if (stat == g.FAIL):
        sys.exit(lib.testComplete(g.FAIL))
    caput(pv1+".CNEN", on, wait=True, timeout=g.TIMEOUT)

    #Enable auto drive control
    caput(pv1+":AutoEnable", on, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisable", on, wait=True, timeout=g.TIMEOUT)
    stat = lib.move(pv1, 0, g.TIMEOUT)
    if (stat == g.FAIL):
        sys.exit(lib.testComplete(g.FAIL))
    
    #Test auto enable/disable delay
    caput(pv1+":AutoEnableDelay", 5000, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisableDelay", 5, wait=True, timeout=g.TIMEOUT)
    stat = lib.move(pv1, 10, g.TIMEOUT)
    if (stat == g.FAIL):
        sys.exit(lib.testComplete(g.FAIL))
    #Make sure drive is still enabled
    power = caget(pv1+".CNEN")
    if (power != on):
        print "Drive is disabled when it should be disabled at end of move."
        sys.exit(lib.testComplete(g.FAIL))
    time.sleep(10)
    power = caget(pv1+".CNEN")
    if (power == on):
        print "Drive is still enabled at end of move."
        sys.exit(lib.testComplete(g.FAIL))

    caput(pv1+":AutoEnableDelay", 0, wait=True, timeout=g.TIMEOUT)
    caput(pv1+":AutoDisableDelay", 0, wait=True, timeout=g.TIMEOUT)

    positions = range(0,100)
    for pos in positions:
        pos = float(pos)/100.0
        print "Move to " + str(pos)
        stat = lib.move(pv1, pos, g.TIMEOUT)
        if (stat == g.FAIL):
            sys.exit(lib.testComplete(g.FAIL))
        #Wait 2s to ensure drive is off
        time.sleep(2)
        power = caget(pv1+".CNEN")
        if (power == on):
            print "Drive is still enabled at end of multi move."
            sys.exit(lib.testComplete(g.FAIL))

    sys.exit(lib.testComplete(g.SUCCESS))
   

if __name__ == "__main__":
        main()
