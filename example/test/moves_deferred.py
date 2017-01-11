#!/usr/bin/python

import sys
import time

import cothread
from cothread.catools import *

sys.path.append('/home/controls/epics/motor/master/examples/tests/')

from motor_lib import motor_lib
from motor_globals import motor_globals

def main():

    pv1 = "BL99:Mot:P6K1"
    pv2 = "BL99:Mot:P6K2"
    defer = "BL99:Mot:Controller1:Defer"

    print "Test deferred move sequence on both motors"
    
    lib = motor_lib()
    g = motor_globals()
    
    positions = range(20)
    
    for pos in positions:
        pos2 = pos+1
        print "Set deferred flag."
        caput(defer, 1, wait=True)
        print "Set positions pos: " + str(pos) + " pos2: " + str(pos2)
        caput(pv1, pos, wait=False)
        caput(pv2, pos2, wait=False)
        print "Execute deferred move."
        caput(defer, 0, wait=True)

        #Wait for both motors to finish
        done = 0
        count = 0
        while (done == 0):
            dmov1 = caget(pv1+".DMOV")
            dmov2 = caget(pv2+".DMOV")
            if (dmov1 == 1) and (dmov2 == 1):
                done = 1
                count = 0
                print "Move Finished"
            count = count + 1
            if count > 1000:
                print "Moves did not complete."
                sys.exit(lib.testComplete(g.FAIL))
            time.sleep(0.1)

        try:
            lib.verifyPosition(pv1, pos)
        except Exception as e:
            print str(e)
            sys.exit(lib.testComplete(g.FAIL))

        try:
            lib.verifyPosition(pv2, pos2)
        except Exception as e:
            print str(e)
            sys.exit(lib.testComplete(g.FAIL))

    sys.exit(lib.testComplete(g.SUCCESS))
   

if __name__ == "__main__":
        main()
