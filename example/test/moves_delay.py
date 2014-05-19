#!/usr/bin/python

import sys
import time

from p6k_lib import p6k_lib
from p6k_globals import p6k_globals
    
def main():
    
    pv = str(sys.argv[1]) 
    delay = float(sys.argv[2])
    
    print "Test move sequence on motor " + pv + " with delays " + str(delay)
    
    lib = p6k_lib()
    g = p6k_globals()

    positions = range(10)

    for pos in positions:
        print "Move to " + str(pos)
        stat = lib.move(pv, pos, g.TIMEOUT)
        if (stat == g.FAIL):
            sys.exit(lib.testComplete(g>FAIL))
        time.sleep(delay)

    sys.exit(lib.testComplete(g.SUCCESS))

if __name__ == "__main__":
        main()
