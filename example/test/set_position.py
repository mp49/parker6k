#!/usr/bin/python

import sys
import time

from p6k_lib import p6k_lib
from p6k_globals import p6k_globals
    
def main():
    
    pv = str(sys.argv[1]) 

    print "Set position test on motor " + pv

    lib = p6k_lib()
    g = p6k_globals()

    positions = [0, 1, 1.234, -1.1, 0]

    for pos in positions:
        print "Set position to " + str(pos)
        stat = lib.setPosition(pv, pos, g.TIMEOUT)
        if (stat == g.FAIL):
            sys.exit(lib.testComplete(g.FAIL))

    sys.exit(lib.testComplete(g.SUCCESS))


if __name__ == "__main__":
    main()
