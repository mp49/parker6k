#!/usr/bin/python

import sys
import os

filename = "run_tests.txt"

def main():

    pv = sys.argv[1]
    
    print "Running tests in " + str(os.getcwd()) + " on " + pv

    data = None
    try:
        file = open(filename)
        data = file.read().splitlines()
        file.close()
    except IOError:
        print __file__ + " ERROR: Could not read " + filename
        data = None
        sys.exit(1)

    stat = 0
    for T in data:
        stat = os.system(T + " " + pv)
        if stat > 0:
            print  __file__ + " ERROR: Test " + T + " failed."
            sys.exit(1)

    print "Success"
    sys.exit(0)


if __name__ == "__main__":
    main()
