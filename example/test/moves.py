#!/usr/bin/python

import sys

import cothread
from cothread.catools import *

from p6k_lib import p6k_lib

__MOTOR="BL99:Mot:P6K1"
__TIMEOUT=100
    
def main():

   print "Test move sequence on motor " + __MOTOR
    
   __lib = p6k_lib()

   positions = range(10)

   for pos in positions:
       print "Move to " + str(pos)
       stat = __lib.move(__MOTOR, pos, __TIMEOUT)
       if (stat == 1):
           sys.exit(1)
       
   print "Test complete"
   sys.exit(0)
   


if __name__ == "__main__":
        main()
