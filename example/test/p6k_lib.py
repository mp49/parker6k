#!/usr/bin/python

import sys

import cothread
from cothread.catools import *

class p6k_lib(object):
    """
    Library of useful test functions for parker6k applications
    """


    def move(self, motor, position, timeout):
        """
        Move motor to position. We use put_callback
        and check final position is within RDBD.
        """
    
        try:
            caput(motor, position, wait=True, timeout=timeout)
        except:
            e = sys.exc_info()
            print str(e)
            print "ERROR: caput failed."
            print (motor + " pos:" + str(position) + " timeout:" + str(timeout))
            return 1
        
        rdbd = motor + ".RDBD"
        rbv = motor + ".RBV"

        final_pos = caget(rbv)
        deadband = caget(rdbd)

        if ((final_pos < position-deadband) or (final_pos > position+deadband)):
            print "ERROR: final_pos out of deadband."
            print (motor + " " + str(position) + " " + str(timeout) + " " 
                   + str(final_pos) + " " + str(deadband))
            return 1

        return 0

