#!/usr/bin/python

"""
Author: Matt Pearson
Date: May 2014

Description: Class to hold utility functions
"""

import sys

import cothread
from cothread.catools import *

from p6k_globals import p6k_globals

class p6k_lib(object):
    """
    Library of useful test functions for parker6k applications
    """

    __g = p6k_globals()

    def testComplete(self, fail):
        """
        Function to be called at end of test
        fail = true or false
        """
        if not fail:
            print "Test Complete"
            return self.__g.SUCCESS
        else:
            print "Test Failed"
            return self.__g.FAIL


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
            return self.__g.FAIL
        
        rdbd = motor + ".RDBD"
        rbv = motor + ".RBV"

        final_pos = caget(rbv)
        deadband = caget(rdbd)

        success = True

        if ((final_pos < position-deadband) or (final_pos > position+deadband)):
            print "ERROR: final_pos out of deadband."
            print (motor + " " + str(position) + " " + str(timeout) + " " 
                   + str(final_pos) + " " + str(deadband))
            success = False

        if (success):
            return self.postMoveCheck(motor)
        else:
            self.postMoveCheck(motor)
            return self.__g.FAIL


    def setPosition(self, motor, position, timeout):
        """
        Set position on motor and check it worked ok.
        """
        
        _set = motor + ".SET"
        _rbv = motor + ".RBV"
        _dval = motor + ".DVAL"
        _off = motor + ".OFF"

        offset = caget(_off)

        caput(_set, 1, wait=True, timeout=timeout)
        caput(_dval, position, wait=True, timeout=timeout)
        caput(_off, offset, wait=True, timeout=timeout)
        caput(_set, 0, wait=True, timeout=timeout)

        if (self.postMoveCheck(motor) != self.__g.SUCCESS):
            return self.__g.FAIL

        try:
            self.verifyPosition(motor, position+offset)
        except Exception as e:
            print str(e)
            return self.__g.FAIL

    def checkInitRecord(self, motor):
        """
        Check the record for correct state at start of test.
        """
        self.postMoveCheck()


    def verifyPosition(self, motor, position):
        """
        Verify that field == reference.
        """
        _rdbd = motor + ".RDBD"
        deadband = caget(_rdbd)
        _rbv = motor + ".RBV"
        current_pos = caget(_rbv)
        
        if ((current_pos < position-deadband) or (current_pos > position+deadband)):
            print "ERROR: final_pos out of deadband."
            msg = (motor + " " + str(position) + " " 
                   + str(current_pos) + " " + str(deadband))
            raise Exception(__name__ + msg)
        
        return self.__g.SUCCESS

    def verifyField(self, pv, field, reference):
        """
        Verify that field == reference.
        """
        full_pv = pv + "." + field
        if (caget(full_pv) != reference):
            msg = "ERROR: " + full_pv + " not equal to " + str(reference)
            raise Exception(__name__ + msg)

        return self.__g.SUCCESS


    def postMoveCheck(self, motor):
        """
        Check the motor for the correct state at the end of move.
        """
        
        DMOV = 1
        MOVN = 0
        STAT = 0
        SEVR = 0
        LVIO = 0
        MISS = 0
        RHLS = 0
        RLLS = 0

        try:
            self.verifyField(motor, "DMOV", DMOV)
            self.verifyField(motor, "MOVN", MOVN)
            self.verifyField(motor, "STAT", STAT)
            self.verifyField(motor, "SEVR", SEVR)
            self.verifyField(motor, "LVIO", LVIO)
            self.verifyField(motor, "MISS", MISS)
            self.verifyField(motor, "RHLS", RHLS)
            self.verifyField(motor, "RLLS", RLLS)
        except Exception as e:
            print str(e)
            return self.__g.FAIL

        return self.__g.SUCCESS
            
        


