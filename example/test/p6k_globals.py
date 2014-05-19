#!/usr/bin/python

"""
Author: Matt Pearson
Date: May 2014

Description: Class to hold globals for parker6k example test scripts
"""

class p6k_globals(object):
    """
    Class to hold globals for parker6k example test scripts.

    Constants are local variables to methods, and they are called
    by either the method name or the read-only property associated with that
    method.
    """

    def __init__(self):
        pass

    def getMotor(self):
        return "BL99:Mot:P6K1"

    MOTOR = property(getMotor, doc="Motor PV name")

