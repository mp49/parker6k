TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build the IOC support library

LIBRARY_IOC += parker6kSupport

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# Install devXxxSoft.dbd into <top>/dbd
#DBD += xxxSupport.dbd

# Compile and add the code to the support library
#parker6kSupport_SRCS += xxxRecord.c
#parker6kSupport_SRCS += devXxxSoft.c

parker6kSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

