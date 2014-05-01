#!../../bin/linux-x86_64/example

## You may have to change example to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/example.dbd"
example_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadRecords "db/example.db"

####################################################
# P6K Controller

drvAsynIPPortConfigure("6K","192.168.200.153:4001",0,0,0)

#This prints low level commands and responses
#asynSetTraceMask("6K",0,0x2)
#asynSetTraceIOMask("6K",0,0x2)

#asynSetTraceMask("6K",0,0xFF)
#asynSetTraceIOMask("6K",0,0xFF)

p6kCreateController("P6K","6K",0,2,500,1000)

#asynSetTraceMask("P6K",0,0xFF)
#asynSetTraceIOMask("P6K",0,0xFF)
#asynSetTraceMask("P6K",1,0xFF)
#asynSetTraceIOMask("P6K",1,0xFF)

p6kCreateAxis("P6K",1)
p6kCreateAxis("P6K",2)

####################################################


cd ${TOP}/iocBoot/${IOC}
iocInit

