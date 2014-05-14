#!../../bin/linux-x86_64/example

## You may have to change example to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

epicsEnvSet("P6K_CONFIG", "/home/controls/motion/devel/matt/config")

## Register all support components
dbLoadDatabase "dbd/example.dbd"
example_registerRecordDeviceDriver pdbbase

####################################################
# P6K Controller

drvAsynIPPortConfigure("6K","192.168.200.177:4001",0,0,0)

#This prints low level commands and responses
#asynSetTraceMask("6K",0,0x2)
#asynSetTraceIOMask("6K",0,0x2)

#asynSetTraceMask("6K",0,0xFF)
#asynSetTraceIOMask("6K",0,0xFF)

p6kCreateController("P6K","6K",0,2,500,1000,0)
p6kUpload("P6K", "$(P6K_CONFIG)")

#asynSetTraceMask("P6K",0,0xFF)
#asynSetTraceIOMask("P6K",0,0xFF)
#asynSetTraceMask("P6K",1,0xFF)
#asynSetTraceIOMask("P6K",1,0xFF)

p6kCreateAxis("P6K",1)
p6kCreateAxis("P6K",2)

####################################################

## Load record instances
dbLoadRecords "db/example.db"

#################################################
# autosave

epicsEnvSet("IOCNAME","p6kExample")
epicsEnvSet("SAVE_DIR","/tmp/p6kExample")

save_restoreSet_Debug(0)

### status-PV prefix, so save_restore can find its status PV's.
save_restoreSet_status_prefix("BL99:Mot:P6K:")

set_requestfile_path("$(SAVE_DIR)")
set_savefile_path("$(SAVE_DIR)")

save_restoreSet_NumSeqFiles(3)
save_restoreSet_SeqPeriodInSeconds(600)
set_pass0_restoreFile("$(IOCNAME).sav")
set_pass0_restoreFile("$(IOCNAME)_pass0.sav")
set_pass1_restoreFile("$(IOCNAME).sav")

#################################################

cd ${TOP}/iocBoot/${IOC}
iocInit

# Create request file and start periodic 'save'
epicsThreadSleep(5)
makeAutosaveFileFromDbInfo("$(SAVE_DIR)/$(IOCNAME).req", "autosaveFields")
makeAutosaveFileFromDbInfo("$(SAVE_DIR)/$(IOCNAME)_pass0.req", "autosaveFields_pass0")
create_monitor_set("$(IOCNAME).req", 5)
create_monitor_set("$(IOCNAME)_pass0.req", 30)

