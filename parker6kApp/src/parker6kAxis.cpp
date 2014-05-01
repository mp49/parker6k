/********************************************
 *  p6kAxis.cpp
 * 
 *  P6K Asyn motor based on the 
 *  asynMotorAxis class.
 * 
 *  Matt Pearson
 *  26 March 2014
 * 
 ********************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsExit.h>
#include <epicsString.h>
#include <iocsh.h>

#include "parker6kController.h"
#include <iostream>
using std::cout;
using std::endl;

/* TAS Status Bits (position in char array, not TAS bit position) */
const epicsUInt32 p6kAxis::P6K_TAS_MOVING_        = 0;
const epicsUInt32 p6kAxis::P6K_TAS_DIRECTION_     = 1;
const epicsUInt32 p6kAxis::P6K_TAS_ACCELERATING_  = 2;
const epicsUInt32 p6kAxis::P6K_TAS_ATVELOCITY_    = 3;
const epicsUInt32 p6kAxis::P6K_TAS_HOMED_         = 5;
const epicsUInt32 p6kAxis::P6K_TAS_ABSOLUTE_      = 6;
const epicsUInt32 p6kAxis::P6K_TAS_CONTINUOUS_    = 7;
const epicsUInt32 p6kAxis::P6K_TAS_JOG_           = 8;
const epicsUInt32 p6kAxis::P6K_TAS_JOYSTICK_      = 10;
const epicsUInt32 p6kAxis::P6K_TAS_STALL_         = 13;
const epicsUInt32 p6kAxis::P6K_TAS_DRIVE_         = 15;
const epicsUInt32 p6kAxis::P6K_TAS_DRIVEFAULT_    = 16;
const epicsUInt32 p6kAxis::P6K_TAS_POSLIM_        = 17;
const epicsUInt32 p6kAxis::P6K_TAS_NEGLIM_        = 18;
const epicsUInt32 p6kAxis::P6K_TAS_POSLIMSOFT_    = 20;
const epicsUInt32 p6kAxis::P6K_TAS_NEGLIMSOFT_    = 21;
const epicsUInt32 p6kAxis::P6K_TAS_POSERROR_      = 27;
const epicsUInt32 p6kAxis::P6K_TAS_TARGETZONE_    = 28;
const epicsUInt32 p6kAxis::P6K_TAS_TARGETTIMEOUT_ = 30;
const epicsUInt32 p6kAxis::P6K_TAS_GOWHENPEND_    = 31;
const epicsUInt32 p6kAxis::P6K_TAS_MOVEPEND_      = 33; 
const epicsUInt32 p6kAxis::P6K_TAS_PREEMPT_       = 36;

const epicsUInt32 p6kAxis::P6K_STEPPER_       = 0;
const epicsUInt32 p6kAxis::P6K_SERVO_       = 1;

const epicsUInt32 p6kAxis::P6K_LIM_DISABLE_       = 0;
const epicsUInt32 p6kAxis::P6K_LIM_ENABLE_       = 3;

static void shutdownCallback(void *pPvt)
{
  p6kController *pC = static_cast<p6kController *>(pPvt);

  pC->lock();
  pC->shuttingDown_ = 1;
  pC->unlock();
}

/**
 * p6kAxis constructor.
 * @param pC Pointer to a p6kController object.
 * @param axisNo The axis number for this p6kAxis (1 based).
 */
p6kAxis::p6kAxis(p6kController *pC, int32_t axisNo)
  :   asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  static const char *functionName = "p6kAxis::p6kAxis";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (axisNo > pC_->numAxes_-1) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR: Axis number out of range. Max: %d\n", 
	      functionName, pC_->numAxes_-1);
    return;
  }

  //Initialize non-static data members
  setpointPosition_ = 0.0;
  encoderPosition_ = 0.0;
  currentVelocity_ = 0.0;
  velocity_ = 0.0;
  accel_ = 0.0;
  highLimit_ = 0.0;
  lowLimit_ = 0.0;
  limitsDisabled_ = 0;
  deferredPosition_ = 0.0;
  deferredMove_ = 0;
  deferredRelative_ = 0;
  previous_position_ = 0.0;
  previous_direction_ = 0;
  amp_enabled_ = 0;
  fatal_following_ = 0;
  encoder_axis_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  printNextError_ = false;
  commandError_ = false;

  /* Set an EPICS exit handler that will shut down polling before asyn kills the IP sockets */
  epicsAtExit(shutdownCallback, pC_);

  //Initialise some axis specifc parameters
  bool paramStatus = true;
  paramStatus = ((setIntegerParam(pC_->P6K_A_DRES_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_ERES_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_DRIVE_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_MaxDigits_, 2) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->motorStatusHasEncoder_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->motorStatusGainSupport_, 1) == asynSuccess) && paramStatus);
  paramStatus = ((setStringParam(pC_->P6K_A_Command_, " ") == asynSuccess) && paramStatus);
  paramStatus = ((setStringParam(pC_->P6K_A_Command_RBV_, " ") == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_LS_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_LH_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setStringParam(pC_->P6K_A_Error_, " ") == asynSuccess) && paramStatus);
  if (!paramStatus) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s Unable To Set Driver Parameters In Constructor. Axis:%d\n", 
	      functionName, axisNo_);
  }
  
  //Do an initial poll to get some values from the P6K
  if (axisNo_ > 0) {
    if (getAxisInitialStatus() != asynSuccess) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
		"%s: getAxisInitialStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n", 
		functionName, pC_->portName, axisNo_);
      return;
    }
  }

  callParamCallbacks();

  /* Wake up the poller task which will make it do a poll, 
   * updating values for this axis to use the new resolution (stepSize_) */   
  pC_->wakeupPoller();
 
}

/**
 * Wrapper for common read int param operation at startup.
 * @param cmd The command to send
 * @param param The asyn param to set with the result
 * @param val The result read back
 * @return asynStatus
 */
asynStatus p6kAxis::readIntParam(const char *cmd, epicsUInt32 param, uint32_t *val)
{
  char command[P6K_MAXBUF] = {0};
  char response[P6K_MAXBUF] = {0};
  char scan[P6K_MAXBUF] = {0};
  uint32_t nvals = 0;
  uint32_t axisNum = 0;
  asynStatus status = asynSuccess; 

  static const char *functionName = "p6kAxis::readIntParam";
  
  snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, cmd);
  status = pC_->lowLevelWriteRead(command, response);
  if (status == asynSuccess) {
    snprintf(scan,  P6K_MAXBUF, "%%d%s%%d", cmd);
    nvals = sscanf(response, scan, &axisNum, val);    
    if (nvals == 2) {
      setIntegerParam(param, *val);
      status = asynSuccess;
    } else {
      status = asynError;
    }
  }

  if (status != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR: Failed to read %s at startup.\n", functionName, cmd);
  }
  
  return status;

} 


/**
 * Wrapper for common read double param operation at startup.
 * @param cmd The command to send
 * @param param The asyn param to set with the result
 * @param val The result read back
 * @return asynStatus
 */
asynStatus p6kAxis::readDoubleParam(const char *cmd, epicsUInt32 param, double *val)
{
  char command[P6K_MAXBUF] = {0};
  char response[P6K_MAXBUF] = {0};
  char scan[P6K_MAXBUF] = {0};
  uint32_t nvals = 0;
  uint32_t axisNum = 0;
  asynStatus status = asynSuccess; 

  static const char *functionName = "p6kAxis::readDoubleParam";
  
  snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, cmd);
  status = pC_->lowLevelWriteRead(command, response);
  if (status == asynSuccess) {
    snprintf(scan,  P6K_MAXBUF, "%%d%s%%lf", cmd);
    nvals = sscanf(response, scan, &axisNum, val);
    if (nvals == 2) {
      setDoubleParam(param, *val);
      status = asynSuccess;
    } else {
      status = asynError;
    }
  }

  if (status != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR: Failed to read %s at startup.\n", functionName, cmd);
  }

  return status;
} 


/**
 * Poll for initial axis status (soft limits, PID settings).
 * Set parameters needed for correct motor record behaviour.
 * @return asynStatus
 */
asynStatus p6kAxis::getAxisInitialStatus(void)
{
  uint32_t intVal = 0;
  double doubleVal = 0.0;
  bool stat = true;

  static const char *functionName = "p6kAxis::getAxisInitialStatus";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (axisNo_ != 0) {

    stat = (readIntParam(P6K_CMD_AXSDEF, pC_->P6K_A_AXSDEF_, &intVal) == asynSuccess) && stat;
    if (stat) {
      driveType_ = intVal;
    }
    stat = (readIntParam(P6K_CMD_DRES, pC_->P6K_A_DRES_, &intVal) == asynSuccess) && stat;
    stat = (readIntParam(P6K_CMD_ERES, pC_->P6K_A_ERES_, &intVal) == asynSuccess) && stat;
    stat = (readIntParam(P6K_CMD_DRIVE, pC_->P6K_A_DRIVE_, &intVal) == asynSuccess) && stat;
    stat = (readIntParam(P6K_CMD_ENCCNT, pC_->motorStatusHasEncoder_, &intVal) == asynSuccess) && stat;
    stat = (readIntParam(P6K_CMD_LH, pC_->P6K_A_LH_, &intVal) == asynSuccess) && stat;
    stat = (readIntParam(P6K_CMD_LS, pC_->P6K_A_LS_, &intVal) == asynSuccess) && stat;
    stat = (readDoubleParam(P6K_CMD_LSPOS, pC_->motorHighLimit_, &doubleVal) == asynSuccess) && stat;
    stat = (readDoubleParam(P6K_CMD_LSNEG, pC_->motorLowLimit_, &doubleVal) == asynSuccess) && stat;

  }

  //  setIntegerParam(pC_->motorStatusHasEncoder_, 1);

  if (!stat) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR: Could not read all axis parameters at startup.\n", functionName);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    return asynError;
  } else {
    printAxisParams();
    return asynSuccess;
  }
  
  return asynSuccess;
}


p6kAxis::~p6kAxis() 
{
  //Destructor
}

void p6kAxis::printAxisParams(void)
{
  int32_t  intVal = 0;
  double doubleVal = 0.0;
  
  printf("Axis %d\n", axisNo_);
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_AXSDEF_, &intVal);
  printf("  "P6K_CMD_AXSDEF": %d\n", intVal);
  if (static_cast<epicsUInt32>(intVal) == P6K_STEPPER_) {
    printf("  Stepper Drive\n");
  } else if (static_cast<epicsUInt32>(intVal) == P6K_SERVO_) {
    printf("  Servo Drive\n");
  } else {
    printf("  Unknown Drive Type\n");
  }
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_DRIVE_, &intVal);
  printf("  "P6K_CMD_DRIVE": %d\n", intVal);
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_DRES_, &intVal);  
  printf("  "P6K_CMD_DRES": %d\n", intVal);
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_ERES_, &intVal);
  printf("  "P6K_CMD_ERES": %d\n", intVal);
  pC_->getIntegerParam(axisNo_, pC_->motorStatusHasEncoder_, &intVal);
  printf("  "P6K_CMD_ENCCNT": %d\n", intVal);
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_LS_, &intVal);
  printf("  "P6K_CMD_LS": %d\n", intVal);
  if (static_cast<epicsUInt32>(intVal) != P6K_LIM_ENABLE_) {
    printf("  WARNING: One or both soft limits are disabled.\n");
  }
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_LH_, &intVal);
  printf("  "P6K_CMD_LH": %d\n", intVal);
  if (static_cast<epicsUInt32>(intVal) != P6K_LIM_ENABLE_) {
    printf("  WARNING: One or both hard limits are disabled.\n");
  }
  pC_->getDoubleParam(axisNo_, pC_->motorHighLimit_, &doubleVal);
  printf("  "P6K_CMD_LSPOS": %f\n", doubleVal);
  pC_->getDoubleParam(axisNo_, pC_->motorLowLimit_, &doubleVal);
  printf("  "P6K_CMD_LSNEG": %f\n", doubleVal);

}

/**
 * See asynMotorAxis::move
 */
asynStatus p6kAxis::move(double position, int32_t relative, double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::move";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  int32_t axisDef = 0;
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_AXSDEF_, &axisDef);
  cout << functionName << " axisDef: " << axisDef << endl;

  int32_t maxDigits = 0;
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_MaxDigits_, &maxDigits);
  cout << functionName << " maxDigits: " << maxDigits << endl;

  //Read DRES and ERES for velocity and accel scaling
  int32_t dres = 0;
  int32_t eres = 0;
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_DRES_, &dres);
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_ERES_, &eres);
  int32_t scale = 0;
  if (axisDef == 0) {
    scale = eres;
  } else {
    scale = dres;
  }

  cout << " scale factor: " << scale << endl;
  
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s DRES=%d, ERES=%d\n", functionName, dres, eres);
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s scale=%d\n", functionName, scale);

  if (relative > 1) {
    relative = 1;
  }
  cout << functionName << " relative: " << relative << endl;
  snprintf(command, P6K_MAXBUF, "%dMA%d", axisNo_, !relative);
  status = pC_->lowLevelWriteRead(command, response);
  memset(command, 0, sizeof(command));

  if (max_velocity != 0) {
    cout << functionName << " max_velocity: " << max_velocity << endl;
    epicsFloat64 vel = max_velocity / scale;
    snprintf(command, P6K_MAXBUF, "%dV%.*f", axisNo_, maxDigits, vel);
    status = pC_->lowLevelWriteRead(command, response);
    memset(command, 0, sizeof(command));
  }

  if (acceleration != 0) {
    if (max_velocity != 0) {
      printf("%s  acceleration: %f\n", functionName, acceleration);
      epicsFloat64 accel = acceleration / scale;
      printf("%s  A: %f\n", functionName, accel);
      snprintf(command, P6K_MAXBUF, "%dA%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));
      
      //Set S curve parameters too
      printf("%s  AA: %f\n", functionName, accel/2);
      snprintf(command, P6K_MAXBUF, "%dAA%.*f", axisNo_, maxDigits, accel/2);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));

      
      snprintf(command, P6K_MAXBUF, "%dAD%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));

      snprintf(command, P6K_MAXBUF, "%dADA%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));
    }
  }
  
  //Don't set position if we are doing deferred moves.
  //In case we cancel the deferred move.
  epicsUInt32 pos = static_cast<epicsUInt32>(position);
  if (pC_->movesDeferred_ == 0) {
    snprintf(command, P6K_MAXBUF, "%dD%d", axisNo_, pos);
    status = pC_->lowLevelWriteRead(command, response);
    memset(command, 0, sizeof(command));
    snprintf(command, P6K_MAXBUF, "%dGO", axisNo_);
  } else { /* deferred moves */
    deferredPosition_ = pos;
    deferredMove_ = 1;
    //deferredRelative_ = relative; //This is already taken care of on the controller by the MA command
  }
        
  status = pC_->lowLevelWriteRead(command, response);

  //Check the status of the GO command so we are notified of failed moves.
  if (status != asynSuccess) {
    setStringParam(pC_->P6K_A_Error_, response);
    commandError_ = true;
  } else {
    setStringParam(pC_->P6K_A_Error_, " ");
    commandError_ = false;
  }
  
  return status;
}


/**
 * See asynMotorAxis::home
 */ 
asynStatus p6kAxis::home(double min_velocity, double max_velocity, double acceleration, int32_t forwards)
{
  asynStatus status = asynError;
  //  char command[P6K_MAXBUF] = {0};
  //char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::home";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s Homing not implemented yet.\n", functionName);

  return status;
}

/**
 * See asynMotorAxis::moveVelocity
 */
asynStatus p6kAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration)
{
  asynStatus status = asynError;
  //  char command[P6K_MAXBUF]  = {0};
  //char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::moveVelocity";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s moveVelocity not implemented yet.\n", functionName);

  return status;
}

/**
 * See asynMotorAxis::setPosition
 */
asynStatus p6kAxis::setPosition(double position)
{
  asynStatus asynStatus = asynError;
  bool stat = true;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::setPosition";
  
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  /*Set position on motor axis.*/
  epicsInt32 pos = static_cast<epicsInt32>(floor(position + 0.5));

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	    "%s: Set axis %d on controller %s to position %d\n", 
	    functionName, axisNo_, pC_->portName, pos);

  snprintf(command, P6K_MAXBUF, "!%dS", axisNo_);
  stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  memset(command, 0, sizeof(command));

  if (stat) {
    snprintf(command, P6K_MAXBUF, "%dPSET%d", axisNo_, pos);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  memset(command, 0, sizeof(command));
  }

  /*Now set position on encoder axis.*/
  if (stat) {             
    epicsFloat64 encRatio = 0.0;
    pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encRatio);

    epicsInt32 encpos = (epicsInt32) floor((position*encRatio) + 0.5);

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	      "%s: Set encoder axis %d on controller %s to position %d, encRatio: %f\n", 
	      functionName, axisNo_, pC_->portName, pos, encRatio);
    
    snprintf(command, P6K_MAXBUF, "%dPESET%d", axisNo_, encpos);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    memset(command, 0, sizeof(command));
  }

  /*Now do a fast update, to get the new position from the controller.*/
  bool moving = true;
  getAxisStatus(&moving);
 
  if (!stat) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Failed to set position on axis %d on controller %s.\n", 
	      functionName, axisNo_, pC_->portName);
    asynStatus = asynError;
  } else {
    asynStatus = asynSuccess;
  }

  return asynStatus;
}

/**
 * See asynMotorAxis::stop
 */
asynStatus p6kAxis::stop(double acceleration)
{
  asynStatus status = asynError;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::stopAxis";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  snprintf(command, P6K_MAXBUF, "!%dS", axisNo_);
  status = pC_->lowLevelWriteRead(command, response);

  deferredMove_ = 0;

  return status;
}

asynStatus p6kAxis::setEncoderRatio(double ratio)
{ 
  asynStatus status = asynSuccess;
  static const char *functionName = "p6kAxis::setEncoderRatio";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  status = setDoubleParam(pC_->motorEncoderRatio_, ratio);
  return status;
}

asynStatus p6kAxis::setHighLimit(double highLimit)
{
  asynStatus status = asynSuccess;
  bool stat = true;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::setHighLimit";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  epicsInt32 limit = static_cast<epicsInt32>(floor(highLimit + 0.5));

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
	    "%s: Setting high limit on controller %s, axis %d to %d\n",
	    functionName, pC_->portName, axisNo_, limit);
  
  snprintf(command, P6K_MAXBUF, "%d%s3", axisNo_, P6K_CMD_LS);
  stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  memset(command, 0, sizeof(command));
  
  snprintf(command, P6K_MAXBUF, "%d%s%d", axisNo_, P6K_CMD_LSPOS, limit);
  stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  
  if (!stat) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: ERROR: Failed to set high limit on controller %s, axis %d\n",
	      functionName, pC_->portName, axisNo_);
    status = asynError;
  }
  
  return status;
}

asynStatus p6kAxis::setLowLimit(double lowLimit)
{
  asynStatus status = asynSuccess;
  bool stat = true;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::setLowLimit";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  epicsInt32 limit = static_cast<epicsInt32>(floor(lowLimit + 0.5));

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW,
	    "%s: Setting high limit on controller %s, axis %d to %d\n",
	    functionName, pC_->portName, axisNo_, limit);
  
  snprintf(command, P6K_MAXBUF, "%d%s3", axisNo_, P6K_CMD_LS);
  stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  memset(command, 0, sizeof(command));
  
  snprintf(command, P6K_MAXBUF, "%d%s%d", axisNo_, P6K_CMD_LSNEG, limit);
  stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
  
  if (!stat) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: ERROR: Failed to set low limit on controller %s, axis %d\n",
	      functionName, pC_->portName, axisNo_);
    status = asynError;
  }
  
  return status;
}

/**
 * See asynMotorAxis::setClosedLoop
 */
asynStatus p6kAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynError;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::setClosedLoop";
 
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
 
  if (closedLoop) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	      "%s Drive enable on axis %d\n", functionName, axisNo_);
    sprintf(command, "%dDRIVE1",  axisNo_);
  } else {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	      "%s Drive disable on axis %d\n", functionName, axisNo_);
    sprintf(command, "%dDRIVE0",  axisNo_);
  }
  status = pC_->lowLevelWriteRead(command, response);
  return status;
}

/**
 * See asynMotorAxis::poll
 */
asynStatus p6kAxis::poll(bool *moving)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "p6kAxis::poll";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s Polling axis: %d\n", functionName, this->axisNo_);

  if (axisNo_ != 0) {

    if (!pC_->lowLevelPortUser_) {
      setIntegerParam(pC_->motorStatusCommsError_, 1);
      return asynError;
    }
    
    //Now poll axis status
    if ((status = getAxisStatus(moving)) != asynSuccess) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
		"Controller %s Axis %d. %s: getAxisStatus failed to return asynSuccess.\n", pC_->portName, axisNo_, functionName);
    }
  }
  
  callParamCallbacks();
  return status;
}


/**
 * Read the axis status and set axis related parameters.
 * @param moving Boolean flag to indicate if the axis is moving. This is set by this function
 * to indcate to the polling thread how quickly to poll for status.
 * @return asynStatus
 */
asynStatus p6kAxis::getAxisStatus(bool *moving)
{
    char command[P6K_MAXBUF] = {0};
    char response[P6K_MAXBUF] = {0};
    bool stat = true;
    int32_t nvals = 0;
    int32_t axisNum = 0;
    bool printErrors = true;
    int32_t intVal = 0;
    char stringVal[P6K_MAXBUF] = {0};
    bool doneMoving = false;

    static const char *functionName = "p6kAxis::getAxisStatus";
    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
    
    //Get the time and decide if we want to print errors.
    //Crude error message throttling.
    epicsTimeGetCurrent(&nowTime_);
    nowTimeSecs_ = nowTime_.secPastEpoch;
    if ((nowTimeSecs_ - lastTimeSecs_) < pC_->P6K_ERROR_PRINT_TIME_) {
      printErrors = 0;
    } else {
      printErrors = 1;
      lastTimeSecs_ = nowTimeSecs_;
    }
    
    if (printNextError_) {
      printErrors = 1;
    }

    //printf("Axis: %d\n", axisNo_);

    /* Transfer current position and encoder position.*/
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TPC);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_TPC"%d", &axisNum, &intVal);
      if (nvals == 2) {
	setDoubleParam(pC_->motorPosition_, intVal);
      }
    }
    memset(command, 0, sizeof(command));

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TPE);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_TPE"%d", &axisNum, &intVal);
      if (nvals == 2) {
	setDoubleParam(pC_->motorEncoderPosition_, intVal);
      }
    }
    memset(command, 0, sizeof(command));

    /* Transfer axis status */
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TAS);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      //printf("  Status response: %s\n", response);
      nvals = sscanf(response, "%d"P6K_CMD_TAS"%s", &axisNum, stringVal);
      if (nvals != 2) {
	stat = false;
      } 
    }
    memset(command, 0, sizeof(command));

    if (!stat) {
      if (printErrors) {
	asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "ERROR: Problem reading position and status on controller %s, axis %d\n", 
		  pC_->portName, axisNo_);
	printNextError_ = false;
      }
    } else {

      if (deferredMove_) {
	doneMoving = false; 
      } else {
	doneMoving = !(stringVal[P6K_TAS_MOVING_] == pC_->P6K_ON_);
      }
      
      if (doneMoving) {
	if (driveType_ == P6K_SERVO_) {
	  bool targetZone = (stringVal[P6K_TAS_TARGETZONE_] == pC_->P6K_ON_);
	  doneMoving = targetZone && !(stringVal[P6K_TAS_TARGETTIMEOUT_] == pC_->P6K_ON_);
	}
      }
      
      if (!doneMoving) {
	*moving = true;
      } else {
	*moving = false;
      }

      //TODO: Also read TER status here, as there is more info this status message
      
      
      stat = (setIntegerParam(pC_->motorStatusDone_, 
	      doneMoving) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusMoving_, 
	     (stringVal[P6K_TAS_MOVING_] == pC_->P6K_ON_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusDirection_, 
	     (stringVal[P6K_TAS_DIRECTION_] == pC_->P6K_OFF_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusHighLimit_, 
	    ((stringVal[P6K_TAS_POSLIM_] || stringVal[P6K_TAS_POSLIMSOFT_]) == pC_->P6K_ON_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusLowLimit_, 
	    ((stringVal[P6K_TAS_NEGLIM_] || stringVal[P6K_TAS_NEGLIMSOFT_]) == pC_->P6K_ON_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusHomed_, 
	     (stringVal[P6K_TAS_HOMED_] == pC_->P6K_ON_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusHomed_, 
	     (stringVal[P6K_TAS_HOMED_] == pC_->P6K_ON_)) == asynSuccess) && stat;
      stat = (setIntegerParam(pC_->motorStatusPowerOn_, 
	     (stringVal[P6K_TAS_DRIVE_] == pC_->P6K_OFF_)) == asynSuccess) && stat;
      
      //      cout << "TAS DRIVE BIT: " << stringVal[P6K_TAS_DRIVE_] << endl;

      if (driveType_ == P6K_SERVO_) {
	stat = (setIntegerParam(pC_->motorStatusFollowingError_, 
               (stringVal[P6K_TAS_POSERROR_] == pC_->P6K_ON_)) == asynSuccess) && stat;
      } else {
	stat = (setIntegerParam(pC_->motorStatusFollowingError_, 
               (stringVal[P6K_TAS_STALL_] == pC_->P6K_ON_)) == asynSuccess) && stat;
      }
      
      uint32_t problem = 0;
      problem = 
	(stringVal[P6K_TAS_DRIVEFAULT_] == pC_->P6K_ON_) |
	(stringVal[P6K_TAS_TARGETTIMEOUT_] == pC_->P6K_ON_) |
	(stringVal[P6K_TAS_POSERROR_] == pC_->P6K_ON_) |
	commandError_;
      stat = (setIntegerParam(pC_->motorStatusProblem_, (problem!=0)) == asynSuccess) && stat;
      
      if (!stat) {
	if (printErrors) {
	  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
		    "ERROR: Problem setting params on controller %s, axis %d\n", pC_->portName, axisNo_);
	  printNextError_ = false;
	}
      }
    }

    //Clear error print flag for this axis if problem has been removed.
    if (stat) {
      printNextError_ = true;
    }
    
    //This currently isn't checked by base class polling thread.
    return asynSuccess;
}

