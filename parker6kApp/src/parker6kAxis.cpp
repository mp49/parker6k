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

const char p6kAxis::P6K_TAS_ON_       = '1';
const char p6kAxis::P6K_TAS_OFF_      = '0';

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

  /* Set an EPICS exit handler that will shut down polling before asyn kills the IP sockets */
  epicsAtExit(shutdownCallback, pC_);

  //Initialise some axis specifc parameters
  bool paramStatus = true;
  paramStatus = ((setIntegerParam(pC_->P6K_A_DRES_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_ERES_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_DRIVE_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_MaxDigits_, 2) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->motorStatusHasEncoder_, 1) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->motorStatusGainSupport_, 1) == asynSuccess) && paramStatus);
  paramStatus = ((setStringParam(pC_->P6K_A_Command_, " ") == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_LS_, 0) == asynSuccess) && paramStatus);
  paramStatus = ((setIntegerParam(pC_->P6K_A_LH_, 0) == asynSuccess) && paramStatus);
  if (!paramStatus) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s Unable To Set Driver Parameters In Constructor. Axis:%d\n", 
	      functionName, axisNo_);
  }
  
  //Do an initial poll to get some values from the P6K
  if (getAxisInitialStatus() != asynSuccess) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
	      "%s: getAxisInitialStatus failed to return asynSuccess. Controller: %s, Axis: %d.\n", 
	      functionName, pC_->portName, axisNo_);
  }

  callParamCallbacks();

  /* Wake up the poller task which will make it do a poll, 
   * updating values for this axis to use the new resolution (stepSize_) */   
  pC_->wakeupPoller();
 
}

/**
 * Poll for initial axis status (soft limits, PID settings).
 * Set parameters needed for correct motor record behaviour.
 * @return asynStatus
 */
asynStatus p6kAxis::getAxisInitialStatus(void)
{
  char command[P6K_MAXBUF] = {0};
  char response[P6K_MAXBUF] = {0};
  asynStatus status = asynSuccess;
  int32_t intVal = 0;
  double doubleVal = 0.0;
  int32_t nvals = 0;
  int32_t softLimit = 0;
  int32_t axisNum = 0;
  bool stat = true;

  static const char *functionName = "p6kAxis::getAxisInitialStatus";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (axisNo_ != 0) {

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_AXSDEF);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_AXSDEF"%d", &axisNum, &intVal);    
      setIntegerParam(pC_->P6K_A_AXSDEF_, intVal);
      driveType_ = intVal;
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_DRES);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_DRES"%d", &axisNum, &intVal);    
      setIntegerParam(pC_->P6K_A_DRES_, intVal);
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_ERES);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_ERES"%d", &axisNum, &intVal);
      setIntegerParam(pC_->P6K_A_ERES_, intVal);
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_DRIVE);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_DRIVE"%d", &axisNum, &intVal);
      setIntegerParam(pC_->P6K_A_DRIVE_, intVal);
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_ENCCNT);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_ENCCNT"%d", &axisNum, &intVal);
      setIntegerParam(pC_->motorStatusHasEncoder_, intVal);
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_LH);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_LH"%d", &axisNum, &intVal);
      setIntegerParam(pC_->P6K_A_LH_, intVal);
    }
    
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_LS);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_LS"%d", &axisNum, &softLimit);
      setIntegerParam(pC_->P6K_A_LS_, intVal);
    }

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_LSPOS);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_LSPOS"%f", &axisNum, &doubleVal);
      setDoubleParam(pC_->motorHighLimit_, doubleVal);
    }
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_LSNEG);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_LSNEG"%f", &axisNum, &doubleVal);
      setDoubleParam(pC_->motorLowLimit_, doubleVal);
    }
   
  }

  //setIntegerParam(pC_->motorStatusGainSupport_, 1);

  if (!stat) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR: Could not read all axis parameters at startup.", functionName);
  }
  
  printAxisParams();
  
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
  if (intVal == P6K_STEPPER_) {
    printf("  Stepper Drive\n");
  } else if (intVal == P6K_SERVO_) {
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
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_LS_, &intVal);
  printf("  "P6K_CMD_LS": %d\n", intVal);
  if (intVal != P6K_LIM_ENABLE_) {
    printf("  WARNING: One or both soft limits are disabled.\n");
  }
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_LH_, &intVal);
  printf("  "P6K_CMD_LH": %d\n", intVal);
  if (intVal != P6K_LIM_ENABLE_) {
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
  static const char *functionName = "p6kAxis::move";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  char acc_buff[P6K_MAXBUF] = {0};
  char vel_buff[P6K_MAXBUF] = {0};
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};

  int32_t axisDef = 0;
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_AXSDEF_, &axisDef);
  cout << functionName << " axisDef: " << endl;

  int32_t maxDigits = 0;
  pC_->getIntegerParam(axisNo_, pC_->P6K_A_MaxDigits_, &maxDigits);
  cout << functionName << " maxDigits: " << endl;

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
  
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s DRES=%d, ERES=%d\n", functionName, dres, eres);

  if (relative > 1) {
    relative = 1;
  }
  cout << functionName << " relative: " << relative << endl;
  sprintf(command, "%dMA%d", axisNo_, !relative);
  status = pC_->lowLevelWriteRead(command, response);
  memset(command, 0, sizeof(command));

  if (max_velocity != 0) {
    cout << functionName << " max_velocity: " << max_velocity << endl;
    epicsFloat64 vel = max_velocity / scale;
    sprintf(command, "%dV%.*f", axisNo_, maxDigits, vel);
    status = pC_->lowLevelWriteRead(command, response);
    memset(command, 0, sizeof(command));
  }

  if (acceleration != 0) {
    if (max_velocity != 0) {
      printf("%s  acceleration: %f\n", functionName, acceleration);
      epicsFloat64 accel = acceleration / scale;
      sprintf(command, "%dA%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));
      
      //Set S curve parameters too
      sprintf(command, "%dAA%.*f", axisNo_, maxDigits, accel/2);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));

      sprintf(command, "%dAD%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));

      sprintf(command, "%dADA%.*f", axisNo_, maxDigits, accel);
      status = pC_->lowLevelWriteRead(command, response);
      memset(command, 0, sizeof(command));
    }
  }
  
  //Don't set position if we are doing deferred moves.
  //In case we cancel the deferred move.
  epicsUInt32 pos = static_cast<epicsUInt32>(position);
  if (pC_->movesDeferred_ == 0) {
    sprintf(command, "%dD%d", axisNo_, pos);
    status = pC_->lowLevelWriteRead(command, response);
    memset(command, 0, sizeof(command));
    sprintf(command, "%dGO", axisNo_);
  } else { /* deferred moves */
    deferredPosition_ = pos;
    deferredMove_ = 1;
    //deferredRelative_ = relative;
  }
        
  status = pC_->lowLevelWriteRead(command, response);
  
  return status;
}


/**
 * See asynMotorAxis::home
 */ 
asynStatus p6kAxis::home(double min_velocity, double max_velocity, double acceleration, int32_t forwards)
{
  asynStatus status = asynError;
  char command[P6K_MAXBUF] = {0};
  char response[P6K_MAXBUF] = {0};
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
  char acc_buff[P6K_MAXBUF] = {0};
  char vel_buff[P6K_MAXBUF] = {0};
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
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
  bool status = true;
  char command[P6K_MAXBUF]  = {0};
  char response[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kAxis::setPosition";
  
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  /*Set position on motor axis.*/
  epicsInt32 pos = static_cast<epicsInt32>(floor(position + 0.5));
    
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	    "%s: Set axis %d on controller %s to position %f\n", 
	    functionName, axisNo_, pC_->portName, pos);

  sprintf(command, "!%dS", axisNo_);
  if ( command[0] != 0 && status) {
    status = (pC_->lowLevelWriteRead(command, response) == asynSuccess);
  }
  memset(command, 0, sizeof(command));

  sprintf(command, "%dPSET%d", axisNo_, pos);
  if ( command[0] != 0 && status) {
    status = (pC_->lowLevelWriteRead(command, response) == asynSuccess);
  }
  memset(command, 0, sizeof(command));

  /*Now set position on encoder axis.*/
               
  epicsFloat64 encRatio = 0.0;
  pC_->getDoubleParam(pC_->motorEncoderRatio_,  &encRatio);
  epicsInt32 encpos = (epicsInt32) floor((position*encRatio) + 0.5);

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, 
	    "%s: Set encoder axis %d on controller %s to position %f, encRatio: %f\n", 
	    functionName, axisNo_, pC_->portName, pos, encRatio);

  sprintf(command, "%dPESET%d", axisNo_, encpos);
  if ( command[0] != 0 && status) {
    status = (pC_->lowLevelWriteRead(command, response) == asynSuccess);
  }
  memset(command, 0, sizeof(command));
   
  /*Now do a fast update, to get the new position from the controller.*/
  bool moving = true;
  getAxisStatus(&moving);
 
  if (!status) {
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

  sprintf(command, "!%dS", axisNo_);
  status = pC_->lowLevelWriteRead(command, response);
  memset(command, 0, sizeof(command));

  deferredMove_ = 0;

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
    sprintf(command, "%dDRIVE1",  axisNo_);
  } else {
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
    asynStatus status = asynError;
    char command[P6K_MAXBUF] = {0};
    char response[P6K_MAXBUF] = {0};
    int32_t cmdStatus = 0;
    bool stat = true;
    int32_t done = 0;
    double position = 0; 
    double enc_position = 0;
    int32_t nvals = 0;
    int32_t axisNum = 0;
    int32_t axisProblemFlag = 0;
    int32_t limitsDisabledBit = 0;
    bool printErrors = true;
    int32_t intVal = 0;
    char stringVal[P6K_MAXBUF] = {0};
    bool doneMoving = false;

    static const char *functionName = "p6kAxis::getAxisStatus";
    
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
    
    //Get the time and decide if we want to print32_t errors.
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

    printf("Axis: %d\n", axisNo_);

    /* Transfer current position and encoder position.*/
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TPC);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_TPC"%d", &axisNum, &intVal);
      setDoubleParam(pC_->motorPosition_, intVal);
    }
    memset(command, 0, sizeof(command));

    printf("  Position: %d\n", intVal);

    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TPE);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      nvals = sscanf(response, "%d"P6K_CMD_TPE"%d", &axisNum, &intVal);
      setDoubleParam(pC_->motorEncoderPosition_, intVal);
    }
    memset(command, 0, sizeof(command));

    printf("  Enc Position: %d\n", intVal);

    /* Transfer axis status */
    snprintf(command, P6K_MAXBUF, "%d%s", axisNo_, P6K_CMD_TAS);
    stat = (pC_->lowLevelWriteRead(command, response) == asynSuccess) && stat;
    if (stat) {
      printf("  Status response: %s\n", response);
      nvals = sscanf(response, "%d"P6K_CMD_TAS"%s", &axisNum, stringVal);
      setDoubleParam(pC_->motorEncoderPosition_, intVal);
    }
    memset(command, 0, sizeof(command));

    printf("  Status string: %s\n", stringVal);

    if (deferredMove_) {
      doneMoving = false; 
    } else {
      doneMoving = !(stringVal[P6K_TAS_MOVING_] == P6K_TAS_ON_);
    }

    if (doneMoving) {
      if (driveType_ == P6K_SERVO_) {
	bool targetZone = (stringVal[P6K_TAS_TARGETZONE_] == P6K_TAS_ON_);
	doneMoving = targetZone && !(stringVal[P6K_TAS_TARGETTIMEOUT_] == P6K_TAS_ON_);
      }
    }

    if (!doneMoving) {
      *moving = true;
    } else {
      *moving = false;
    }

    stat = (setIntegerParam(pC_->motorStatusDone_, doneMoving) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusMoving_, (stringVal[P6K_TAS_MOVING_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusDirection_, (stringVal[P6K_TAS_DIRECTION_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusHighLimit_, (stringVal[P6K_TAS_POSLIM_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusLowLimit_, (stringVal[P6K_TAS_NEGLIM_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusHomed_, (stringVal[P6K_TAS_HOMED_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusHomed_, (stringVal[P6K_TAS_HOMED_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(pC_->motorStatusPowerOn_, (stringVal[P6K_TAS_DRIVE_] == P6K_TAS_ON_)) == asynSuccess) && stat;

    if (driveType_ == P6K_SERVO_) {
      stat = (setIntegerParam(pC_->motorStatusFollowingError_, (stringVal[P6K_TAS_POSERROR_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    } else {
      stat = (setIntegerParam(pC_->motorStatusFollowingError_, (stringVal[P6K_TAS_STALL_] == P6K_TAS_ON_)) == asynSuccess) && stat;
    }

    int problem = 0;
    problem = 
      (stringVal[P6K_TAS_DRIVEFAULT_] == P6K_TAS_ON_) |
      (stringVal[P6K_TAS_TARGETTIMEOUT_] == P6K_TAS_ON_) |
      (stringVal[P6K_TAS_POSERROR_] == P6K_TAS_ON_);
    stat = (setIntegerParam(pC_->motorStatusProblem_, (problem!=0)) == asynSuccess) && stat;

    

    /*    
    // Read all the status for this axis in one go 
    if (encoder_axis_ != 0) {
      // Encoder position comes back on a different axis 
      sprintf(command, "#%d ? P #%d P", axisNo_,  encoder_axis_);
    } else {
      // Encoder position comes back on this axis - note we initially read 
      // the following error into the position variable 
      sprintf(command, "#%d ? F P", axisNo_);
    }
    
    cmdStatus = pC_->lowLevelWriteRead(command, response);
    nvals = sscanf( response, "%6x%6x %lf %lf", &status[0], &status[1], &position, &enc_position );
	
    if ( cmdStatus || nvals != 4) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
		"drvP6kAxisGetStatus: not all status values returned. Status: %d\nCommand :%s\nResponse:%s",
		cmdStatus, command, response );
    } else {
      int32_t homeSignal = ((status[1] & pC_->P6K_STATUS2_HOME_COMPLETE) != 0);
      int32_t direction = 0;
      
      //For closed loop axes, position is actually following error up to this point
      if ( encoder_axis_ == 0 ) {
	position += enc_position;
      }
      
      position *= scale_;
      enc_position  *= scale_;
      
      setDoubleParam(pC_->motorPosition_, position);
      setDoubleParam(pC_->motorEncoderPosition_, enc_position);
      
      // Use previous position and current position to calculate direction.
      if ((position - previous_position_) > 0) {
	direction = 1;
      } else if (position - previous_position_ == 0.0) {
	direction = previous_direction_;
      } else {
	direction = 0;
      }
      setIntegerParam(pC_->motorStatusDirection_, direction);
      //Store position to calculate direction for next poll.
      previous_position_ = position;
      previous_direction_ = direction;

      if(deferredMove_) {
	done = 0; 
      } else {
	done = (((status[1] & pC_->P6K_STATUS2_IN_POSITION) != 0) || ((status[0] & pC_->P6K_STATUS1_MOTOR_ON) == 0)); 
	//If we are not done, but amp has been disabled, then set done (to stop when we get following errors).
	if ((done == 0) && ((status[0] & pC_->P6K_STATUS1_AMP_ENABLED) == 0)) {
	  done = 1;
	}
      }

      if (!done) {
	*moving = true;
      } else {
	*moving = false;
      }

      setIntegerParam(pC_->motorStatusDone_, done);
      setIntegerParam(pC_->motorStatusHighLimit_, ((status[0] & pC_->P6K_STATUS1_POS_LIMIT_SET) != 0) );
      setIntegerParam(pC_->motorStatusHomed_, homeSignal);
      //If desired_vel_zero is false && motor activated (ix00=1) && amplifier enabled, set moving=1.
      setIntegerParam(pC_->motorStatusMoving_, ((status[0] & pC_->P6K_STATUS1_DESIRED_VELOCITY_ZERO) == 0) && ((status[0] & pC_->P6K_STATUS1_MOTOR_ON) != 0) && ((status[0] & pC_->P6K_STATUS1_AMP_ENABLED) != 0) );
      setIntegerParam(pC_->motorStatusLowLimit_, ((status[0] & pC_->P6K_STATUS1_NEG_LIMIT_SET)!=0) );
      setIntegerParam(pC_->motorStatusFollowingError_,((status[1] & pC_->P6K_STATUS2_ERR_FOLLOW_ERR) != 0) );
      fatal_following_ = ((status[1] & pC_->P6K_STATUS2_ERR_FOLLOW_ERR) != 0);

      axisProblemFlag = 0;
      //Set any axis specific general problem bits.
      if ( ((status[0] & pC_->PMAX_AXIS_GENERAL_PROB1) != 0) || ((status[1] & pC_->PMAX_AXIS_GENERAL_PROB2) != 0) ) {
	axisProblemFlag = 1;
      }

      int32_t globalStatus = 0;
      int32_t feedrate_problem = 0;
      pC_->getIntegerParam(0, pC_->P6K_C_GlobalStatus_, &globalStatus);
      pC_->getIntegerParam(0, pC_->P6K_C_FeedRateProblem_, &feedrate_problem);
      if (globalStatus || feedrate_problem) {
	axisProblemFlag = 1;
      }
      //Check limits disabled bit in ix24, and if we haven't intentially disabled limits
      //	because we are homing, set the motorAxisProblem bit. Also check the limitsCheckDisable
      //	flag, which the user can set to disable this feature.
      if (!limitsCheckDisable_) {
	//Check we haven't intentially disabled limits for homing.
	if (!limitsDisabled_) {
	  sprintf(command, "i%d24", axisNo_);
	  cmdStatus = pC_->lowLevelWriteRead(command, response);
	  if (cmdStatus == asynSuccess) {
	    sscanf(response, "$%x", &limitsDisabledBit);
	    limitsDisabledBit = ((0x20000 & limitsDisabledBit) >> 17);
	    if (limitsDisabledBit) {
	      axisProblemFlag = 1;
	      if (printErrors) {
		asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "*** WARNING *** Limits are disabled on controller %s, axis %d\n", pC_->portName, axisNo_);
		printNextError_ = false;
	      }

	    }
	  }
	}
      }
      setIntegerParam(pC_->motorStatusProblem_, axisProblemFlag);
      
      //Clear error print flag for this axis if problem has been removed.
      if (axisProblemFlag == 0) {
	printNextError_ = true;
      }
            

    }


    //Set amplifier enabled bit.
    if ((status[0] & pC_->P6K_STATUS1_AMP_ENABLED) != 0) {
      amp_enabled_ = 1;
    } else {
      amp_enabled_ = 0;
    }
    setIntegerParam(pC_->motorStatusPowerOn_, amp_enabled_);
    
    */

    return asynSuccess;
}

