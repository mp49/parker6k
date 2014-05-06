/********************************************
 *  p6kController.cpp
 * 
 *  P6K Asyn motor based on the 
 *  asynMotorController class.
 * 
 *  Matt Pearson
 *  26 March 2014
 * 
 ********************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <iostream>
using std::cout;
using std::endl;
using std::dec;

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>

#include "asynOctetSyncIO.h"

#include "parker6kController.h"

static const char *driverName = "parker6k";

const epicsUInt32 p6kController::P6K_MAXBUF_ = P6K_MAXBUF;
const epicsUInt32 p6kController::P6K_MAXAXES_ = 8;
const epicsFloat64 p6kController::P6K_TIMEOUT_ = 5.0;
const epicsUInt32 p6kController::P6K_ERROR_PRINT_TIME_ = 1; //seconds (this should be set larger when we finish debugging)
const epicsUInt32 p6kController::P6K_FORCED_FAST_POLLS_ = 10;
const epicsUInt32 p6kController::P6K_OK_ = 0;
const epicsUInt32 p6kController::P6K_ERROR_ = 1;

const char * p6kController::P6K_ASYN_IEOS_ = ">";
const char * p6kController::P6K_ASYN_OEOS_ = "\n";

const char p6kController::P6K_ON_       = '1';
const char p6kController::P6K_OFF_      = '0';

/* TSS Status Bits (position in char array, not TSS bit position) */
const epicsUInt32 p6kController::P6K_TSS_SYSTEMREADY_ = 0;
const epicsUInt32 p6kController::P6K_TSS_PROGRUNNING_ = 2;
const epicsUInt32 p6kController::P6K_TSS_IMMEDIATE_   = 3;
const epicsUInt32 p6kController::P6K_TSS_CMDERROR_    = 12;
const epicsUInt32 p6kController::P6K_TSS_MEMERROR_    = 26;

//C function prototypes, for the functions that can be called on IOC shell.
//Some of these functions are provided to ease transition to the model 3 driver. Some of these
//functions could be handled by the parameter library.
extern "C" {
  asynStatus p6kCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
					 int numAxes, int movingPollPeriod, int idlePollPeriod);
  
  asynStatus p6kCreateAxis(const char *p6kName, int axis);

  asynStatus p6kCreateAxes(const char *p6kName, int numAxes);
  
   
}

/**
 * p6kController constructor.
 * @param portName The Asyn port name to use (that the motor record connects to).
 * @param lowLevelPortName The name of the low level port that has already been created, to enable comms to the controller.
 * @param lowLevelPortAddress The asyn address for the low level port
 * @param numAxes The number of axes on the controller (1 based)
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are moving
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are idle
 */
p6kController::p6kController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
			       int numAxes, double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes+1, NUM_MOTOR_DRIVER_PARAMS,
			0, // No additional interfaces
			0, // No addition interrupt interfaces
			ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
			1, // autoconnect
			0, 0)  // Default priority and stack size
{
  static const char *functionName = "p6kController::p6kController";

  printf("%s: Constructor.\n", functionName);

  //Initialize non static data members
  lowLevelPortUser_ = NULL;
  movesDeferred_ = 0;
  nowTimeSecs_ = 0.0;
  lastTimeSecs_ = 0.0;
  printNextError_ = false;

  pAxes_ = (p6kAxis **)(asynMotorController::pAxes_);

  //Create controller-specific parameters
  printf("%s: Create controller parameters.\n", functionName);
  createParam(P6K_C_FirstParamString,       asynParamInt32, &P6K_C_FirstParam_);
  createParam(P6K_C_GlobalStatusString,     asynParamInt32, &P6K_C_GlobalStatus_);
  createParam(P6K_C_CommsErrorString,       asynParamInt32, &P6K_C_CommsError_);
  createParam(P6K_C_CommandString,          asynParamOctet, &P6K_C_Command_);
  createParam(P6K_C_CommandRBVString,       asynParamOctet, &P6K_C_Command_RBV_);
  createParam(P6K_C_ErrorString,            asynParamOctet, &P6K_C_Error_);
  createParam(P6K_C_TSS_SystemReadyString,  asynParamInt32, &P6K_C_TSS_SystemReady_);
  createParam(P6K_C_TSS_ProgRunningString,  asynParamInt32, &P6K_C_TSS_ProgRunning_);
  createParam(P6K_C_TSS_ImmediateString,    asynParamInt32, &P6K_C_TSS_Immediate_);
  createParam(P6K_C_TSS_CmdErrorString,     asynParamInt32, &P6K_C_TSS_CmdError_);
  createParam(P6K_C_TSS_MemErrorString,     asynParamInt32, &P6K_C_TSS_MemError_);
  createParam(P6K_C_LastParamString,        asynParamInt32, &P6K_C_LastParam_);

  //Create axis specific parameters
  //createParam adds the parameters to all param lists automatically (using maxAddr).
  printf("%s: Create axis parameters.\n", functionName);
  createParam(P6K_A_DRESString,             asynParamInt32, &P6K_A_DRES_);
  createParam(P6K_A_ERESString,             asynParamInt32, &P6K_A_ERES_);
  createParam(P6K_A_DRIVEString,            asynParamInt32, &P6K_A_DRIVE_);
  createParam(P6K_A_AXSDEFString,           asynParamInt32, &P6K_A_AXSDEF_);
  createParam(P6K_A_MaxDigitsString,        asynParamInt32, &P6K_A_MaxDigits_);
  createParam(P6K_A_LSString,               asynParamInt32, &P6K_A_LS_);
  createParam(P6K_A_LHString,               asynParamInt32, &P6K_A_LH_);
  createParam(P6K_A_CommandString,          asynParamOctet, &P6K_A_Command_);
  createParam(P6K_A_CommandRBVString,       asynParamOctet, &P6K_A_Command_RBV_);
  createParam(P6K_A_ErrorString,            asynParamOctet, &P6K_A_Error_);
  createParam(P6K_A_DelayTimeString,        asynParamFloat64, &P6K_A_DelayTime_);
  createParam(P6K_A_AutoDriveEnableString,  asynParamInt32, &P6K_A_AutoDriveEnable_);

  //Create dummy axis for asyn address 0. This is used for controller parameters.
  printf("%s: Create pAxisZero for controller parameters.\n", functionName);
  pAxisZero = new p6kAxis(this, 0);
  
  //Connect our Asyn user to the low level port that is a parameter to this constructor
  //NOTE:
  // The P6K will send back a command with a \r\r\n> \n>
  // The low level port EOS will remove the first >. 
  // We will need to deal with the rest in p6kController::lowLevelWriteRead
  // Error responses are handled differently, and unfortunately rely on a asyn timeout.
  printf("%s: Connect to low level Asyn port.\n", functionName);
  //const char * ieos = P6K_ASYN_IEOS_;
  //const char * oeos = P6K_ASYN_OEOS_;
  if (lowLevelPortConnect(lowLevelPortName, lowLevelPortAddress, &lowLevelPortUser_, P6K_ASYN_IEOS_, P6K_ASYN_OEOS_) != asynSuccess) {
    printf("%s: Failed to connect to low level asynOctetSyncIO port %s\n", functionName, lowLevelPortName);
    setIntegerParam(P6K_C_CommsError_, P6K_ERROR_);
  } else {
    setIntegerParam(P6K_C_CommsError_, P6K_OK_);
  }

  char command[P6K_MAXBUF_] = {0};
  char response[P6K_MAXBUF_] = {0};

  //Disable command echo
  snprintf(command, P6K_MAXBUF_, "%s0", P6K_CMD_ECHO);
  if (lowLevelWriteRead(command, response) != asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Turning off %s failed.\n", functionName, P6K_CMD_ECHO);
  } else {

    memset(command, 0, sizeof(command));
    //Enable continuous command execution mode
    snprintf(command, P6K_MAXBUF_, "%s1", P6K_CMD_COMEXC);
    if (lowLevelWriteRead(command, response) != asynSuccess) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: Continuous command execution mode (%s) failed.\n", functionName, P6K_CMD_COMEXC);
    }
    
    startPoller(movingPollPeriod, idlePollPeriod, P6K_FORCED_FAST_POLLS_);

    bool paramStatus = true;
    paramStatus = ((setIntegerParam(P6K_C_GlobalStatus_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setStringParam(P6K_C_Command_, " ") == asynSuccess) && paramStatus);
    paramStatus = ((setStringParam(P6K_C_Command_RBV_, " ") == asynSuccess) && paramStatus);
    paramStatus = ((setStringParam(P6K_C_Error_, " ") == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(P6K_C_TSS_SystemReady_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(P6K_C_TSS_ProgRunning_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(P6K_C_TSS_Immediate_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(P6K_C_TSS_CmdError_, 0) == asynSuccess) && paramStatus);
    paramStatus = ((setIntegerParam(P6K_C_TSS_MemError_, 0) == asynSuccess) && paramStatus);

    callParamCallbacks();

    if (!paramStatus) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s Unable To Set Driver Parameters In Constructor.\n", functionName);
    }

  }
 
}


p6kController::~p6kController(void) 
{
  //Destructor. Should never get here.
  delete pAxisZero;
}


/**
 * Connect to the underlying low level Asyn port that is used for comms.
 * This uses the asynOctetSyncIO interface, and also sets the input and output terminators.
 * @param port The port to connect to
 * @param addr The address of the port to connect to
 * @param ppasynUser A pointer to the pasynUser structure used by the controller
 * @param inputEos The input EOS character
 * @param outputEos The output EOS character
 * @return asynStatus  
 */
asynStatus p6kController::lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, const char *inputEos, const char *outputEos)
{
  asynStatus status = asynSuccess;
 
  static const char *functionName = "p6kController::lowLevelPortConnect";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "p6kController::motorAxisAsynConnect: unable to connect to port %s\n", 
	      port);
    return status;
  }

  //Do I want to disconnect below? If the IP address comes up, will the driver recover
  //if the poller functions are running? Might have to use asynManager->isConnected to
  //test connection status of low level port (in the pollers). But then autosave 
  //restore doesn't work (and we would save wrong positions). So I need to 
  //have a seperate function(s) to deal with connecting after IOC init.

  status = pasynOctetSyncIO->setInputEos(*ppasynUser, inputEos, strlen(inputEos) );
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "p6kController: unable to set input EOS on %s: %s\n", 
	      port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }
  
  status = pasynOctetSyncIO->setOutputEos(*ppasynUser, outputEos, strlen(outputEos));
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
	      "p6kController: unable to set output EOS on %s: %s\n", 
	      port, (*ppasynUser)->errorMessage);
    pasynOctetSyncIO->disconnect(*ppasynUser);
    //Set my low level pasynUser pointer to NULL
    *ppasynUser = NULL;
    return status;
  }
  
  return status;
}

/**
 * Utilty function to print the connected status of the low level asyn port.
 * @return asynStatus
 */
asynStatus p6kController::printConnectedStatus()
{
  asynStatus status = asynSuccess;
  int32_t asynManagerConnected = 0;
  static const char *functionName = "p6kController::printConnectedStatus";
  
  if (lowLevelPortUser_) {
    status = pasynManager->isConnected(lowLevelPortUser_, &asynManagerConnected);
      if (status!=asynSuccess) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
		"p6kController: Error calling pasynManager::isConnected.\n");
      return status;
      } else {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s isConnected: %d\n", functionName, asynManagerConnected);
    }
  }
  return status;
}

/**
 * Wrapper for asynOctetSyncIO write/read functions.
 * @param command - String command to send.
 * @response response - String response back.
 */
asynStatus p6kController::lowLevelWriteRead(const char *command, char *response)
{
  asynStatus status = asynSuccess;
  int32_t eomReason = 0;
  size_t nwrite = 0;
  size_t nread = 0;
  int32_t commsError = 0;
  char temp[P6K_MAXBUF_] = {0};
  static const char *functionName = "p6kController::lowLevelWriteRead";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);
  
  if (!lowLevelPortUser_) {
    setIntegerParam(P6K_C_CommsError_, P6K_ERROR_);
    return asynError;
  }
  
  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: command: %s\n", functionName, command);
  
  //Make sure the low level port is connected before we attempt comms
  //Use the controller-wide param P6K_C_CommsError_
  getIntegerParam(P6K_C_CommsError_, &commsError);

  memset(response, 0, sizeof(response));
  
  status = pasynOctetSyncIO->writeRead(lowLevelPortUser_ ,
				       command, strlen(command),
				       temp, P6K_MAXBUF_,
				       P6K_TIMEOUT_,
				       &nwrite, &nread, &eomReason );
  
  if (status) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, "%s: Error from pasynOctetSyncIO->writeRead. command: %s\n", functionName, command);
    setIntegerParam(P6K_C_CommsError_, P6K_ERROR_);
  } else {
    setIntegerParam(P6K_C_CommsError_, P6K_OK_);
  }

  //Search for an error response
  status = errorResponse(temp, response);
  if (status == asynSuccess) {
    asynPrint(lowLevelPortUser_, ASYN_TRACE_ERROR, 
	      "%s: ERROR: Command %s returned an error: %s\n", functionName, command, response);
    status = asynError;
  } else {
    //Deal with a successful command
    //The P6K will send back a command with a \r\r\n> \n>
    //The low level port asyn EOS will remove the first >
    //We deal with the rest in this function
    status = trimResponse(temp, response);
  }  

  asynPrint(lowLevelPortUser_, ASYN_TRACEIO_DRIVER, "%s: response: %s\n", functionName, response); 
  
  return status;
}

/**
 * The P6K will send back an error string with a ? prompt afterwards it.
 * We search for this before dealing with a successful command. An error
 * response would have caused an Asyn timeout, because there was no standard
 * IEOS character and by defaut we search for the character that terminates a
 * successful command.
 * @param input - input buffer. This will be modified.
 * @param output - output buffer. No bigger than P6K_MAXBUF_.
 */
asynStatus p6kController::errorResponse(char *input, char *output)
{
  static const char *trailer = "?";
  static const char *header = "*";
  
  static const char *functionName = "p6kController::errorResponse";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  char *pTrailer = strstr(input, trailer);

  if (pTrailer != NULL) {
    *pTrailer = '\0';
    //Remove leading '*' character. Make sure it's there first.
    //For error strings there may be some leading chars before the '*'
    char *pHeader = strstr(input, header);
    if (pHeader != NULL) {
      pHeader++;
      if (output != NULL) {
	strncpy(output, pHeader, P6K_MAXBUF_-1);
      }
      return asynSuccess;
    }
  } 

  //asynError is used to indicate we have not found an error
  return asynError;
}

/**
 * Remove a \r\r\n from an input buffer.
 * Also remove leading '*' character. 
 * @param input - input buffer. This will be modified.
 * @param output - output buffer. No bigger than P6K_MAXBUF_.
 */
asynStatus p6kController::trimResponse(char *input, char *output)
{
  asynStatus status = asynSuccess;
  static const char *trailer = "\r\r\n";
  static const char *smallTrailer = "\r\n";
  static const char *header = "*";
  static const char *functionName = "p6kController::trimResponse";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  char *pTrailer = strstr(input, trailer);
  if (pTrailer != NULL) {
    *pTrailer = '\0';
  } else {
    //Try the smallTrailer instead
    pTrailer = strstr(input, smallTrailer);
    if (pTrailer != NULL) {
      *pTrailer = '\0';
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s Could not find correct trailer.\n", functionName);
      status = asynError;
    }
  }

  //Remove leading '*' character. Make sure it's there first.
  //Occasionally there may be some leading chars before the '*', eg a space.
  char *pHeader = strstr(input, header);
  if (pHeader != NULL) {
    pHeader++;
    if (output != NULL) {
      strncpy(output, pHeader, P6K_MAXBUF_-1);
    }
  }

  return status;
}


void p6kController::report(FILE *fp, int level)
{
  int32_t axis = 0;
  p6kAxis *pAxis = NULL;

  fprintf(fp, "p6k motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      if (!pAxis) continue;
      fprintf(fp, "  axis %d\n", 
              pAxis->axisNo_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/**
 * Deal with controller specific epicsFloat64 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus p6kController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  bool status = true;
  p6kAxis *pAxis = NULL;
  char command[P6K_MAXBUF_] = {0};
  char response[P6K_MAXBUF_] = {0};
	
  static const char *functionName = "p6kController::writeFloat64";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  }

  /* Set the parameter and readback in the parameter library. */
  status = (pAxis->setDoubleParam(function, value) == asynSuccess) && status;

  if (function == P6K_A_DelayTime_) {
    cout << "Setting delay time to " << value << endl;
  }

  //if (command[0] != 0 && status) {
  //  status = (lowLevelWriteRead(command, response) == asynSuccess) && status;
  //}

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeFloat64(pasynUser, value) == asynSuccess) && status;

  if (!status) {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, P6K_ERROR_);
    return asynError;
  } else {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, P6K_OK_);
  }

  return asynSuccess;

}

/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus p6kController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  //char command[P6K_MAXBUF_] = {0};
  //char response[P6K_MAXBUF_] = {0};
  bool status = true;
  p6kAxis *pAxis = NULL;
  static const char *functionName = "p6kController::writeInt32";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  pAxis = this->getAxis(pasynUser);
  if (!pAxis) {
    return asynError;
  } 

  status = (pAxis->setIntegerParam(function, value) == asynSuccess) && status;

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = (asynMotorController::writeInt32(pasynUser, value) == asynSuccess) && status;
  
  if (!status) {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, P6K_ERROR_);
    return asynError;
  } else {
    setIntegerParam(pAxis->axisNo_, this->motorStatusCommsError_, P6K_OK_);
  }

  return asynSuccess;

}


asynStatus p6kController::writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    p6kAxis *pAxis = NULL;
    char command[P6K_MAXBUF_] = {0};
    char response[P6K_MAXBUF_] = {0};
    const char *functionName = "parker6kController::writeOctet";

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s.\n", functionName);

    pAxis = this->getAxis(pasynUser);
    if (!pAxis) {
      return asynError;
    } 
    
    if (function == P6K_C_Command_) {
      //Send command to controller
      snprintf(command, P6K_MAXBUF_, "%s", value);
      if (lowLevelWriteRead(command, response) != asynSuccess) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: Command %s failed.\n", functionName, command);
      } else {
	setStringParam(P6K_C_Command_RBV_, response);
      }
    } else if (function == P6K_A_Command_) {
      //Send axis specific command to controller. This supports the 
      //primitive commands PREM and POST.?
      //This adds on the axis number to the command
      snprintf(command, P6K_MAXBUF_, "%d%s", pAxis->axisNo_, value);
      if (lowLevelWriteRead(command, response) != asynSuccess) {
	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		  "%s: Command %s failed for axis %d.\n", functionName, command, pAxis->axisNo_);
      } else {
	setStringParam(P6K_A_Command_RBV_, response);
      }
    } else {
      status = asynMotorController::writeOctet(pasynUser, value, nChars, nActual);
    }

    if (status != asynSuccess) {
      callParamCallbacks();
      return asynError;
    }
    
    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);
    /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status!=asynSuccess) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
              "%s Error Setting Parameter. asynUser->reason: %d\n", 
              functionName, function);
    }

    *nActual = nChars;
    return status;
}


/** Returns a pointer to an p6kAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
p6kAxis* p6kController::getAxis(asynUser *pasynUser)
{
  int32_t axisNo = 0;
    
  getAddress(pasynUser, &axisNo);
  return getAxis(axisNo);
}



/** Returns a pointer to an p6kAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
p6kAxis* p6kController::getAxis(int axisNo)
{
  if ((axisNo < 0) || (axisNo >= numAxes_)) return NULL;
  return pAxes_[axisNo];
}


/** 
 * Polls the controller, rather than individual axis.
 * @return asynStatus
 */
asynStatus p6kController::poll()
{
  char command[P6K_MAXBUF] = {0};
  char response[P6K_MAXBUF] = {0};
  bool stat = true;
  int32_t nvals = 0;
  bool printErrors = 0;
  int32_t intVal = 0;
  char stringVal[P6K_MAXBUF] = {0};
  static const char *functionName = "p6kController::poll";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (!lowLevelPortUser_) {
    return asynError;
  }

  /* Get the time and decide if we want to print errors.*/
  epicsTimeGetCurrent(&nowTime_);
  nowTimeSecs_ = nowTime_.secPastEpoch;
  if ((nowTimeSecs_ - lastTimeSecs_) < P6K_ERROR_PRINT_TIME_) {
    printErrors = 0;
  } else {
    printErrors = 1;
    lastTimeSecs_ = nowTimeSecs_;
  }

  if (printNextError_) {
    printErrors = 1;
  }
  
  //Set any controller specific parameters. 
  //Some of these may be used by the axis poll to set axis problem bits.

  //NOTE: do we have to use TLIM to monitor limit status?
  //See: https://trac.sns.gov/slowcontrols/ticket/125
  //Can't read back TLIM for one axis, only all axes at once.
  //So it's a 'controller' command.
  
  //Transfer system status
  snprintf(command, P6K_MAXBUF, "%s", P6K_CMD_TSS);
  stat = (lowLevelWriteRead(command, response) == asynSuccess) && stat;
  if (stat) {
    //printf("  Status response: %s\n", response);
    nvals = sscanf(response, P6K_CMD_TSS"%s", stringVal);
  }
  memset(command, 0, sizeof(command));
  
  //printf("  Status string: %s\n", stringVal);
   
  if (stat) {
    stat = (setIntegerParam(P6K_C_TSS_SystemReady_, (stringVal[P6K_TSS_SYSTEMREADY_] == P6K_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(P6K_C_TSS_ProgRunning_, (stringVal[P6K_TSS_PROGRUNNING_] == P6K_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(P6K_C_TSS_Immediate_,   (stringVal[P6K_TSS_IMMEDIATE_]   == P6K_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(P6K_C_TSS_CmdError_,    (stringVal[P6K_TSS_CMDERROR_]    == P6K_ON_)) == asynSuccess) && stat;
    stat = (setIntegerParam(P6K_C_TSS_MemError_,    (stringVal[P6K_TSS_MEMERROR_]    == P6K_ON_)) == asynSuccess) && stat;
  }
  
  callParamCallbacks();

  if (!stat) {
    if (printErrors) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		"ERROR: Problem reading status on controller %s\n", 
		this->portName);
    }
    setIntegerParam(P6K_C_CommsError_, P6K_ERROR_);
    printNextError_ = false;
    return asynError;
  } else {
    setIntegerParam(P6K_C_CommsError_, P6K_OK_);
    printNextError_ = true;
    return asynSuccess;
  }
}




/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * @param axis Axis number to disable the check for.
 */
/*asynStatus p6kController::p6kDisableLimitsCheck(int axis) 
{
  p6kAxis *pA = NULL;
  static const char *functionName = "p6kController::p6kDisableLimitsCheck";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    pA->limitsCheckDisable_ = 1;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Disabling hardware limits disable check on controller %s, axis %d\n", 
              functionName, portName, pA->axisNo_);
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using p6kCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
  }*/

/**
 * Disable the check in the axis poller that reads ix24 to check if hardware limits
 * are disabled. By default this is enabled for safety reasons. It sets the motor
 * record PROBLEM bit in MSTA, which results in the record going into MAJOR/STATE alarm.
 * This function will disable the check for all axes on this controller.
 */
 /*asynStatus p6kController::p6kDisableLimitsCheck(void) 
{
  p6kAxis *pA = NULL;
  static const char *functionName = "p6kController::p6kDisableLimitsCheck";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  for (int i=0; i<numAxes_; i++) {
    pA = getAxis(i);
    if (!pA) continue;
    pA->limitsCheckDisable_ = 1;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Disabling hardware limits disable check on controller %s, axis %d\n", 
              functionName, portName, pA->axisNo_);
  }
  this->unlock();
  return asynSuccess;
  }*/


/**
 * Set the P6K axis scale factor to increase resolution in the motor record.
 * Default value is 1.
 * @param axis Axis number to set the P6K axis scale factor.
 * @param scale Scale factor to set
 */
  /*asynStatus p6kController::p6kSetAxisScale(int axis, int scale) 
{
  p6kAxis *pA = NULL;
  static const char *functionName = "p6kController::p6kSetAxisScale";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  if (scale < 1) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: Error: scale factor must be >=1.\n", functionName);
    return asynError;
  }

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    pA->scale_ = scale;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting scale factor of &d on axis %d, on controller %s.\n", 
              functionName, pA->scale_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using p6kCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
  }*/


/**
 * If we have an open loop axis that has an encoder coming back on a different channel
 * then the encoder readback axis number can be set here. This ensures that the encoder
 * will be used for the position readback. It will also ensure that the encoder axis
 * is set correctly when performing a set position on the open loop axis.
 *
 * To use this function, the axis number used for the encoder must have been configured
 * already using p6kCreateAxis.
 *
 * @param controller The Asyn port name for the P6K controller.
 * @param axis Axis number to set the P6K axis scale factor.
 * @param encoder_axis The axis number that the encoder is fed into.  
 */
   /*asynStatus p6kController::p6kSetOpenLoopEncoderAxis(int axis, int encoder_axis)
{
  p6kAxis *pA = NULL;
  static const char *functionName = "p6kController::p6kSetOpenLoopEncoderAxis";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  this->lock();
  pA = getAxis(axis);
  if (pA) {
    //Test that the encoder axis has also been configured
    if (getAxis(encoder_axis) == NULL) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
		"%s: Error: encoder axis %d has not been configured using p6kCreateAxis.\n", functionName, encoder_axis);
      return asynError;
    }
    pA->encoder_axis_ = encoder_axis;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "%s. Setting encoder axis &d for axis %d, on controller %s.\n", 
              functionName, pA->encoder_axis_, pA->axisNo_, portName);

  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s: Error: axis %d has not been configured using p6kCreateAxis.\n", functionName, axis);
    return asynError;
  }
  this->unlock();
  return asynSuccess;
  }*/


asynStatus p6kController::setDeferredMoves(bool deferMoves)
{
  asynStatus status = asynSuccess;
  bool stat = true;
  char command[P6K_MAXBUF_] = {0};
  char response[P6K_MAXBUF_] = {0};
  uint32_t move[P6K_MAXAXES_+1] = {0};
  p6kAxis *pAxis = NULL;
  static const char *functionName = "p6kController::setDeferredMoves";

  cout << functionName << endl;

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s\n", functionName);

  //If we are not ending deferred moves then return
  if (deferMoves || !movesDeferred_) {
    movesDeferred_ = true;
    return asynSuccess;
  }

  //Set the distance to move for each axis
  for (int32_t axis=0; axis<numAxes_; ++axis) {
    pAxis = getAxis(axis);
    if (pAxis != NULL) {
      if (pAxis->deferredMove_) {
	cout << "axis: " << axis << endl;
	cout << "position: " << pAxis->deferredPosition_ << endl;
	snprintf(command, P6K_MAXBUF, "%dD%d", pAxis->axisNo_, pAxis->deferredPosition_);
	cout << "command: " << command << endl;
	stat = (lowLevelWriteRead(command, response) == asynSuccess) && stat;
	if (axis <= P6K_MAXAXES_) {
	  move[axis] = 1;
	}
	memset(command, 0, sizeof(command));
      }
    }
  }

  //If any commands failed, don't execute, cancle deferred move and return
  if (!stat) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
	      "%s ERROR Sending Deferred Move Positions.\n", functionName);
    status = asynError;
  } else {
  
    //Execute the deferred move
    snprintf(command, P6K_MAXBUF, "GO%d%d%d%d%d%d%d%d", move[1],move[2],move[3],move[4],move[5],move[6],move[7],move[8]);
    cout << "command: " << command << endl;
    if (lowLevelWriteRead(command, response) != asynSuccess) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s ERROR Sending Deferred Move Command.\n", functionName);
      setStringParam(P6K_C_Error_, "ERROR: Deferred Move Failed");
      status = asynError;
    } else {
      setStringParam(P6K_C_Error_, " ");
      status = asynSuccess;
    }
    
  }

  //Clear deferred move flag for the axes involved.
  for (int32_t axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (pAxis!=NULL) {
      if (pAxis->deferredMove_) {
	pAxis->deferredMove_ = 0;
      }
    }
  }

  movesDeferred_ = false;
     
  return status;
}





/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

/**
 * C wrapper for the p6kController constructor.
 * See p6kController::p6kController.
 *
 */
asynStatus p6kCreateController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, 
				int numAxes, int movingPollPeriod, int idlePollPeriod)
{

    p6kController *pp6kController
      = new p6kController(portName, lowLevelPortName, lowLevelPortAddress, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    pp6kController = NULL;

    return asynSuccess;
}

/**
 * C wrapper for the p6kAxis constructor.
 * See p6kAxis::p6kAxis.
 *
 */
asynStatus p6kCreateAxis(const char *p6kName,         /* specify which controller by port name */
			  int axis)                    /* axis number (start from 1). */
{
  p6kController *pC;
  p6kAxis *pAxis;

  static const char *functionName = "p6kCreateAxis";

  pC = (p6kController*) findAsynPortDriver(p6kName);
  if (!pC) {
    printf("%s::%s: ERROR Port %s Not Found.\n",
           driverName, functionName, p6kName);
    return asynError;
  }

  if (axis == 0) {
    printf("%s::%s: ERROR Axis Number 0 Not Allowed. This Asyn Address Is Reserved For Controller Specific Parameters.\n",
	   driverName, functionName);
    return asynError;
  }
  
  pC->lock();
  pAxis = new p6kAxis(pC, axis);
  pAxis = NULL;
  pC->unlock();
  return asynSuccess;
}

/**
 * C Wrapper function for p6kAxis constructor.
 * See p6kAxis::p6kAxis.
 * This function allows creation of multiple p6kAxis objects with axis numbers 1 to numAxes.
 * @param p6kName Asyn port name for the controller (const char *)
 * @param numAxes The number of axes to create, starting at 1.
 *
 */
asynStatus p6kCreateAxes(const char *p6kName,        
			  int numAxes)                   
{
  p6kController *pC;
  p6kAxis *pAxis;

  static const char *functionName = "p6kCreateAxis";

  pC = (p6kController*) findAsynPortDriver(p6kName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, p6kName);
    return asynError;
  }
  
  pC->lock();
  for (int axis=1; axis<=numAxes; axis++) {
    pAxis = new p6kAxis(pC, axis);
    pAxis = NULL;
  }
  pC->unlock();
  return asynSuccess;
}



/* Code for iocsh registration */

/* p6kCreateController */
static const iocshArg p6kCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kCreateControllerArg1 = {"Low level port name", iocshArgString};
static const iocshArg p6kCreateControllerArg2 = {"Low level port address", iocshArgInt};
static const iocshArg p6kCreateControllerArg3 = {"Number of axes", iocshArgInt};
static const iocshArg p6kCreateControllerArg4 = {"Moving poll rate (ms)", iocshArgInt};
static const iocshArg p6kCreateControllerArg5 = {"Idle poll rate (ms)", iocshArgInt};
static const iocshArg * const p6kCreateControllerArgs[] = {&p6kCreateControllerArg0,
							    &p6kCreateControllerArg1,
							    &p6kCreateControllerArg2,
							    &p6kCreateControllerArg3,
							    &p6kCreateControllerArg4,
							    &p6kCreateControllerArg5};
static const iocshFuncDef configp6kCreateController = {"p6kCreateController", 6, p6kCreateControllerArgs};
static void configp6kCreateControllerCallFunc(const iocshArgBuf *args)
{
  p6kCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}


/* p6kCreateAxis */
static const iocshArg p6kCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg * const p6kCreateAxisArgs[] = {&p6kCreateAxisArg0,
                                                     &p6kCreateAxisArg1};
static const iocshFuncDef configp6kAxis = {"p6kCreateAxis", 2, p6kCreateAxisArgs};

static void configp6kAxisCallFunc(const iocshArgBuf *args)
{
  p6kCreateAxis(args[0].sval, args[1].ival);
}

/* p6kCreateAxes */
static const iocshArg p6kCreateAxesArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kCreateAxesArg1 = {"Num Axes", iocshArgInt};
static const iocshArg * const p6kCreateAxesArgs[] = {&p6kCreateAxesArg0,
                                                     &p6kCreateAxesArg1};
static const iocshFuncDef configp6kAxes = {"p6kCreateAxes", 2, p6kCreateAxesArgs};

static void configp6kAxesCallFunc(const iocshArgBuf *args)
{
  p6kCreateAxes(args[0].sval, args[1].ival);
}


/* p6kDisableLimitsCheck */
/*static const iocshArg p6kDisableLimitsCheckArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kDisableLimitsCheckArg1 = {"Axis number", iocshArgInt};
static const iocshArg p6kDisableLimitsCheckArg2 = {"All Axes", iocshArgInt};
static const iocshArg * const p6kDisableLimitsCheckArgs[] = {&p6kDisableLimitsCheckArg0,
							      &p6kDisableLimitsCheckArg1,
							      &p6kDisableLimitsCheckArg2};
static const iocshFuncDef configp6kDisableLimitsCheck = {"p6kDisableLimitsCheck", 3, p6kDisableLimitsCheckArgs};

static void configp6kDisableLimitsCheckCallFunc(const iocshArgBuf *args)
{
  p6kDisableLimitsCheck(args[0].sval, args[1].ival, args[2].ival);
  }*/



/* p6kSetAxisScale */
/*static const iocshArg p6kSetAxisScaleArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kSetAxisScaleArg1 = {"Axis number", iocshArgInt};
static const iocshArg p6kSetAxisScaleArg2 = {"Scale", iocshArgInt};
static const iocshArg * const p6kSetAxisScaleArgs[] = {&p6kSetAxisScaleArg0,
							      &p6kSetAxisScaleArg1,
							      &p6kSetAxisScaleArg2};
static const iocshFuncDef configp6kSetAxisScale = {"p6kSetAxisScale", 3, p6kSetAxisScaleArgs};

static void configp6kSetAxisScaleCallFunc(const iocshArgBuf *args)
{
  p6kSetAxisScale(args[0].sval, args[1].ival, args[2].ival);
  }*/

/* p6kSetOpenLoopEncoderAxis */
/*static const iocshArg p6kSetOpenLoopEncoderAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg p6kSetOpenLoopEncoderAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg p6kSetOpenLoopEncoderAxisArg2 = {"Encoder Axis", iocshArgInt};
static const iocshArg * const p6kSetOpenLoopEncoderAxisArgs[] = {&p6kSetOpenLoopEncoderAxisArg0,
								  &p6kSetOpenLoopEncoderAxisArg1,
								  &p6kSetOpenLoopEncoderAxisArg2};
static const iocshFuncDef configp6kSetOpenLoopEncoderAxis = {"p6kSetOpenLoopEncoderAxis", 3, p6kSetOpenLoopEncoderAxisArgs};

static void configp6kSetOpenLoopEncoderAxisCallFunc(const iocshArgBuf *args)
{
  p6kSetOpenLoopEncoderAxis(args[0].sval, args[1].ival, args[2].ival);
  }*/


static void p6kControllerRegister(void)
{
  iocshRegister(&configp6kCreateController,   configp6kCreateControllerCallFunc);
  iocshRegister(&configp6kAxis,               configp6kAxisCallFunc);
  iocshRegister(&configp6kAxes,               configp6kAxesCallFunc);
  //  iocshRegister(&configp6kDisableLimitsCheck, configp6kDisableLimitsCheckCallFunc);
  //  iocshRegister(&configp6kSetAxisScale, configp6kSetAxisScaleCallFunc);
  // iocshRegister(&configp6kSetOpenLoopEncoderAxis, configp6kSetOpenLoopEncoderAxisCallFunc);
}
epicsExportRegistrar(p6kControllerRegister);


} // extern "C"

