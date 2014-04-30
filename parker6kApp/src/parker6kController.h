/********************************************
 *  parker6kController.h 
 * 
 *  Parker 6k Asyn motor based on the 
 *  asynMotorController class.
 * 
 *  Matt Pearson
 *  26 March 2014
 * 
 ********************************************/

#ifndef parker6kController_H
#define parker6kController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "parker6kAxis.h"

#define P6K_C_FirstParamString "P6K_C_FIRSTPARAM"
#define P6K_C_LastParamString  "P6K_C_LASTPARAM"

//Controller specific parameters
#define P6K_C_GlobalStatusString    "P6K_C_GLOBALSTATUS"
#define P6K_C_CommsErrorString      "P6K_C_COMMSERROR"
#define P6K_C_CommandString         "P6K_C_COMMAND"
#define P6K_C_CommandRBVString      "P6K_C_COMMAND_RBV"
#define P6K_C_ErrorString           "P6K_C_ERROR"
#define P6K_C_TSS_SystemReadyString "P6K_C_TSS_SYSTEMREADY"
#define P6K_C_TSS_ProgRunningString "P6K_C_TSS_PROGRUNNING"
#define P6K_C_TSS_ImmediateString   "P6K_C_TSS_IMMEDIATE"
#define P6K_C_TSS_CmdErrorString    "P6K_C_TSS_CMDERROR"
#define P6K_C_TSS_MemErrorString    "P6K_C_TSS_MEMERROR"

//Axis specific parameters
#define P6K_A_DRESString       "P6K_A_DRES"
#define P6K_A_ERESString       "P6K_A_ERES"
#define P6K_A_DRIVEString      "P6K_A_DRIVE"
#define P6K_A_AXSDEFString     "P6K_A_AXSDEF"
#define P6K_A_MaxDigitsString  "P6K_A_MAXDIGITS"
#define P6K_A_CommandString    "P6K_A_COMMAND"
#define P6K_A_CommandRBVString "P6K_A_COMMAND_RBV"
#define P6K_A_LSString         "P6K_A_LS"
#define P6K_A_LHString         "P6K_A_LH"
#define P6K_A_ErrorString      "P6K_A_ERROR"

#define P6K_MAXBUF 1024

//Controller commands
#define P6K_CMD_A        "A"
#define P6K_CMD_AA       "AA"
#define P6K_CMD_AD       "AD"
#define P6K_CMD_ADA      "ADA"
#define P6K_CMD_AXSDEF   "AXSDEF"
#define P6K_CMD_COMEXC   "COMEXC"
#define P6K_CMD_D        "D"
#define P6K_CMD_DRIVE    "DRIVE"
#define P6K_CMD_DRES     "DRES"
#define P6K_CMD_ECHO     "ECHO"
#define P6K_CMD_ENCCNT   "ENCCNT"
#define P6K_CMD_ERES     "ERES"
#define P6K_CMD_GO       "GO"
#define P6K_CMD_LH       "LH"
#define P6K_CMD_LS       "LS"
#define P6K_CMD_LSNEG    "LSNEG"
#define P6K_CMD_LSPOS    "LSPOS"
#define P6K_CMD_MA       "MA"
#define P6K_CMD_PESET    "PESET"
#define P6K_CMD_PSET     "PSET"
#define P6K_CMD_S        "S"
#define P6K_CMD_TAS      "TAS"
#define P6K_CMD_TLIM     "TLIM"
#define P6K_CMD_TPC      "TPC"
#define P6K_CMD_TPE      "TPE"
#define P6K_CMD_TSS      "TSS"
#define P6K_CMD_V        "V"

class p6kController : public asynMotorController {

 public:
  p6kController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, int numAxes, double movingPollPeriod, 
		 double idlePollPeriod);

  virtual ~p6kController();

  asynStatus printConnectedStatus(void);

  /* These are the methods that we override */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus setDeferredMoves(bool deferMoves);
  virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual);
  void report(FILE *fp, int level);
  p6kAxis* getAxis(asynUser *pasynUser);
  p6kAxis* getAxis(int axisNo);
  asynStatus poll();

  //Set the open loop encoder axis
  asynStatus pk6SetOpenLoopEncoderAxis(int axis, int encoder_axis);

 protected:
  p6kAxis **pAxes_;       /**< Array of pointers to axis objects */

  #define FIRST_P6K_PARAM P6K_C_FirstParam_
  int P6K_C_FirstParam_;
  int P6K_C_GlobalStatus_;
  int P6K_C_CommsError_;
  int P6K_A_DRES_;
  int P6K_A_ERES_;
  int P6K_A_DRIVE_;
  int P6K_A_AXSDEF_;
  int P6K_A_MaxDigits_;
  int P6K_A_LS_;
  int P6K_A_LH_;
  int P6K_C_Command_;
  int P6K_A_Command_;
  int P6K_C_Command_RBV_;
  int P6K_A_Command_RBV_;
  int P6K_A_Error_;
  int P6K_C_Error_;
  int P6K_C_TSS_SystemReady_;
  int P6K_C_TSS_ProgRunning_;
  int P6K_C_TSS_Immediate_;
  int P6K_C_TSS_CmdError_;
  int P6K_C_TSS_MemError_;
  int P6K_C_LastParam_;
  #define LAST_P6K_PARAM P6K_C_LastParam_

 private:
  p6kAxis *pAxisZero;
  asynUser* lowLevelPortUser_;
  epicsUInt32 movesDeferred_;
  epicsTimeStamp nowTime_;
  epicsFloat64 nowTimeSecs_;
  epicsFloat64 lastTimeSecs_;
  bool printNextError_;
  asynStatus lowLevelWriteRead(const char *command, char *response);
  asynStatus trimResponse(char *input, char *output);
  asynStatus errorResponse(char *input, char *output);
  asynStatus lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, const char *inputEos, const char *outputEos);

  //asynStatus processDeferredMoves(void);

  //static class data members

  static const epicsUInt32 P6K_MAXBUF_;
  static const epicsUInt32 P6K_MAXAXES_;
  static const epicsFloat64 P6K_TIMEOUT_;
  static const epicsUInt32 P6K_FORCED_FAST_POLLS_;
  static const epicsUInt32 P6K_OK_;
  static const epicsUInt32 P6K_ERROR_;
  static const epicsUInt32 P6K_ERROR_PRINT_TIME_;

  static const char * P6K_ASYN_IEOS_;
  static const char * P6K_ASYN_OEOS_;

  static const char P6K_ON_;
  static const char P6K_OFF_;

  static const epicsUInt32 P6K_TSS_SYSTEMREADY_;
  static const epicsUInt32 P6K_TSS_PROGRUNNING_;
  static const epicsUInt32 P6K_TSS_IMMEDIATE_;
  static const epicsUInt32 P6K_TSS_CMDERROR_;
  static const epicsUInt32 P6K_TSS_MEMERROR_;

  friend class p6kAxis;

};

#define NUM_P6K_PARAMS (&LAST_P6K_PARAM - &FIRST_P6K_PARAM + 1)

#endif /* parker6kController_H */
