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
#define P6K_C_LastParamString "P6K_C_LASTPARAM"

//Controller specific parameters
#define P6K_C_GlobalStatusString "P6K_C_GLOBALSTATUS"
#define P6K_C_CommsErrorString "P6K_C_COMMSERROR"
#define P6K_C_CommandString "P6K_C_COMMAND"

//Axis specific parameters
#define P6K_A_DRESString  "P6K_A_DRES"
#define P6K_A_ERESString  "P6K_A_ERES"
#define P6K_A_DRIVEString "P6K_A_DRIVE"
#define P6K_A_MaxDigitsString "P6K_A_MAXDIGITS"
#define P6K_A_CommandString "P6K_A_Command"

#define P6K_MAXBUF 1024

class p6kController : public asynMotorController {

 public:
  p6kController(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress, int numAxes, double movingPollPeriod, 
		 double idlePollPeriod);

  virtual ~p6kController();

  asynStatus printConnectedStatus(void);

  /* These are the methods that we override */
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
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
  int P6K_C_Command_;
  int P6K_A_Command_;
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
  asynStatus lowLevelPortConnect(const char *port, int addr, asynUser **ppasynUser, char *inputEos, char *outputEos);

  asynStatus getGlobalStatus(epicsUInt32 *globalStatus);

  asynStatus processDeferredMoves(void);

  //static class data members

  static const epicsUInt32 P6K_MAXBUF_;
  static const epicsFloat64 P6K_TIMEOUT_;
  static const epicsUInt32 P6K_FORCED_FAST_POLLS_;
  static const epicsUInt32 P6K_OK_;
  static const epicsUInt32 P6K_ERROR_;
  static const epicsUInt32 P6K_ERROR_PRINT_TIME_;

  /*  static const epicsUInt32 P6K_STATUS1_MAXRAPID_SPEED;    
  static const epicsUInt32 P6K_STATUS1_ALT_CMNDOUT_MODE;  
  static const epicsUInt32 P6K_STATUS1_SOFT_POS_CAPTURE;
  static const epicsUInt32 P6K_STATUS1_ERROR_TRIGGER;
  static const epicsUInt32 P6K_STATUS1_FOLLOW_ENABLE;   
  static const epicsUInt32 P6K_STATUS1_FOLLOW_OFFSET;   
  static const epicsUInt32 P6K_STATUS1_PHASED_MOTOR;   
  static const epicsUInt32 P6K_STATUS1_ALT_SRC_DEST;    
  static const epicsUInt32 P6K_STATUS1_USER_SERVO;      
  static const epicsUInt32 P6K_STATUS1_USER_PHASE;      
  static const epicsUInt32 P6K_STATUS1_HOMING;          
  static const epicsUInt32 P6K_STATUS1_BLOCK_REQUEST;   
  static const epicsUInt32 P6K_STATUS1_DECEL_ABORT;     
  static const epicsUInt32 P6K_STATUS1_DESIRED_VELOCITY_ZERO;
  static const epicsUInt32 P6K_STATUS1_DATABLKERR;        
  static const epicsUInt32 P6K_STATUS1_DWELL;             
  static const epicsUInt32 P6K_STATUS1_INTEGRATE_MODE;    
  static const epicsUInt32 P6K_STATUS1_MOVE_TIME_ON;      
  static const epicsUInt32 P6K_STATUS1_OPEN_LOOP;         
  static const epicsUInt32 P6K_STATUS1_AMP_ENABLED;       
  static const epicsUInt32 P6K_STATUS1_X_SERVO_ON;        
  static const epicsUInt32 P6K_STATUS1_POS_LIMIT_SET;     
  static const epicsUInt32 P6K_STATUS1_NEG_LIMIT_SET;     
  static const epicsUInt32 P6K_STATUS1_MOTOR_ON; 
  
  static const epicsUInt32 P6K_HARDWARE_PROB;
 static const epicsUInt32  P6K_AXIS_GENERAL_PROB1;
  static const epicsUInt32 P6K_AXIS_GENERAL_PROB2;
  */

  friend class p6kAxis;

};

#define NUM_P6K_PARAMS (&LAST_P6K_PARAM - &FIRST_P6K_PARAM + 1)

#endif /* parker6kController_H */
