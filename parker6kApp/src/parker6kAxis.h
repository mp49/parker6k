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

#ifndef p6kAxis_H
#define p6kAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class p6kController;

class p6kAxis : public asynMotorAxis
{
  public:
  /* These are the methods we override from the base class */
  p6kAxis(p6kController *pController, int axisNo);
  virtual ~p6kAxis();
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  
  private:
  p6kController *pC_;
  
  asynStatus getAxisStatus(bool *moving);
  asynStatus getAxisInitialStatus(void);

  double setpointPosition_;
  double encoderPosition_;
  double currentVelocity_;
  double velocity_;
  double accel_;
  double highLimit_;
  double lowLimit_;
  int limitsDisabled_;
  double stepSize_;
  double deferredPosition_;
  int deferredMove_;
  int deferredRelative_;
  int scale_;
  double previous_position_;
  int previous_direction_;
  int amp_enabled_;
  int fatal_following_;
  int encoder_axis_;
  //  int limitsCheckDisable_;
  epicsTimeStamp nowTime_;
  epicsFloat64 nowTimeSecs_;
  epicsFloat64 lastTimeSecs_; 
  bool printNextError_;

  static const epicsUInt32 P6K_TAS_MOVING_;
  static const epicsUInt32 P6K_TAS_DIRECTION_;
  static const epicsUInt32 P6K_TAS_ACCELERATING_;
  static const epicsUInt32 P6K_TAS_ATVELOCITY_;
  static const epicsUInt32 P6K_TAS_HOMED_;
  static const epicsUInt32 P6K_TAS_ABSOLUTE_;
  static const epicsUInt32 P6K_TAS_CONTINUOUS_;
  static const epicsUInt32 P6K_TAS_JOG_;
  static const epicsUInt32 P6K_TAS_JOYSTICK_;
  static const epicsUInt32 P6K_TAS_STALL_;
  static const epicsUInt32 P6K_TAS_DRIVE_;
  static const epicsUInt32 P6K_TAS_DRIVEFAULT_;
  static const epicsUInt32 P6K_TAS_POSLIM_;
  static const epicsUInt32 P6K_TAS_NEGLIM_;
  static const epicsUInt32 P6K_TAS_POSLIMSOFT_;
  static const epicsUInt32 P6K_TAS_NEGLIMSOFT_;
  static const epicsUInt32 P6K_TAS_POSERROR_;
  static const epicsUInt32 P6K_TAS_TARGETZONE_;
  static const epicsUInt32 P6K_TAS_TARGETTIMEOUT_;
  static const epicsUInt32 P6K_TAS_GOWHENPEND_;
  static const epicsUInt32 P6K_TAS_MOVEPEND_;
  static const epicsUInt32 P6K_TAS_PREEMPT_;
 
  
  

  friend class p6kController;
};


#endif /* p6kAxis_H */
