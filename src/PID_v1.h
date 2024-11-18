#ifndef PID_H
#define PID_H

typedef float FloatType;
// typedef double floatType;
#include <stdbool.h>

// Constants used in some of the functions below
typedef enum
{
  PID_Mode_Automatic = 1,
  PID_Mode_Manual    = 0
} PidModeType;

typedef struct {
  FloatType dispKp; // * we'll hold on to the tuning parameters in user-entered
  FloatType dispKi; //   format for display purposes
  FloatType dispKd; //

  FloatType kp; // * (P)roportional Tuning Parameter
  FloatType ki; // * (I)ntegral Tuning Parameter
  FloatType kd; // * (D)erivative Tuning Parameter

  //  unsigned long lastTime;
  FloatType ITerm, lastError;

  unsigned long SampleTime;
  FloatType outMin, outMax;
  bool inAuto;
} PidType;

// commonly used functions
// **************************************************************************

//  constructor.  links the PID to the Input, Output, and
//  Setpoint.  Initial tuning parameters are also set here
void PID_init(PidType* pid, FloatType kp, FloatType ki, FloatType kd);

// sets PID to either Manual (0) or Auto (non-0)
void PID_SetMode(PidType* pid, PidModeType mode);

// performs the PID calculation.  it should be
// called every time loop() cycles. ON/OFF and
// calculation frequency can be set using SetMode
// SetSampleTime respectively
FloatType PID_Compute(PidType* pid, FloatType target, FloatType actual);

// clamps the output to a specific range. 0-255 by default, but
// it's likely the user will want to change this depending on
// the application
void PID_SetOutputLimits(PidType* pid, FloatType min, FloatType max);

// available but not commonly used functions
// ********************************************************

// While most users will set the tunings once in the
// constructor, this function gives the user the option
// of changing tunings during runtime for Adaptive control
void PID_SetTunings(PidType* pid, FloatType kp, FloatType ki, FloatType kd);

// sets the frequency, in Milliseconds, with which
// the PID calculation is performed.  default is 100
void PID_SetSampleTime(PidType* pid, int newSampleTime);

// Display functions
// ****************************************************************
//  These functions query the pid for interal values.
//   they were created mainly for the pid front-end,
//  where it's important to know what is actually
//   inside the PID.
FloatType PID_GetKp(PidType* pid);
FloatType PID_GetKi(PidType* pid);
FloatType PID_GetKd(PidType* pid);
PidModeType PID_GetMode(PidType* pid);

// void PID_Initialize(PidType* pid);
#endif
