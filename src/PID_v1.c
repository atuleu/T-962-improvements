/**********************************************************************************************
 * C PID Library - Version 1.0.1
 * modified my Matthew Blythe <mblythester@gmail.com> mjblythe.com/hacks
 * originally by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "PID_v1.h"
#include "math.h"
#include "utils.h"

void PID_Initialize(PidType* pid);

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_init(PidType* pid, FloatType Kp, FloatType Ki, FloatType Kd) {
  pid->ITerm     = 0;
  pid->lastError = NAN;
  pid->inAuto    = false;

  PID_SetOutputLimits(pid, 0, 0xffff);

  // default Controller Sample Time is 0.1 seconds
  pid->SampleTime = 100;

  PID_SetTunings(pid, Kp, Ki, Kd);

  //  pid->lastTime = millis() - pid->SampleTime;
}

/* Compute()
 *********************************************************************** This,
 *as they say, is where the magic happens.  this function should be called every
 *time "void loop()" executes.  the function will decide for itself whether a
 *new pid Output needs to be computed.  returns true when the output is
 *computed, false when nothing has been done.
 **********************************************************************************/
FloatType PID_Compute(PidType* pid, FloatType target, FloatType actual) {
  if(!pid->inAuto) {
    return NAN;
  }
  //  unsigned long now = millis();
  //  unsigned long timeChange = (now - pid->lastTime);
  //  if (timeChange >= pid->SampleTime) {
  /*Compute all the working error variables*/
  FloatType error = target - actual;
  pid->ITerm += pid->ki * error;
  pid->ITerm = MATH_CLAMP(pid->ITerm, pid->outMin, pid->outMax);

  FloatType dError = isnan(pid->lastError) ? 0.0 : (error - pid->lastError);

  /*Compute PID Output*/
  FloatType output = pid->kp * error + pid->ITerm - pid->kd * dError;
  /*Remember some variables for next time*/
  pid->lastError   = error;

  // printf("\n no clamp: %f min: %f max:%f \n", output, pid->outMin,
  // pid->outMax);

  return MATH_CLAMP(output, pid->outMin, pid->outMax);
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/

void PID_SetTunings(PidType* pid, FloatType Kp, FloatType Ki, FloatType Kd) {
  if(Kp < 0 || Ki < 0 || Kd < 0) {
    return;
  }

  pid->dispKp = Kp;
  pid->dispKi = Ki;
  pid->dispKd = Kd;

  FloatType SampleTimeInSec = ((FloatType)pid->SampleTime) / 1000;
  pid->kp                   = Kp;
  pid->ki                   = Ki * SampleTimeInSec;
  pid->kd                   = Kd / SampleTimeInSec;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PidType* pid, int NewSampleTime) {
  if(NewSampleTime > 0) {
    FloatType ratio = (FloatType)NewSampleTime / (FloatType)pid->SampleTime;
    pid->ki *= ratio;
    pid->kd /= ratio;
    pid->SampleTime = (unsigned long)NewSampleTime;
  }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PidType* pid, FloatType Min, FloatType Max) {
  if(Min >= Max) {
    return;
  }

  pid->outMin = Min;
  pid->outMax = Max;

  if(pid->inAuto) {
    pid->ITerm = MATH_CLAMP(pid->ITerm, Min, Max);
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PidType* pid, PidModeType Mode) {
  bool newAuto = (Mode == PID_Mode_Automatic);
  if(newAuto == !pid->inAuto) { /*we just went from manual to auto*/
    PID_Initialize(pid);
  }
  pid->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_Initialize(PidType* pid) {
  pid->ITerm     = MATH_CLAMP(0, pid->outMin, pid->outMax);
  pid->lastError = NAN;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
FloatType PID_GetKp(PidType* pid) { return pid->dispKp; }
FloatType PID_GetKi(PidType* pid) { return pid->dispKi; }
FloatType PID_GetKd(PidType* pid) { return pid->dispKd; }
PidModeType PID_GetMode(PidType* pid) {
  return pid->inAuto ? PID_Mode_Automatic : PID_Mode_Manual;
}
