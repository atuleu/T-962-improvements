/*
 * reflow.c - Actual reflow profile logic for T-962 reflow controller
 *
 * Copyright (C) 2014 Werner Johansson, wj@unifiedengineering.se
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "reflow.h"
#include "LPC214x.h"
#include "PID_v1.h"
#include "io.h"
#include "lcd.h"
#include "math.h"
#include "nvstorage.h"
#include "reflow_profiles.h"
#include "rtc.h"
#include "sched.h"
#include "sensor.h"
#include "setup.h"
#include "t962.h"
#include <stdint.h>
#include <stdio.h>

// Standby temperature in degrees Celsius
#define STANDBYTEMP (50)

// 250ms between each run
#define PID_TIMEBASE (250)

#define TICKS_PER_SECOND (1000 / PID_TIMEBASE)

static PidType PID;

static uint16_t intsetpoint;
static int bake_timer = 0;

static float avgtemp;

static bool reflowdone     = false;
static ReflowMode_t mymode = REFLOW_STANDBY;
static uint16_t numticks   = 0;

static int standby_logging = 0;

bool Reflow_RunProfile(uint32_t thetime,
                       float meastemp,
                       uint8_t* pheat,
                       uint8_t* pfan);

bool Reflow_RunManual(float meastemp,
                      uint8_t* pheat,
                      uint8_t* pfan,
                      int32_t setpoint);

bool Reflow_RunAutotune(float meastemp,
                        uint8_t* pheat,
                        uint8_t* pfan,
                        int32_t setpoint);

static int32_t Reflow_Work(void) {
  static ReflowMode_t oldmode = REFLOW_INITIAL;
  static uint32_t lasttick    = 0;
  uint8_t fan, heat;
  uint32_t ticks = RTC_Read();

  Sensor_DoConversion();
  avgtemp = Sensor_GetTemp(TC_AVERAGE);

  const char* modestr = "UNKNOWN";

  // Depending on mode we should run this with different parameters
  if(mymode == REFLOW_STANDBY || mymode == REFLOW_STANDBYFAN) {
    intsetpoint = STANDBYTEMP;
    // Cool to standby temp but don't heat to get there
    Reflow_RunManual(avgtemp, &heat, &fan, intsetpoint);
    heat = 0;

    // Suppress slow-running fan in standby
    if(mymode == REFLOW_STANDBY && avgtemp < (float)STANDBYTEMP) {
      fan = 0;
    }
    modestr = "STANDBY";

  } else if(mymode == REFLOW_BAKE) {
    reflowdone = Reflow_RunManual(avgtemp, &heat, &fan, intsetpoint);
    modestr    = "BAKE";

  } else if(mymode == REFLOW_REFLOW) {
    reflowdone = Reflow_RunProfile(ticks, avgtemp, &heat, &fan);
    modestr    = "REFLOW";

  } else if(mymode == REFLOW_AUTOTUNE) {
    reflowdone = Reflow_RunAutotune(avgtemp, &heat, &fan, intsetpoint);
    modestr    = "AUTOTUNE";
  } else {
    heat = fan = 0;
  }
  Set_Heater(heat);
  Set_Fan(fan);

  if(mymode != oldmode) {
    printf("\n# Time,  Temp0, Temp1, Temp2, Temp3,  Set,Actual, Heat, Fan,  "
           "ColdJ, Mode");
    oldmode  = mymode;
    numticks = 0;
  } else if(mymode == REFLOW_BAKE) {
    if(bake_timer > 0 && numticks >= bake_timer) {
      printf("\n DONE baking, set bake timer to 0.");
      bake_timer = 0;
      Reflow_SetMode(REFLOW_STANDBY);
    }

    // start increasing ticks after setpoint is reached...
    if(avgtemp < intsetpoint && bake_timer > 0) {
      modestr = "BAKE-PREHEAT";
    } else {
      numticks++;
    }
  } else if(mymode == REFLOW_REFLOW) {
    numticks++;
  }

  if(!(mymode == REFLOW_STANDBY && standby_logging == 0)) {
    printf("\n%6.1f,  %5.1f, %5.1f, %5.1f, %5.1f,  %3u, %5.1f,  %3u, %3u,  "
           "%5.1f, %s",
           ((float)numticks / TICKS_PER_SECOND), Sensor_GetTemp(TC_LEFT),
           Sensor_GetTemp(TC_RIGHT), Sensor_GetTemp(TC_EXTRA1),
           Sensor_GetTemp(TC_EXTRA2), intsetpoint, avgtemp, heat, fan,
           Sensor_GetTemp(TC_COLD_JUNCTION), modestr);
  }

  if(numticks & 1) {
    // Force UI refresh every other cycle
    Sched_SetState(MAIN_WORK, 2, 0);
  }

  uint32_t thistick = Sched_GetTick();
  if(lasttick == 0) {
    lasttick = thistick - TICKS_MS(PID_TIMEBASE);
  }

  int32_t nexttick = (2 * TICKS_MS(PID_TIMEBASE)) - (thistick - lasttick);
  if((thistick - lasttick) > (2 * TICKS_MS(PID_TIMEBASE))) {
    printf("\nReflow can't keep up with desired PID_TIMEBASE!");
    nexttick = 0;
  }
  lasttick += TICKS_MS(PID_TIMEBASE);
  return nexttick;
}

void Reflow_Init(void) {
  Sched_SetWorkfunc(REFLOW_WORK, Reflow_Work);

  // This does not reach the setpoint fast enough
  // PID_init(&PID, 10, 0.04, 5, PID_Direction_Direct);

  // This reaches the setpoint but oscillates a bit especially during cooling
  // PID_init(&PID, 30, 0.2, 5, PID_Direction_Direct);

  // This overshoots the setpoint
  // PID_init(&PID, 30, 0.2, 15, PID_Direction_Direct);

  // This
  // overshoots the setpoint slightly
  // PID_init(&PID, 25, 0.15, 15, PID_Direction_Direct);

  // PID_init(&PID, 20, 0.07, 25, PID_Direction_Direct);

  // PID_init(&PID, 20, 0.04, 25, PID_Direction_Direct);
  // Improvement as far as I can tell, still work in progress
  //  Can't supply tuning to PID_Init when not using the default timebase

  // PID_SetTunings(&PID, 80, 0, 0); // This results in oscillations with 14.5s
  // cycle time PID_SetTunings(&PID, 30, 0, 0); // This results in oscillations
  // with 14.5s cycle time PID_SetTunings(&PID, 15, 0, 0); PID_SetTunings(&PID,
  // 10, 0, 0); // no oscillations, but offset PID_SetTunings(&PID, 10, 0.020,
  // 0); // getting there PID_SetTunings(&PID, 10, 0.013, 0);
  // PID_SetTunings(&PID, 10, 0.0066, 0);
  // PID_SetTunings(&PID, 10, 0.2, 0);
  // PID_SetTunings(&PID, 10, 0.020, 1.0); // Experimental

  Reflow_LoadCustomProfiles();

  Reflow_ValidateNV();
  Sensor_ValidateNV();

  Reflow_LoadSetpoint();

  PID_SetOutputLimits(&PID, -255, 255);
  PID_SetMode(&PID, PID_Mode_Manual);
  PID_SetMode(&PID, PID_Mode_Automatic);

  PID_init(&PID, 0, 0, 0);
  PID_SetSampleTime(&PID, PID_TIMEBASE);
  // Adjusted values to compensate for the incorrect timebase earlier
  PID_SetTunings(&PID, Setup_getValue(PID_K_VALUE_H),
                 Setup_getValue(PID_I_VALUE_H), Setup_getValue(PID_D_VALUE_H));

  RTC_Zero();

  // Start work
  Sched_SetState(REFLOW_WORK, 2, 0);
}

void Reflow_SetMode(ReflowMode_t themode) {
  mymode = themode;
  // reset reflowdone if mode is set to standby.
  if(themode == REFLOW_STANDBY) {
    reflowdone = false;
  }
}

void Reflow_SetSetpoint(uint16_t thesetpoint) {
	intsetpoint = thesetpoint;

	NV_SetConfig(REFLOW_BAKE_SETPOINT_H, (uint8_t)(thesetpoint >> 8));
	NV_SetConfig(REFLOW_BAKE_SETPOINT_L, (uint8_t)thesetpoint);
}

void Reflow_LoadSetpoint(void) {
	intsetpoint = NV_GetConfig(REFLOW_BAKE_SETPOINT_H) << 8;
	intsetpoint |= NV_GetConfig(REFLOW_BAKE_SETPOINT_L);

	printf("\n bake setpoint values: %x, %x, %d\n",
		NV_GetConfig(REFLOW_BAKE_SETPOINT_H),
		NV_GetConfig(REFLOW_BAKE_SETPOINT_L), intsetpoint);
}

int16_t Reflow_GetActualTemp(void) {
	return (int)Sensor_GetTemp(TC_AVERAGE);
}

uint8_t Reflow_IsDone(void) {
	return reflowdone;
}

uint16_t Reflow_GetSetpoint(void) {
	return intsetpoint;
}

void Reflow_SetBakeTimer(int seconds) {
	// reset ticks to 0 when adjusting timer.
	numticks = 0;
	bake_timer = seconds * TICKS_PER_SECOND;
}

int Reflow_IsPreheating(void) {
	return bake_timer > 0 && avgtemp < intsetpoint;
}

int Reflow_GetTimeLeft(void) {
  if(bake_timer == 0) {
    return -1;
  }
  return (bake_timer - numticks) / TICKS_PER_SECOND;
}

void Reflow_setOuput(int16_t out, uint8_t* pheat, uint8_t* pfan) {
  if(out > 0) {
    *pheat = out;
    *pfan  = NV_GetConfig(REFLOW_MIN_FAN_SPEED);
  } else {
    *pheat = 0;
    *pfan  = -out;
  }
}

bool Reflow_RunManual(float meastemp,
                      uint8_t* pheat,
                      uint8_t* pfan,
                      int32_t setpoint) {
  Reflow_setOuput(PID_Compute(&PID, setpoint, meastemp), pheat, pfan);
  if(bake_timer > 0 &&
     (Reflow_GetTimeLeft() == 0 || Reflow_GetTimeLeft() == -1)) {
    return true;
  }
  return false;
}

// returns -1 if the reflow process is done.
bool Reflow_RunProfile(uint32_t thetime,
                       float meastemp,
                       uint8_t* pheat,
                       uint8_t* pfan) {

  float target;

  // Figure out what setpoint to use from the profile, brute-force way. Fix
  // this.
  uint8_t idx     = thetime / 10;
  uint16_t start  = idx * 10;
  uint16_t offset = thetime - start;
  if(idx < (NUMPROFILETEMPS - 2)) {
    uint16_t valueStartOfSegment = Reflow_GetSetpointAtIdx(idx);
    uint16_t valueEndOfSegment   = Reflow_GetSetpointAtIdx(idx + 1);

    if(valueStartOfSegment > 0 && valueEndOfSegment > 0) {
      uint16_t interpolated =
          (valueStartOfSegment * (10 - offset) + valueEndOfSegment * offset) /
          10;

      // Keep the interpolated setpoint for the UI...
      intsetpoint = interpolated;
      if(valueEndOfSegment > interpolated) {
        // Temperature is rising,
        // using the future value for PID regulation produces better result
        // when heating
        target = (float)valueEndOfSegment;
      } else {
        // Use the interpolated value when cooling
        target = (float)interpolated;
      }
    } else { // we reach the end of the segments

      // we expect here to be cooling, just ensure we are not heating for
      // whatsoever reason;
      *pheat = 0;

      return true;
    }
  } else { // no more segment in profile

    // same as above, we ensure we are not heating for any reason.
    *pheat = 0;
    return true;
  }

  // Plot actual temperature on top of desired profile
  int realx = (thetime / 5) + XAXIS;
  int y     = (uint16_t)(meastemp * 0.2f);
  y         = YAXIS - y;
  LCD_SetPixel(realx, y);

  Reflow_setOuput(PID_Compute(&PID, target, meastemp), pheat, pfan);

  return false;
}

void Reflow_ToggleStandbyLogging(void) { standby_logging = !standby_logging; }

#define AT_CYCLES 8
typedef enum
{
  AT_COOLDONW   = 0,
  AT_RUN_CYCLES = 1
} AT_State_t;

typedef struct {
  float extremums[2 * AT_CYCLES];
  uint32_t times[2 * AT_CYCLES];
  float last;
  uint8_t nextIdx;
  uint32_t numTick;
  float Kmin;
  float Kmax;
  float KNext;
  AT_State_t State;
} Autotune_t;

static Autotune_t at_data;

void Reflow_StartAutotune(float Kstart, uint16_t setpoint) {
  reflowdone = false;
  Reflow_SetSetpoint(setpoint);
  mymode          = REFLOW_AUTOTUNE;
  at_data.KNext   = Kstart;
  at_data.Kmax    = NAN;
  at_data.Kmax    = NAN;
  at_data.State   = AT_COOLDONW;
  at_data.numTick = 0;
}

bool AT_done() {
  if(isnan(at_data.Kmax) || isnan(at_data.Kmin)) {
    return false;
  }
  return (at_data.Kmax - at_data.Kmin) / at_data.Kmin <
         0.05; // 5% relative error;
}

extern uint8_t graphbmp[];

void plotTemperature(uint32_t tick, float temp) {
  if(tick % (TICKS_PER_SECOND * 5) != 0) {
    return;
  }
  int realx = tick / (TICKS_PER_SECOND * 5) + XAXIS;
  int y     = (uint16_t)(temp * 0.2f);
  y         = YAXIS - y;
  LCD_SetPixel(realx, y);
}

float Reflow_Autotune_GetKp() { return at_data.KNext; }

bool Reflow_RunAutotune(float meastemp,
                        uint8_t* pheat,
                        uint8_t* pfan,
                        int32_t setpoint) {
  plotTemperature(at_data.numTick, meastemp);
  if(at_data.State == AT_COOLDONW) {
    if(meastemp > STANDBYTEMP) {
      *pfan = 255;
      return false;
    }

    if(AT_done()) {
      float Tu = 0.0;
      for(int i = 0; i < AT_CYCLES - 1; i++) {
        Tu += 0.25 * (at_data.times[2 * (i + 1)] - at_data.times[2 * i]);
      }
      Tu /= (AT_CYCLES - 1);
      printf("\n\nFound Ku: %.3f and Tu= %.3f\n", at_data.KNext, Tu);
      printf("\n\nPID with small overshoot values : %.3f %.3f %.3f\n",
             at_data.KNext / 3.0, at_data.KNext * 0.666 / Tu,
             at_data.KNext * Tu / 9);
      return true;
    }

    LCD_FB_Clear();
    LCD_BMPDisplay(graphbmp, 0, 0);

    at_data.nextIdx = 0;
    at_data.State   = AT_RUN_CYCLES;
    PID_SetTunings(&PID, at_data.KNext, 0, 0);
    PID_SetMode(&PID, PID_Mode_Manual);
    PID_SetMode(&PID, PID_Mode_Automatic);
    at_data.last         = meastemp;
    at_data.extremums[0] = NAN;
    at_data.numTick      = -1;
    printf("\nTesting K=%.3f\n\n", at_data.KNext);
  }

  Reflow_setOuput(PID_Compute(&PID, intsetpoint, meastemp), pheat, pfan);
  bool newCandidate = false;
  if(at_data.nextIdx % 2 == 0) {
    newCandidate = meastemp > at_data.last;
  } else {
    newCandidate = meastemp < at_data.last;
  }
  at_data.last = meastemp;

  if(newCandidate) {
    at_data.extremums[at_data.nextIdx] = meastemp;
    at_data.times[at_data.nextIdx]     = at_data.numTick;
    // not reached an extrema yet, continue
    return false;
  } else if((at_data.times[at_data.nextIdx] - at_data.numTick) >= 10 // 2.5s
            && isnan(at_data.extremums[at_data.nextIdx]) == false) {
    at_data.nextIdx++;
    printf("\n found max at %.1f\n\n", at_data.extremums[at_data.nextIdx - 1]);
  } else {
    return false; // no ext found, continue
  }

  if(at_data.nextIdx < 2 * AT_CYCLES) {
    at_data.extremums[at_data.nextIdx] = NAN;
    // still not measured enough cycles, continue
    return false;
  }

  float avgStart = 0.0, avgEnd = 0.0;
  for(int i = 0; i < AT_CYCLES / 2; ++i) {
    avgStart += fabsf(at_data.extremums[i] - intsetpoint);
    avgEnd += fabs(at_data.extremums[2 * AT_CYCLES - 1 - i] - intsetpoint);
  }

  if((avgEnd - avgStart) / avgEnd < -0.01) {
    // Decreasing -> we set Kmin
    at_data.Kmin = at_data.KNext;
    printf("\nDecreasing oscillation\n\n");
  } else {
    // Increasing -> we set Kmax
    printf("\nIncreasing oscillation\n\n");
    at_data.Kmax = at_data.KNext;
  }

  if(isnan(at_data.Kmax)) {
    at_data.KNext = 2 * at_data.Kmin;
  } else if(isnan(at_data.Kmin)) {
    at_data.KNext = at_data.Kmax / 2.0;
  } else {
    at_data.KNext = at_data.Kmax + at_data.Kmin / 2;
  }

  // we cooldown
  at_data.State = AT_COOLDONW;
  *pheat        = 0;
  *pfan         = 255;

  return false;
}
