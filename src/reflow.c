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
#include "utils.h"
#include <stdint.h>
#include <stdio.h>

// Standby temperature in degrees Celsius
#define STANDBYTEMP (50)

// 250ms between each run
#define PID_TIMEBASE_MS (250)

// 100ms between each run
#define AUTOTUNE_TIMEBASE_MS (100)

static PidType PID;

static uint16_t intsetpoint;
static int bake_timer = 0;

static float avgtemp;

static bool reflowdone     = false;
static ReflowMode_t mymode = REFLOW_STANDBY;

static bool standby_logging = false;

typedef struct {
  uint16_t tick;
  uint8_t period_ms;
  uint32_t period_tick;
  uint16_t tick_per_second;
} period_t;

static period_t time = {
    .tick            = 0,
    .period_ms       = PID_TIMEBASE_MS,
    .period_tick     = TICKS_MS(PID_TIMEBASE_MS),
    .tick_per_second = 1000 / PID_TIMEBASE_MS,
};

void Reflow_setPeriod(uint8_t period_ms) {
  if(period_ms == 0 || period_ms == time.period_ms) {
    return;
  }
  time.period_ms       = period_ms;
  time.period_tick     = TICKS_MS(period_ms);
  time.tick_per_second = 1000 / period_ms;
}

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

  bool logging = false;
  // Depending on mode we should run this with different parameters
  switch(mymode) {
  case REFLOW_STANDBY:
  case REFLOW_STANDBYFAN:
  case REFLOW_AUTOTUNE_COOLDOWN:
    Reflow_setPeriod(PID_TIMEBASE_MS);
    intsetpoint = STANDBYTEMP;
    // Cool to standby temp but don't heat to get there
    heat        = 0;
    fan =
        (uint8_t)MATH_CLAMP(30.0 * (avgtemp - (float)STANDBYTEMP), 0.0, 255.0);
    // Suppress slow-running fan in standby
    modestr = "STANDBY";

    logging = standby_logging;
    break;
  case REFLOW_BAKE:
    Reflow_setPeriod(PID_TIMEBASE_MS);
    reflowdone = Reflow_RunManual(avgtemp, &heat, &fan, intsetpoint);
    modestr    = "BAKE";
    logging    = true;
    break;
  case REFLOW_REFLOW:
    Reflow_setPeriod(PID_TIMEBASE_MS);
    reflowdone = Reflow_RunProfile(ticks, avgtemp, &heat, &fan);
    modestr    = "REFLOW";
    logging    = true;
    break;
  case REFLOW_AUTOTUNE:
    Reflow_setPeriod(AUTOTUNE_TIMEBASE_MS);
    modestr    = "AUTOTUNE";
    logging    = false;
    reflowdone = Reflow_RunAutotune(avgtemp, &heat, &fan, intsetpoint);
    if(reflowdone == true) {
      // we switch to standby mode immediatly once autotune is done.
      mymode = REFLOW_AUTOTUNE_COOLDOWN;
    }
    break;
  default:
    Reflow_setPeriod(PID_TIMEBASE_MS);
    heat = fan = 0;
  }

  Set_Heater(heat);
  Set_Fan(fan);

  if(mymode != oldmode) {
    printf("\n# Time,  Temp0, Temp1, Temp2, Temp3,  Set,Actual, Heat, Fan,  "
           "ColdJ, Mode");
    oldmode   = mymode;
    time.tick = 0;
  } else if(mymode == REFLOW_BAKE) {
    if(bake_timer > 0 && time.tick >= bake_timer) {
      printf("\n DONE baking, set bake timer to 0.");
      bake_timer = 0;
      Reflow_SetMode(REFLOW_STANDBY);
    }

    // start increasing ticks after setpoint is reached...
    if(avgtemp < intsetpoint && bake_timer > 0) {
      modestr = "BAKE-PREHEAT";
    } else {
      time.tick++;
    }
  } else if(mymode == REFLOW_REFLOW || mymode == REFLOW_AUTOTUNE) {
    time.tick++;
  }

  if(logging == true) {
    printf("\n%6.1f,  %5.1f, %5.1f, %5.1f, %5.1f,  %3u, %5.1f,  %3u, %3u,  "
           "%5.1f, %s",
           ((float)time.tick / time.tick_per_second), Sensor_GetTemp(TC_LEFT),
           Sensor_GetTemp(TC_RIGHT), Sensor_GetTemp(TC_EXTRA1),
           Sensor_GetTemp(TC_EXTRA2), intsetpoint, avgtemp, heat, fan,
           Sensor_GetTemp(TC_COLD_JUNCTION), modestr);
  }

  if(time.tick % time.tick_per_second == 0) {
    // Force UI refresh every second
    Sched_SetState(MAIN_WORK, 2, 0);
  }

  // period            = TICKS_MS(period);
  //  reschedule task according to period
  uint32_t thistick = Sched_GetTick();
  if(lasttick == 0) {
    lasttick = thistick - time.period_tick;
  }

  int32_t nexttick = (2 * time.period_tick) - (thistick - lasttick);
  if((thistick - lasttick) > (2 * time.period_tick)) {
    printf("\nReflow can't keep up with desired PID_TIMEBASE!");
    nexttick = 0;
  }
  lasttick += time.period_tick;
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

  PID_Reset(&PID);

  PID_init(&PID, 0, 0, 0);
  PID_SetOutputLimits(&PID, -255, 255);

  PID_SetSampleTime(&PID, PID_TIMEBASE_MS);
  // Adjusted values to compensate for the incorrect timebase earlier
  float Kp = Setup_getValue(SETTINGS_PID_KP);
  float Ki = Setup_getValue(SETTINGS_PID_KI);
  float Kd = Setup_getValue(SETTINGS_PID_KD);
  Reflow_UpdatePID(Kp, Ki, Kd);

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

void Reflow_UpdatePID(float Kp, float Ki, float Kd) {
  printf("\nPID parameters are Kp=%.1f Ki=%.3f Kd=%.1f\n", Kp, Ki, Kd);
  PID_SetTunings(&PID, Kp, Ki, Kd);
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
  time.tick  = 0;
  bake_timer = seconds * (1000 / PID_TIMEBASE_MS);
}

int Reflow_IsPreheating(void) {
	return bake_timer > 0 && avgtemp < intsetpoint;
}

int Reflow_GetTimeLeft(void) {
  if(bake_timer == 0) {
    return -1;
  }
  return (bake_timer - time.tick) / time.tick_per_second;
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

  float out = PID_Compute(&PID, target, meastemp);

  Reflow_setOuput(out, pheat, pfan);

  return false;
}

void Reflow_ToggleStandbyLogging(void) { standby_logging = !standby_logging; }

//==============================================================================
// PID autotune Inspired from marlin controller. Idea comes from "Automatic
// tuning of simple regulators with specifications on phase and amplitude
// margins." ( https://doi.org/10.1016/0005-1098(84)90014-1 ). The idea is to
// use a relay controller and to look for anti-phase oscillation. An euristic is
// to adjust the amplitude of the relay controller in order to match the Up and
// down half-period.

#define AT_CYCLES 10

typedef struct {
  uint16_t Target_low, Target_high;
  float max, min;
  int32_t amplitude;
  int32_t bias;
  int8_t iter, on_target;
  bool rampUp;
  uint16_t t_down, t_up, t_last;

} Autotune_t;

static Autotune_t at_data;

extern uint8_t graphbmp[];
extern uint8_t stopbmp[];

static char buf[22];
static int len;

void Reflow_lcdStatus(char* msg, int len) {}

void Reflow_at_startHeat(uint8_t* pheat, uint8_t* pfan) {
  at_data.rampUp = true;
  at_data.max    = at_data.Target_low;

  intsetpoint = at_data.Target_high; // do not save it to eeprom

  len = snprintf(buf, sizeof(buf), "Heat [%d/%d]", at_data.iter + 1, AT_CYCLES);
  LCD_disp_str((uint8_t*)buf, len, 13, 0, FONT6X6);
  if(pheat != NULL && pfan != NULL) {
    int32_t out = MATH_CLAMP(at_data.bias + at_data.amplitude, -255, 255);
    Reflow_setOuput(out, pheat, pfan);
  }
}

void Reflow_at_startCool(uint8_t* pheat, uint8_t* pfan) {
  at_data.rampUp = false;
  at_data.min    = at_data.Target_high;

  intsetpoint = at_data.Target_low; // do not save it to eeprom

  len = snprintf(buf, sizeof(buf), "Cool [%d/%d]", at_data.iter + 1, AT_CYCLES);
  LCD_disp_str((uint8_t*)buf, len, 13, 0, FONT6X6);
  if(pheat != NULL && pfan != NULL) {
    int32_t out = MATH_CLAMP(at_data.bias - at_data.amplitude, -255, 255);
    Reflow_setOuput(out, pheat, pfan);
  }
}

void Reflow_StartAutotune(uint16_t low, uint16_t high) {
  reflowdone = false;

  mymode              = REFLOW_AUTOTUNE;
  at_data.Target_high = MATH_MAX(low, high);
  at_data.Target_low  = MATH_MIN(low, high);
  at_data.max         = 0.0;
  at_data.min         = 1000.0;
  at_data.iter        = -1;
  at_data.t_down      = 0;
  at_data.t_up        = 0;
  at_data.t_last      = 0;
  at_data.amplitude   = 255;
  at_data.bias        = 0;
  at_data.on_target   = 0;
  time.tick           = 0;

  LCD_FB_Clear();
  LCD_BMPDisplay(graphbmp, 0, 0);
  LCD_BMPDisplay(stopbmp, 127 - 17, 0);
}
void plotTemperature(uint32_t tick, float temp) {
  if(tick % (time.tick_per_second * 5) != 0) {
    return;
  }
  int realx = tick / (time.tick_per_second * 5) + XAXIS;
  int y     = (uint16_t)(temp * 0.2f);
  y         = YAXIS - y;
  LCD_SetPixel(realx, y);
}

float Reflow_Autotune_Ku() {
  float out_amplitude = at_data.max - at_data.min;
  // formula from paper to get Critical gain. We estimated bias and
  // half-amplitude to build a PI/2 phase response. Using ideal relay we got:
  float Ku            = 4.0 * at_data.amplitude * 2.0 /
             ((float)(3.14159265) * out_amplitude * 0.5f);
  return Ku;
}

float Reflow_Autotune_Tu() {
  float Tu = (float)(at_data.t_up + at_data.t_down) / time.tick_per_second;
  return Tu;
}

bool Reflow_RunAutotune(float meastemp,
                        uint8_t* pheat,
                        uint8_t* pfan,
                        int32_t setpoint) {

  uint16_t minOnTime = 4 * time.tick_per_second;
  plotTemperature(time.tick, meastemp);

  at_data.max = MATH_MAX(at_data.max, meastemp);
  at_data.min = MATH_MIN(at_data.min, meastemp);

  if(at_data.iter < 0) {
    Reflow_at_startHeat(pheat, pfan);
    at_data.iter = 0;
    return false;
  }

  if(at_data.rampUp == true) {
    int32_t out = MATH_CLAMP(at_data.amplitude + at_data.bias, -255, 255);
    Reflow_setOuput(out, pheat, pfan);
    if((time.tick - at_data.t_last) < minOnTime ||
       meastemp < at_data.Target_high) {
      // not on enough or temp is too low
      return false;
    }

    at_data.t_up   = time.tick - at_data.t_last;
    at_data.t_last = time.tick;

    // start cooling
    at_data.rampUp = false;
    at_data.min    = meastemp;

    Reflow_at_startCool(pheat, pfan);
    return false;
  } else {
    int32_t out = MATH_CLAMP(at_data.bias - at_data.amplitude, -255, 255);
    Reflow_setOuput(out, pheat, pfan);
    if((time.tick - at_data.t_last) < minOnTime ||
       meastemp > at_data.Target_low) {
      return false;
    }

    at_data.t_down = time.tick - at_data.t_last;
    at_data.t_last = time.tick;
    at_data.iter += 1;
    printf("\nFinished cycle %d/%d\n", at_data.iter, AT_CYCLES);

    float rel_diff = 2.0 * fabs(at_data.t_up - at_data.t_down) /
                     (at_data.t_up + at_data.t_down);

    if(rel_diff > 0.05) {
      at_data.on_target = 0;
    } else {
      at_data.on_target++;
    }

    printf("Up: %.2fs Down: %.2fs diff: %.2f%% @cycle=%d\n",
           (float)at_data.t_up / time.tick_per_second,
           (float)at_data.t_down / time.tick_per_second, rel_diff * 100.0,
           at_data.iter - 1);

    if(at_data.iter > 1) {
      printf("Got Ku=%.3f Tu=%.3fs @cycle=%d\n", Reflow_Autotune_Ku(),
             Reflow_Autotune_Tu(), at_data.iter - 1);
    }

    if(at_data.on_target >= 2) {
      printf("Quitting cycles early\n");
      return true;
    }

    if(at_data.iter >= AT_CYCLES) {
      return true;
    }

    if(at_data.iter > 1) {
      at_data.bias += (at_data.amplitude * (at_data.t_up - at_data.t_down)) /
                      (at_data.t_up + at_data.t_down);
      // we ensure that we have a bit of heating or cooling at each cycle
      at_data.bias = MATH_CLAMP(at_data.bias, -127, 127);

      // Amplitude is chosen to always maximize cooling or heating
      if(at_data.bias >= 0) { // maximize heating
        at_data.amplitude = 255 - at_data.bias;
      } else { // maximize cooling
        at_data.amplitude = 255 + at_data.bias;
      }
      printf(" + new values: bias=%d amplitude=%d @cycle=%d\n", at_data.bias,
             at_data.amplitude, at_data.iter - 1);
    }

    // start heating
    Reflow_at_startHeat(pheat, pfan);
    return false;
  }
}
