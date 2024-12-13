/*
 * main.c - T-962 reflow controller
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

#include "LPC214x.h"
#include "PID_v1.h"
#include "adc.h"
#include "buzzer.h"
#include "eeprom.h"
#include "i2c.h"
#include "io.h"
#include "keypad.h"
#include "lcd.h"
#include "max31855.h"
#include "nvstorage.h"
#include "onewire.h"
#include "reflow.h"
#include "reflow_profiles.h"
#include "rtc.h"
#include "sched.h"
#include "sensor.h"
#include "serial.h"
#include "setup.h"
#include "systemfan.h"
#include "utils.h"
#include "version.h"
#include "vic.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern uint8_t logobmp[];
extern uint8_t stopbmp[];
extern uint8_t selectbmp[];
extern uint8_t editbmp[];
extern uint8_t f3editbmp[];

// No version.c file generated for LPCXpresso builds, fall back to this
__attribute__((weak)) const char* Version_GetGitVersion(void) {
  return "no version info";
}

static char* format_about =
    "\nT-962-controller open source firmware (%s)"
    "\n"
    "\nSee https://github.com/UnifiedEngineering/T-962-improvement for more "
    "details."
    "\n"
    "\nInitializing improved reflow oven...";

static char* help_text =
    "\nT-962-controller serial interface.\n\n"
    " about                   Show about + debug information\n"
    " bake <setpoint>         Enter Bake mode with setpoint\n"
    " bake <setpoint> <time>  Enter Bake mode with setpoint for <time> "
    "seconds\n"
    " help                    Display help text\n"
    " list profiles           List available reflow profiles\n"
    " list settings           List machine settings\n"
    " quiet                   No logging in standby mode\n"
    " reflow                  Start reflow with selected profile\n"
    " setting <id> <value>    Set setting id to value\n"
    " select profile <id>     Select reflow profile by id\n"
    " stop                    Exit reflow or bake mode\n"
    " values                  Dump currently measured values\n"
    " autotune <k> <l> <h>    Runs PID autotuning procedure for <k> \n"
    "                         cycles between <l> and <h> temperature\n"
    "\n";

static int32_t Main_Work(void);

static char buf[22];
static int len;

int main(void) {

  IO_JumpBootloader();

  PLLCFG =
      (1 << 5) |
      (4 << 0); // PLL MSEL=0x4 (+1), PSEL=0x1 (/2) so 11.0592*5 = 55.296MHz,
                // Fcco = (2x55.296)*2 = 221MHz which is within 156 to 320MHz
  PLLCON  = 0x01;
  PLLFEED = 0xaa;
  PLLFEED = 0x55; // Feed complete
  while(!(PLLSTAT & (1 << 10)))
    ; // Wait for PLL to lock
  PLLCON  = 0x03;
  PLLFEED = 0xaa;
  PLLFEED = 0x55; // Feed complete
  VPBDIV  = 0x01; // APB runs at the same frequency as the CPU (55.296MHz)
  MAMTIM  = 0x03; // 3 cycles flash access recommended >40MHz
  MAMCR   = 0x02; // Fully enable memory accelerator

  VIC_Init();
  Sched_Init();
  IO_Init();
  Set_Heater(0);
  Set_Fan(0);
  Serial_Init();
  printf(format_about, Version_GetGitVersion());

  I2C_Init();
  EEPROM_Init();
  NV_Init();

  LCD_Init();
  LCD_BMPDisplay(logobmp, 0, 0);

  IO_InitWatchdog();
  IO_PrintResetReason();

  len = IO_Partinfo(buf, sizeof(buf), "%s rev %c");
  LCD_disp_str((uint8_t*)buf, len, 0, 64 - 6, FONT6X6);
  printf("\nRunning on an %s", buf);

  len = snprintf(buf, sizeof(buf), "%s", Version_GetGitVersion());
  LCD_disp_str((uint8_t*)buf, len, 128 - (len * 6), 0, FONT6X6);

  LCD_FB_Update();
  Keypad_Init();
  Buzzer_Init();
  ADC_Init();
  RTC_Init();
  OneWire_Init();
  SPI_TC_Init();
  Reflow_Init();
  SystemFan_Init();

  Sched_SetWorkfunc(MAIN_WORK, Main_Work);
  Sched_SetState(MAIN_WORK, 1, TICKS_SECS(2)); // Enable in 2 seconds

  Buzzer_Beep(BUZZ_1KHZ, 255, TICKS_MS(100));

  while(1) {
#ifdef ENABLE_SLEEP
    int32_t sleeptime;
    sleeptime = Sched_Do(0); // No fast-forward support
    // printf("\n%d ticks 'til next activity"),sleeptime);
#else
    Sched_Do(0); // No fast-forward support
#endif
  }
  return 0;
}

typedef enum eMainMode
{
  MAIN_HOME = 0,
  MAIN_ABOUT,
  MAIN_SETUP,
  MAIN_BAKE,
  MAIN_SELECT_PROFILE,
  MAIN_EDIT_PROFILE,
  MAIN_REFLOW,
  MAIN_AUTOTUNE,
} MainMode_t;

typedef struct {
  // general
  uint32_t keyspressed;
  MainMode_t mode;
  int32_t retval;

  // profile editing
  uint8_t profile_time_idx;
  uint8_t current_edit_profile;

  // Bake
  int setpoint;
  int timer;
} MainData_t;

void Main_Setup(MainData_t* data);
void Main_About(MainData_t* data);
void Main_Reflow(MainData_t* data);
void Main_SelectProfile(MainData_t* data);
void Main_Bake(MainData_t* data);
void Main_EditProfile(MainData_t* data);
void Main_Home(MainData_t* data);
void Main_Autotune(MainData_t* data);

void ProcessUART(MainData_t* data);

static int32_t Main_Work(void) {

  static MainData_t app = {
      .mode             = MAIN_HOME,
      .retval           = TICKS_MS(500),
      .profile_time_idx = 0,
      .setpoint         = 0,
      .timer            = 0,
  };

  if(app.setpoint == 0) {
    Reflow_LoadSetpoint();
    app.setpoint = Reflow_GetSetpoint();
  }

  app.keyspressed = Keypad_Get();

  ProcessUART(&app);

  switch(app.mode) {
  case MAIN_SETUP:
    Main_Setup(&app);
    break;
  case MAIN_ABOUT:
    Main_About(&app);
    break;
  case MAIN_REFLOW:
    Main_Reflow(&app);
    break;
  case MAIN_SELECT_PROFILE:
    Main_SelectProfile(&app);
    break;
  case MAIN_BAKE:
    Main_Bake(&app);
    break;
  case MAIN_EDIT_PROFILE:
    Main_EditProfile(&app);
    break;
  case MAIN_HOME:
    Main_Home(&app);
    break;
  case MAIN_AUTOTUNE:
    Main_Autotune(&app);
    break;
  }

  LCD_FB_Update();

  return app.retval;
}

void Main_Setup(MainData_t* data) {
  static uint8_t selected = 0;
  int y                   = 0;

  int keyrepeataccel = data->keyspressed >> 17; // Divide the value by 2
  if(keyrepeataccel < 1)
    keyrepeataccel = 1;
  if(keyrepeataccel > 30)
    keyrepeataccel = 30;

  if(data->keyspressed & KEY_F1) {
    if(selected > 0) { // Prev row
      selected--;
    } else { // wrap
      selected = Setup_getNumItems() - 1;
    }
  }
  if(data->keyspressed & KEY_F2) {
    if(selected < (Setup_getNumItems() - 1)) { // Next row
      selected++;
    } else { // wrap
      selected = 0;
    }
  }

  if(data->keyspressed & KEY_F3) {
    Setup_decreaseValue(selected, keyrepeataccel);
  }
  if(data->keyspressed & KEY_F4) {
    Setup_increaseValue(selected, keyrepeataccel);
  }

  LCD_FB_Clear();
  len = snprintf(buf, sizeof(buf), "Setup/calibration");
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), y, FONT6X6);
  y += 7;

#define MAX_DISPLAYED_ITEMS 4
  int startidx = 0;
  int endidx   = MATH_MIN(4, Setup_getNumItems());
  if(selected >= MAX_DISPLAYED_ITEMS) {
    startidx = MAX_DISPLAYED_ITEMS;
    endidx   = MATH_MIN(Setup_getNumItems(), 2 * MAX_DISPLAYED_ITEMS - 1);

    while(endidx <= selected) {
      startidx += MAX_DISPLAYED_ITEMS - 1;
      endidx = MATH_MIN(Setup_getNumItems(), endidx + MAX_DISPLAYED_ITEMS - 1);
    }
  }

  if(startidx > 0) {
    LCD_disp_str((uint8_t*)"...", 3, 0, y, FONT6X6);
    y += 7;
  }

  for(int i = startidx; i < endidx; i++) {
    len = Setup_snprintFormattedValue(buf, sizeof(buf), i);
    LCD_disp_str((uint8_t*)buf, len, 0, y,
                 FONT6X6 | ((selected == i) ? INVERT : 0));
    y += 7;
  }

  if(endidx < Setup_getNumItems()) {
    LCD_disp_str((uint8_t*)"...", 3, 0, y, FONT6X6);
    y += 7;
  }

  // buttons
  y = 64 - 7;
  LCD_disp_str((uint8_t*)" < ", 3, 0, y, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)" > ", 3, 20, y, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)" - ", 3, 45, y, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)" + ", 3, 65, y, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)" DONE ", 6, 91, y, FONT6X6 | INVERT);

  // Leave setup
  if(data->keyspressed & KEY_S) {
    data->mode = MAIN_HOME;
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0; // Force immediate refresh
  }
}

void Main_About(MainData_t* data) {
  LCD_FB_Clear();
  LCD_BMPDisplay(logobmp, 0, 0);

  len = snprintf(buf, sizeof(buf), "T-962 controller");
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), 0, FONT6X6);

  len = snprintf(buf, sizeof(buf), "%s", Version_GetGitVersion());
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), 64 - 6, FONT6X6);

  LCD_BMPDisplay(stopbmp, 127 - 17, 0);

  // Leave about with any key.
  if(data->keyspressed & KEY_ANY) {
    data->mode   = MAIN_HOME;
    data->retval = 0; // Force immediate refresh
  }
}

void Main_Reflow(MainData_t* data) {
  uint32_t ticks = RTC_Read();

  len = snprintf(buf, sizeof(buf), "%03u", Reflow_GetSetpoint());
  LCD_disp_str((uint8_t*)"SET", 3, 110, 7, FONT6X6);
  LCD_disp_str((uint8_t*)buf, len, 110, 13, FONT6X6);

  len = snprintf(buf, sizeof(buf), "%03u", Reflow_GetActualTemp());
  LCD_disp_str((uint8_t*)"ACT", 3, 110, 20, FONT6X6);
  LCD_disp_str((uint8_t*)buf, len, 110, 26, FONT6X6);

  len = snprintf(buf, sizeof(buf), "%03u", (unsigned int)ticks);
  LCD_disp_str((uint8_t*)"RUN", 3, 110, 33, FONT6X6);
  LCD_disp_str((uint8_t*)buf, len, 110, 39, FONT6X6);

  // Abort reflow
  if(Reflow_IsDone() || data->keyspressed & KEY_S) {
    printf("\nReflow %s\n",
           (Reflow_IsDone() ? "done" : "interrupted by keypress"));
    if(Reflow_IsDone()) {
      Buzzer_Beep(BUZZ_1KHZ, 255,
                  TICKS_MS(100) * NV_GetConfig(REFLOW_BEEP_DONE_LEN));
    }
    data->mode = MAIN_HOME;
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0; // Force immediate refresh
  }
}

void Main_SelectProfile(MainData_t* data) {
  int curprofile = Reflow_GetProfileIdx();
  LCD_FB_Clear();

  // Prev profile
  if(data->keyspressed & KEY_F1) {
    curprofile--;
  }
  // Next profile
  if(data->keyspressed & KEY_F2) {
    curprofile++;
  }

  Reflow_SelectProfileIdx(curprofile);

  Reflow_PlotProfile(-1);
  LCD_BMPDisplay(selectbmp, 127 - 17, 0);
  int eeidx = Reflow_GetEEProfileIdx();
  if(eeidx) { // Display edit button
    LCD_BMPDisplay(f3editbmp, 127 - 17, 29);
  }
  len = snprintf(buf, sizeof(buf), "%s", Reflow_GetProfileName());
  LCD_disp_str((uint8_t*)buf, len, 13, 0, FONT6X6);

  if(eeidx && data->keyspressed & KEY_F3) { // Edit ee profile
    data->mode                 = MAIN_EDIT_PROFILE;
    data->current_edit_profile = eeidx;
    data->retval               = 0; // Force immediate refresh
  }

  // Select current profile
  if(data->keyspressed & KEY_S) {
    data->mode   = MAIN_HOME;
    data->retval = 0; // Force immediate refresh
  }
}

void Main_Bake(MainData_t* data) {

  LCD_FB_Clear();
  LCD_disp_str((uint8_t*)"MANUAL/BAKE MODE", 16, 0, 0, FONT6X6);

  int keyrepeataccel = data->keyspressed >> 17; // Divide the value by 2
  if(keyrepeataccel < 1)
    keyrepeataccel = 1;
  if(keyrepeataccel > 30)
    keyrepeataccel = 30;

  // Setpoint-
  if(data->keyspressed & KEY_F1) {
    data->setpoint = MATH_MAX(SETPOINT_MIN, data->setpoint - keyrepeataccel);
  }

  // Setpoint+
  if(data->keyspressed & KEY_F2) {
    data->setpoint += MATH_MIN(SETPOINT_MAX, data->setpoint + keyrepeataccel);
  }

  // timer --
  if(data->keyspressed & KEY_F3) {
    if(data->timer - keyrepeataccel < 0) {
      // infinite bake
      data->timer = -1;
    } else {
      data->timer -= keyrepeataccel;
    }
  }

  // timer ++
  if(data->keyspressed & KEY_F4) {
    data->timer += keyrepeataccel;
  }

  int y           = 10;
  // display F1 button only if setpoint can be decreased
  char f1function = ' ';
  if(data->setpoint > SETPOINT_MIN) {
    LCD_disp_str((uint8_t*)"F1", 2, 0, y, FONT6X6 | INVERT);
    f1function = '-';
  }
  // display F2 button only if setpoint can be increased
  char f2function = ' ';
  if(data->setpoint < SETPOINT_MAX) {
    LCD_disp_str((uint8_t*)"F2", 2, LCD_ALIGN_RIGHT(2), y, FONT6X6 | INVERT);
    f2function = '+';
  }
  len = snprintf(buf, sizeof(buf), "%c SETPOINT %d` %c", f1function,
                 (int)data->setpoint, f2function);
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), y, FONT6X6);

  y = 18;
  if(data->timer == 0) {
    len = snprintf(buf, sizeof(buf), "inf TIMER stop +");
  } else if(data->timer < 0) {
    len = snprintf(buf, sizeof(buf), "no timer    stop");
  } else {
    len = snprintf(buf, sizeof(buf), "- TIMER %3d:%02d +", data->timer / 60,
                   data->timer % 60);
  }
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), y, FONT6X6);

  if(data->timer >= 0) {
    LCD_disp_str((uint8_t*)"F3", 2, 0, y, FONT6X6 | INVERT);
  }
  LCD_disp_str((uint8_t*)"F4", 2, LCD_ALIGN_RIGHT(2), y, FONT6X6 | INVERT);

  y = 26;
  if(data->timer > 0) {
    int time_left = Reflow_GetTimeLeft();
    if(Reflow_IsPreheating()) {
      len = snprintf(buf, sizeof(buf), "PREHEAT");
    } else if(Reflow_IsDone() || time_left < 0) {
      len = snprintf(buf, sizeof(buf), "DONE");
    } else {
      len =
          snprintf(buf, sizeof(buf), "%d:%02d", time_left / 60, time_left % 60);
    }
    LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_RIGHT(len), y, FONT6X6);
  }

  len = snprintf(buf, sizeof(buf), "ACT %3.1f`", Sensor_GetTemp(TC_AVERAGE));
  LCD_disp_str((uint8_t*)buf, len, 0, y, FONT6X6);

  y   = 34;
  len = snprintf(buf, sizeof(buf), "  L %3.1f`", Sensor_GetTemp(TC_LEFT));
  LCD_disp_str((uint8_t*)buf, len, 0, y, FONT6X6);
  len = snprintf(buf, sizeof(buf), "  R %3.1f`", Sensor_GetTemp(TC_RIGHT));
  LCD_disp_str((uint8_t*)buf, len, LCD_CENTER, y, FONT6X6);

  if(Sensor_IsValid(TC_EXTRA1) || Sensor_IsValid(TC_EXTRA2)) {
    y = 42;
    if(Sensor_IsValid(TC_EXTRA1)) {
      len = snprintf(buf, sizeof(buf), " X1 %3.1f`", Sensor_GetTemp(TC_EXTRA1));
      LCD_disp_str((uint8_t*)buf, len, 0, y, FONT6X6);
    }
    if(Sensor_IsValid(TC_EXTRA2)) {
      len = snprintf(buf, sizeof(buf), " X2 %3.1f`", Sensor_GetTemp(TC_EXTRA2));
      LCD_disp_str((uint8_t*)buf, len, LCD_CENTER, y, FONT6X6);
    }
  }

  y   = 50;
  len = snprintf(buf, sizeof(buf), "COLDJUNCTION");
  LCD_disp_str((uint8_t*)buf, len, 0, y, FONT6X6);

  y += 8;
  if(Sensor_IsValid(TC_COLD_JUNCTION)) {
    len =
        snprintf(buf, sizeof(buf), "%3.1f`", Sensor_GetTemp(TC_COLD_JUNCTION));
  } else {
    len = snprintf(buf, sizeof(buf), "NOT PRESENT");
  }
  LCD_disp_str((uint8_t*)buf, len, (12 * 6) - (len * 6), y, FONT6X6);

  LCD_BMPDisplay(stopbmp, 127 - 17, 0);

  Reflow_SetSetpoint(data->setpoint);

  if(data->timer > 0 && Reflow_IsDone()) {
    Buzzer_Beep(BUZZ_1KHZ, 255,
                TICKS_MS(100) * NV_GetConfig(REFLOW_BEEP_DONE_LEN));
    Reflow_SetBakeTimer(0);
    Reflow_SetMode(REFLOW_STANDBY);
  }

  if(data->keyspressed & KEY_F3 || data->keyspressed & KEY_F4) {
    if(data->timer == 0) {
      Reflow_SetMode(REFLOW_STANDBY);
    } else {
      if(data->timer == -1) {
        Reflow_SetBakeTimer(0);
      } else if(data->timer > 0) {
        Reflow_SetBakeTimer(data->timer);
        printf("\nSetting bake timer to %d\n", data->timer);
      }
      Reflow_SetMode(REFLOW_BAKE);
    }
  }

  // Abort bake
  if(data->keyspressed & KEY_S) {
    printf("\nEnd bake mode by keypress\n");

    data->mode = MAIN_HOME;
    Reflow_SetBakeTimer(0);
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0; // Force immediate refresh
  }
}

void Main_EditProfile(MainData_t* data) {
  LCD_FB_Clear();
  int keyrepeataccel = data->keyspressed >> 17; // Divide the value by 2
  if(keyrepeataccel < 1)
    keyrepeataccel = 1;
  if(keyrepeataccel > 30)
    keyrepeataccel = 30;

  int16_t cursetpoint;
  Reflow_SelectEEProfileIdx(data->current_edit_profile);
  if(data->keyspressed & KEY_F1 && data->profile_time_idx > 0) { // Prev time
    data->profile_time_idx--;
  }
  if(data->keyspressed & KEY_F2 && data->profile_time_idx < 47) { // Next time
    data->profile_time_idx++;
  }
  cursetpoint = Reflow_GetSetpointAtIdx(data->profile_time_idx);

  if(data->keyspressed & KEY_F3) { // Decrease setpoint
    cursetpoint -= keyrepeataccel;
  }
  if(data->keyspressed & KEY_F4) { // Increase setpoint
    cursetpoint += keyrepeataccel;
  }
  if(cursetpoint < 0)
    cursetpoint = 0;
  if(cursetpoint > SETPOINT_MAX)
    cursetpoint = SETPOINT_MAX;
  Reflow_SetSetpointAtIdx(data->profile_time_idx, cursetpoint);

  Reflow_PlotProfile(data->profile_time_idx);
  LCD_BMPDisplay(editbmp, 127 - 17, 0);

  len = snprintf(buf, sizeof(buf), "%02u0s %03u`", data->profile_time_idx,
                 cursetpoint);
  LCD_disp_str((uint8_t*)buf, len, 13, 0, FONT6X6);

  // Done editing
  if(data->keyspressed & KEY_S) {
    Reflow_SaveEEProfile();
    data->mode   = MAIN_HOME;
    data->retval = 0; // Force immediate refresh
  }
}

void Main_Home(MainData_t* data) {
  LCD_FB_Clear();

  len = snprintf(buf, sizeof(buf), "MAIN MENU");
  LCD_disp_str((uint8_t*)buf, len, 0, 6 * 0, FONT6X6);
  LCD_disp_str((uint8_t*)"F1", 2, 0, 8 * 1, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)"ABOUT", 5, 14, 8 * 1, FONT6X6);
  LCD_disp_str((uint8_t*)"F2", 2, 0, 8 * 2, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)"SETUP", 5, 14, 8 * 2, FONT6X6);
  LCD_disp_str((uint8_t*)"F3", 2, 0, 8 * 3, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)"BAKE/MANUAL MODE", 16, 14, 8 * 3, FONT6X6);
  LCD_disp_str((uint8_t*)"F4", 2, 0, 8 * 4, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)"SELECT PROFILE", 14, 14, 8 * 4, FONT6X6);
  LCD_disp_str((uint8_t*)"S", 1, 3, 8 * 5, FONT6X6 | INVERT);
  LCD_disp_str((uint8_t*)"RUN REFLOW PROFILE", 18, 14, 8 * 5, FONT6X6);

  len = snprintf(buf, sizeof(buf), "%s", Reflow_GetProfileName());
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), 8 * 6,
               FONT6X6 | INVERT);

  len = snprintf(buf, sizeof(buf), "OVEN TEMPERATURE %d`",
                 Reflow_GetActualTemp());
  LCD_disp_str((uint8_t*)buf, len, LCD_ALIGN_CENTER(len), 64 - 6, FONT6X6);

  // Make sure reflow complete beep is silenced when pressing any key
  if(data->keyspressed) {
    Buzzer_Beep(BUZZ_NONE, 0, 0);
  }

  // About
  if(data->keyspressed & KEY_F1) {
    data->mode   = MAIN_ABOUT;
    data->retval = 0; // Force immediate refresh
  }
  if(data->keyspressed & KEY_F2) { // Setup/cal
    data->mode = MAIN_SETUP;
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0; // Force immediate refresh
  }

  // Bake mode
  if(data->keyspressed & KEY_F3) {
    data->mode = MAIN_BAKE;
    Reflow_Init();
    data->retval = 0; // Force immediate refresh
  }

  // Select profile
  if(data->keyspressed & KEY_F4) {
    data->mode   = MAIN_SELECT_PROFILE;
    data->retval = 0; // Force immediate refresh
  }

  // Start reflow
  if(data->keyspressed & KEY_S) {
    data->mode = MAIN_REFLOW;
    LCD_FB_Clear();
    printf("\nStarting reflow with profile: %s", Reflow_GetProfileName());
    Reflow_Init();
    Reflow_PlotProfile(-1);
    LCD_BMPDisplay(stopbmp, 127 - 17, 0);
    len = snprintf(buf, sizeof(buf), "%s", Reflow_GetProfileName());
    LCD_disp_str((uint8_t*)buf, len, 13, 0, FONT6X6);
    Reflow_SetMode(REFLOW_REFLOW);
    data->retval = 0; // Force immediate refresh
  }
}

void ProcessUART(MainData_t* data) {
  static char serial_cmd[255] = "";
  char* cmd_select_profile    = "select profile %d";
  char* cmd_bake              = "bake %d %d";
  char* cmd_dump_profile      = "dump profile %d";
  char* cmd_setting           = "setting %d %f";
  char* cmd_autotune          = "PID autotune %d %d %f";

  static int read = 0;

  if(uart_isrxready() == false) {
    return;
  }

  int len = uart_read(&serial_cmd[read], 255 - read);

  if(len == 0) {
    return;
  }
  read += len;

  bool eol = serial_cmd[read - 1] == '\n' || serial_cmd[read - 1] == '\r';

  if(read >= 255 && eol == false) {
    printf("\ninput buffer overflow. please be gentle\n");
    read = 0;
    return;
  }

  if(eol == false) {
    // keep growing the buffer
    return;
  }

  read             = MATH_CLAMP(read - 1, 0, 254);
  serial_cmd[read] = '\0'; // replace EOL by NULLCHAR
  // reset our read for next iteration buffer
  read             = 0;

  int param, param1;
  float paramF;

  if(strcmp(serial_cmd, "about") == 0) {
    printf(format_about, Version_GetGitVersion());
    len = IO_Partinfo(buf, sizeof(buf), "\nPart number: %s rev %c\n");
    printf("%s", buf);
    EEPROM_Dump();

    printf("\nSensor values:\n");
    Sensor_ListAll();

  } else if(strcmp(serial_cmd, "help") == 0 || strcmp(serial_cmd, "?") == 0) {
    printf("%s", help_text);

  } else if(strcmp(serial_cmd, "list profiles") == 0) {
    printf("\nReflow profiles available:\n");

    Reflow_ListProfiles();
    printf("\n");

  } else if(strcmp(serial_cmd, "reflow") == 0) {
    printf("\nStarting reflow with profile: %s\n", Reflow_GetProfileName());
    data->mode        = MAIN_HOME;
    // this is a bit dirty, but with the least code duplication.
    data->keyspressed = KEY_S;

  } else if(strcmp(serial_cmd, "list settings") == 0) {
    printf("\nCurrent settings:\n\n");
    for(int i = 0; i < Setup_getNumItems(); i++) {
      printf("%d: ", i);
      Setup_printFormattedValue(i);
      printf("\n");
    }

  } else if(strcmp(serial_cmd, "stop") == 0) {
    printf("\nStopping bake/reflow");
    data->mode = MAIN_HOME;
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0;

  } else if(strcmp(serial_cmd, "quiet") == 0) {
    Reflow_ToggleStandbyLogging();
    printf("\nToggled standby logging\n");

  } else if(strcmp(serial_cmd, "values") == 0) {
    printf("\nActual measured values:\n");
    Sensor_ListAll();
    printf("\n");

  } else if(sscanf(serial_cmd, cmd_select_profile, &param) > 0) {
    // select profile
    Reflow_SelectProfileIdx(param);
    printf("\nSelected profile %d: %s\n", param, Reflow_GetProfileName());

  } else if(sscanf(serial_cmd, cmd_bake, &param, &param1) > 0) {
    if(param < SETPOINT_MIN) {
      printf("\nSetpoint must be >= %ddegC\n", SETPOINT_MIN);
      param = SETPOINT_MIN;
    }
    if(param > SETPOINT_MAX) {
      printf("\nSetpoint must be <= %ddegC\n", SETPOINT_MAX);
      param = SETPOINT_MAX;
    }
    if(param1 < 1) {
      printf("\nTimer must be greater than 0\n");
      param1 = 1;
    }

    if(param1 < BAKE_TIMER_MAX) {
      printf("\nStarting bake with setpoint %ddegC for %ds after reaching "
             "setpoint\n",
             param, param1);
      data->timer = param1;
      Reflow_SetBakeTimer(data->timer);
    } else {
      printf("\nStarting bake with setpoint %ddegC\n", param);
    }

    data->setpoint = param;
    Reflow_SetSetpoint(data->setpoint);
    data->mode = MAIN_BAKE;
    Reflow_SetMode(REFLOW_BAKE);

  } else if(sscanf(serial_cmd, cmd_dump_profile, &param) > 0) {
    printf("\nDumping profile %d: %s\n ", param, Reflow_GetProfileName());
    Reflow_DumpProfile(param);

  } else if(sscanf(serial_cmd, cmd_setting, &param, &paramF) > 0) {
    Setup_setRealValue(param, paramF);
    printf("\nAdjusted setting: ");
    Setup_printFormattedValue(param);
  } else if(sscanf(serial_cmd, cmd_autotune, &param, &param1, &paramF) > 0) {
    Reflow_Init();
    Reflow_StartAutotune(param1, paramF, param);
    data->mode = MAIN_AUTOTUNE;
  } else {
    printf("\nCannot understand command '%s', ? for help\n", serial_cmd);
  }
}

void Main_Autotune(MainData_t* data) {
  uint32_t ticks   = RTC_Read();
  static bool once = true;

  if(Reflow_IsDone() == false && once == true) {
    len = snprintf(buf, sizeof(buf), "%03u", Reflow_GetSetpoint());
    LCD_disp_str((uint8_t*)"SET", 3, 110, 7, FONT6X6);
    LCD_disp_str((uint8_t*)buf, len, 110, 13, FONT6X6);

    len = snprintf(buf, sizeof(buf), "%03u", Reflow_GetActualTemp());
    LCD_disp_str((uint8_t*)"ACT", 3, 110, 20, FONT6X6);
    LCD_disp_str((uint8_t*)buf, len, 110, 26, FONT6X6);

    len = snprintf(buf, sizeof(buf), "%03u", (unsigned int)ticks);
    LCD_disp_str((uint8_t*)"RUN", 3, 110, 33, FONT6X6);
    LCD_disp_str((uint8_t*)buf, len, 110, 39, FONT6X6);
  }

  if(data->keyspressed & KEY_S) {
    printf("\nAutotune interrupted by keypress\n");
    data->mode = MAIN_HOME;
    Reflow_SetMode(REFLOW_STANDBY);
    data->retval = 0;
    once         = true;
    return;
  }

  if(Reflow_IsDone() == false && once == true) {
    return;
  }

  float Kp, Ki, Kd;
  if(once == true) {
    once = false;
    printf("\nAutotune done\n");
    Reflow_SetMode(REFLOW_STANDBY); // will set reflow undone.
    Buzzer_Beep(BUZZ_1KHZ, 255,
                TICKS_MS(100) * NV_GetConfig(REFLOW_BEEP_DONE_LEN));

    LCD_FB_Clear();
    len = snprintf(buf, sizeof(buf), "Autotune Results");
    LCD_disp_str((uint8_t*)buf, len, 1, 0, FONT6X6);
    float Ku = Reflow_Autotune_Ku();
    float Tu = Reflow_Autotune_Tu();
    Kp       = Ku / 3;
    Ki       = 2 * Kp / Tu;
    Kd       = Ku * Tu / 3;

    len = snprintf(buf, sizeof(buf), "Ku=%.1f Tu=%.1fs", Ku, Tu);
    LCD_disp_str((uint8_t*)buf, len, 1, 1, FONT6X6);

    len = snprintf(buf, sizeof(buf), "Kp=%.3f", Kp);
    LCD_disp_str((uint8_t*)buf, len, 2, 1, FONT6X6);
    len = snprintf(buf, sizeof(buf), "Ki=%.3f", Ki);
    LCD_disp_str((uint8_t*)buf, len, 3, 1, FONT6X6);
    len = snprintf(buf, sizeof(buf), "Kd=%.3f", Kd);
    LCD_disp_str((uint8_t*)buf, len, 3, 1, FONT6X6);
    len = snprintf(buf, sizeof(buf), "Press F1 to save");
    LCD_disp_str((uint8_t*)buf, len, 4, 1, FONT6X6);
  }

  if(data->keyspressed & KEY_F1) {
    printf("Setting Kp=%.3f Ki=%.3f Kd=%.3f to NVStorage\n", Kp, Ki, Kd);
    Setup_setRealValue(PID_K_VALUE_H, Kp);
    Setup_setRealValue(PID_I_VALUE_H, Ki);
    Setup_setRealValue(PID_D_VALUE_H, Kd);
    Reflow_Init();
    Reflow_SetMode(REFLOW_STANDBY);

    data->mode   = MAIN_HOME;
    data->retval = 0;
    once         = true;
    return;
  }
}
