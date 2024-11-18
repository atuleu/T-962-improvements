/*
 * setup.c - T-962 reflow controller
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

#include "setup.h"
#include "nvstorage.h"
#include "reflow_profiles.h"
#include <stdint.h>
#include <stdio.h>

static setupMenuStruct setupmenu[] = {
    {"Min fan speed    %4.0f", REFLOW_MIN_FAN_SPEED, 0, 254, 0, 1.0f, false},
    {"Cycle done beep %4.1fs", REFLOW_BEEP_DONE_LEN, 0, 254, 0, 0.1f, false},
    {"Left TC gain     %1.2f", TC_LEFT_GAIN, 10, 190, 0, 0.01f, false},
    {"Left TC offset  %+1.2f", TC_LEFT_OFFSET, 0, 200, -100, 0.25f, false},
    {"Right TC gain    %1.2f", TC_RIGHT_GAIN, 10, 190, 0, 0.01f, false},
    {"Right TC offset %+1.2f", TC_RIGHT_OFFSET, 0, 200, -100, 0.25f, false},
    {"PID K Constant %+1.2f", PID_K_VALUE_H, 0, 10000, 0, 0.1f, true},
    {"PID I Constant %+1.3f", PID_I_VALUE_H, 0, 1000, 0, 0.001f, true},
    {"PID D Constant %+1.2f", PID_D_VALUE_H, 0, 10000, 0, 0.1f, true},
};

#define NUM_SETUP_ITEMS (sizeof(setupmenu) / sizeof(setupmenu[0]))

int Setup_getNumItems(void) { return NUM_SETUP_ITEMS; }

int _getRawValue(int item) {
  if(setupmenu[item].word) {
    return NV_GetWord(setupmenu[item].nvval);
  }
  return NV_GetConfig(setupmenu[item].nvval);
}

float Setup_getValue(int item) {
  int intval = _getRawValue(item);
  intval += setupmenu[item].offset;
  return ((float)intval) * setupmenu[item].multiplier;
}

void _setRawValue(int item, int value) {
  if(setupmenu[item].word) {
    NV_PutWord(setupmenu[item].nvval, value);
  } else {
    NV_SetConfig(setupmenu[item].nvval, value);
  }
}

void Setup_setRealValue(int item, float value) {
  int intval = (int)(value / setupmenu[item].multiplier);
  intval -= setupmenu[item].offset;
  _setRawValue(item, intval);
}

void Setup_increaseValue(int item, int amount) {
  int curval = _getRawValue(item) + amount;

  int maxval = setupmenu[item].maxval;
  if(curval > maxval)
    curval = maxval;

  _setRawValue(item, curval);
}

void Setup_decreaseValue(int item, int amount) {
  int curval = _getRawValue(item) - amount;

  int minval = setupmenu[item].minval;
  if(curval < minval)
    curval = minval;

  _setRawValue(item, curval);
}

void Setup_printFormattedValue(int item) {
  printf(setupmenu[item].formatstr, Setup_getValue(item));
}

int Setup_snprintFormattedValue(char* buf, int n, int item) {
  return snprintf(buf, n, setupmenu[item].formatstr, Setup_getValue(item));
}
