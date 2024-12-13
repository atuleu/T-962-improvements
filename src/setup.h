#ifndef SETUP_H_
#define SETUP_H_

#include "nvstorage.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  const char* formatstr;
  const NVItem_t nvval;
  const uint16_t minval;
  const uint16_t maxval;
  const int16_t offset;
  const float multiplier;
  const bool word;
} setupMenuStruct;

#define SETTINGS_PID_KP 6
#define SETTINGS_PID_KI 7
#define SETTINGS_PID_KD 8

int Setup_getNumItems(void);
float Setup_getValue(int item);
void Setup_setRealValue(int item, float value);
void Setup_increaseValue(int item, int amount);
void Setup_decreaseValue(int item, int amount);
void Setup_printFormattedValue(int item);
int Setup_snprintFormattedValue(char* buf, int n, int item);

#endif /* SETUP_H_ */
