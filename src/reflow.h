#ifndef REFLOW_H_
#define REFLOW_H_

#include <stdint.h>

typedef enum eReflowMode
{
  REFLOW_INITIAL = 0,
  REFLOW_STANDBY,
  REFLOW_BAKE,
  REFLOW_REFLOW,
  REFLOW_STANDBYFAN,
  REFLOW_AUTOTUNE,
} ReflowMode_t;

#define SETPOINT_MIN (30)
#define SETPOINT_MAX (300)
#define SETPOINT_DEFAULT (30)

// 36 hours max timer
#define BAKE_TIMER_MAX (60 * 60 * 36)

void Reflow_Init(void);
void Reflow_SetMode(ReflowMode_t themode);
void Reflow_SetSetpoint(uint16_t thesetpoint);
void Reflow_LoadSetpoint(void);
int16_t Reflow_GetActualTemp(void);
uint8_t Reflow_IsDone(void);
int Reflow_IsPreheating(void);
uint16_t Reflow_GetSetpoint(void);
void Reflow_SetBakeTimer(int seconds);
int Reflow_GetTimeLeft(void);
void Reflow_ToggleStandbyLogging(void);
void Reflow_StartAutotune(uint16_t low, uint16_t high, uint8_t cycles);
float Reflow_Autotune_Ku();
float Reflow_Autotune_Tu();

#endif /* REFLOW_H_ */
