#ifndef DRS_APP_H
#define DRS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// C-compatible DRS state enum
typedef enum {
    DRS_STATE_CLOSED = 0,
    DRS_STATE_OPENING = 1,
    DRS_STATE_OPEN = 2,
    DRS_STATE_CLOSING = 3
} DrsState_C;

// C-compatible outputs struct
typedef struct {
    bool    servo_open_cmd;
    bool    servo_close_cmd;
    uint8_t target_angle_deg;
    uint8_t fault_flags;
} DrsOutputs_C;

// C-compatible inputs struct
typedef struct {
    bool  enable;
    bool  brake_pressed;
    float speed_kph;
    bool  external_fault;
} DrsInputs_C;

// API Functions
void DrsApp_Init(void);
void DrsApp_Tick10ms(void);
void DrsApp_SetInputs(bool enable, bool brake, float speed_kph, bool ext_fault);
DrsOutputs_C DrsApp_GetOutputs(void);
DrsState_C DrsApp_GetState(void);

#ifdef __cplusplus
}
#endif

#endif // DRS_APP_H

