#include "drs_app.h"
#include "DRSController.hpp"

// Static global controller instance
static DrsController g_drs_controller;

// Convert C++ DrsState to C enum
static DrsState_C state_to_c(DrsState state)
{
    switch (state) {
        case DrsState::CLOSED:  return DRS_STATE_CLOSED;
        case DrsState::OPENING: return DRS_STATE_OPENING;
        case DrsState::OPEN:    return DRS_STATE_OPEN;
        case DrsState::CLOSING: return DRS_STATE_CLOSING;
        default:                return DRS_STATE_CLOSED;
    }
}

extern "C" {

void DrsApp_Init(void)
{
    // Initialize with default config (values from DrsConfig struct defaults)
    DrsConfig cfg;
    cfg.open_speed_kph = 40.0f;
    cfg.close_speed_kph = 35.0f;
    cfg.servo_settle_ms = 300;
    cfg.input_timeout_ms = 250;
    cfg.open_angle_deg = 60;
    cfg.close_angle_deg = 0;
    
    g_drs_controller.init(cfg);
}

void DrsApp_Tick10ms(void)
{
    g_drs_controller.tick_10ms();
}

void DrsApp_SetInputs(bool enable, bool brake, float speed_kph, bool ext_fault)
{
    DrsInputs in;
    in.enable = enable;
    in.brake_pressed = brake;
    in.speed_kph = speed_kph;
    in.external_fault = ext_fault;
    
    g_drs_controller.set_inputs(in);
}

DrsOutputs_C DrsApp_GetOutputs(void)
{
    DrsOutputs cpp_out = g_drs_controller.get_outputs();
    DrsOutputs_C c_out;
    c_out.servo_open_cmd = cpp_out.servo_open_cmd;
    c_out.servo_close_cmd = cpp_out.servo_close_cmd;
    c_out.target_angle_deg = cpp_out.target_angle_deg;
    c_out.fault_flags = cpp_out.fault_flags;
    return c_out;
}

DrsState_C DrsApp_GetState(void)
{
    return state_to_c(g_drs_controller.state());
}

} // extern "C"

