#include "DRSController.hpp"

static inline uint16_t ms_from_ticks(uint16_t ticks_10ms) {
    return (uint16_t)(ticks_10ms * 10u);
}

void DrsController::init(const DrsConfig& cfg)
{
    cfg_                 = cfg;
    faults_              = DRS_FAULT_NONE;
    servo_at_target_     = false;
    ticks_in_state_      = 0;
    ticks_since_inputs_  = 0;
    in_                  = {};
    out_                 = {};

    enter_state(DrsState::CLOSED);
}

void DrsController::set_config(const DrsConfig& cfg)
{
    cfg_ = cfg;
}

void DrsController::set_inputs(const DrsInputs& in)
{
    in_ = in;
    ticks_since_inputs_ = 0;
}

void DrsController::notify_servo_at_target(bool at_target)
{
    servo_at_target_ = at_target;
}

void DrsController::tick_10ms()
{
    // Reset one-shot outputs each tick.
    out_.servo_open_cmd  = false;
    out_.servo_close_cmd = false;
    // target_angle_deg and fault_flags will be (re)latched below as needed.

    // Time Tracker
    ticks_in_state_++;
    ticks_since_inputs_++;

    // Keep faults up to date; these influence transitions immediately.
    update_faults(/*dt_ms=*/10);

    // Fast safety check
    const bool must_close_now =
        ((faults_ & (DRS_FAULT_INPUT_TIMEOUT | DRS_FAULT_DISABLED)) != 0) ||
        (in_.external_fault) || (!in_.enable);

    switch (state_) {

    case DrsState::CLOSED:
    {
        if (must_close_now) {
            break;
        }

        // Open when: valid inputs, not braking, and above open-speed threshold.
        const bool can_open =
            inputs_valid() &&
            (!in_.brake_pressed) &&
            (in_.speed_kph > cfg_.open_speed_kph);

        if (can_open) {
            out_.servo_open_cmd   = true;
            out_.target_angle_deg = cfg_.open_angle_deg;
            enter_state(DrsState::OPENING);
        }
        break;
    }

    case DrsState::OPENING:
    {
        // Abort opening and close immediately on any mandatory-close condition.
        const bool should_close =
            must_close_now ||
            (in_.brake_pressed) ||
            (in_.speed_kph < cfg_.close_speed_kph);

        if (should_close) {
            out_.servo_close_cmd  = true;
            out_.target_angle_deg = cfg_.close_angle_deg;
            enter_state(DrsState::CLOSING);
            break;
        }

        const bool settle_elapsed = (time_in_state_ms() >= cfg_.servo_settle_ms);
        if (servo_at_target_ || settle_elapsed) {
            enter_state(DrsState::OPEN);
        }
        break;
    }

    case DrsState::OPEN:
    {
        // Close when: disabled/fault OR braking OR below close-speed threshold.
        const bool should_close =
            must_close_now ||
            (in_.brake_pressed) ||
            (in_.speed_kph < cfg_.close_speed_kph);

        if (should_close) {
            out_.servo_close_cmd  = true;
            out_.target_angle_deg = cfg_.close_angle_deg;
            enter_state(DrsState::CLOSING);
        }
        break;
    }

    case DrsState::CLOSING:
    {
        const bool settle_elapsed = (time_in_state_ms() >= cfg_.servo_settle_ms);
        if (servo_at_target_ || settle_elapsed) {
            enter_state(DrsState::CLOSED);
        }
        break;
    }

    } 

    // Servo motion watchdog: if we linger too long in a motion state, flag it.
    const bool in_motion = (state_ == DrsState::OPENING) || (state_ == DrsState::CLOSING);
    if (in_motion && (time_in_state_ms() > (uint16_t)(2u * cfg_.servo_settle_ms))) {
        faults_ |= DRS_FAULT_SERVO;

        // Favor safety: command closing if not already doing so.
        out_.servo_close_cmd  = true;
        out_.target_angle_deg = cfg_.close_angle_deg;

        if (state_ != DrsState::CLOSING) {
            enter_state(DrsState::CLOSING);
        }
    }

    out_.fault_flags = faults_;
}

void DrsController::force_close()
{
    out_.servo_close_cmd  = true;
    out_.target_angle_deg = cfg_.close_angle_deg;
    enter_state(DrsState::CLOSING);
}

void DrsController::disable()
{
    faults_ |= DRS_FAULT_DISABLED;
    force_close();
}

void DrsController::clear_faults()
{
    faults_ &= (uint8_t)~(DRS_FAULT_SERVO | DRS_FAULT_INPUT_TIMEOUT | DRS_FAULT_DISABLED);
}

void DrsController::enter_state(DrsState s)
{
    state_ = s;
    ticks_in_state_ = 0;

    if (s == DrsState::OPENING) {
        out_.target_angle_deg = cfg_.open_angle_deg;
    } else if (s == DrsState::CLOSING) {
        out_.target_angle_deg = cfg_.close_angle_deg;
    }
}

void DrsController::update_faults(uint16_t /*dt_ms*/)
{
    // Input timeout: if we have not seen set_inputs() in time, assert the bit.
    const uint16_t ms_since_inputs = ms_from_ticks(ticks_since_inputs_);
    if (ms_since_inputs > cfg_.input_timeout_ms) {
        faults_ |= DRS_FAULT_INPUT_TIMEOUT;
    } else {
        faults_ &= (uint8_t)~DRS_FAULT_INPUT_TIMEOUT;
    }

    // Maintain DISABLED bit based on current enable flag.
    if (!in_.enable) {
        faults_ |= DRS_FAULT_DISABLED;
    } else {
        faults_ &= (uint8_t)~DRS_FAULT_DISABLED;
    }

    // DRS_FAULT_SERVO is handled in the motion watchdog inside tick_10ms().
}

bool DrsController::inputs_valid() const
{
    // Minimum conditions to consider opening:
    const bool no_timeout   = ((faults_ & DRS_FAULT_INPUT_TIMEOUT) == 0);
    const bool not_disabled = ((faults_ & DRS_FAULT_DISABLED)     == 0);
    const bool no_ext_fault = (!in_.external_fault);

    return in_.enable && no_timeout && not_disabled && no_ext_fault;
}
