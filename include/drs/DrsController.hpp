#ifndef DRS_CONTROLLER_HPP
#define DRS_CONTROLLER_HPP

#include <stdint.h>
#include "DrsState.hpp"

#define DRS_FAULT_NONE          0x00
#define DRS_FAULT_SERVO         0x01  // servo did not reach target in time
#define DRS_FAULT_INPUT_TIMEOUT 0x02  // missing CAN/sensor updates
#define DRS_FAULT_DISABLED      0x04  // controller disabled via flag


struct DrsInputs {
    bool    enable;            // master enable (true = allowed to operate)
    bool    brake_pressed;     // true if brake pressure above threshold
    float   speed_kph;         // vehicle speed (can be float or scaled int)
    bool    external_fault;    // any upstream hard fault that must close DRS
};

struct DrsOutputs {
    bool    servo_open_cmd;    // pulse this true when transitioning to OPENING
    bool    servo_close_cmd;   // pulse this true when transitioning to CLOSING
    uint8_t target_angle_deg;  // commanded target angle (OPEN/CLOSE)
    uint8_t fault_flags;       // DRS_FAULT_* bits
};

struct DrsConfig {
    // Speed thresholds (kph)
    float open_speed_kph   = 40.0f;  // must exceed to allow opening
    float close_speed_kph  = 35.0f;  // below this, begin closing (hysteresis)

    uint16_t servo_settle_ms    = 300;  // time to consider servo “at target”
    uint16_t input_timeout_ms   = 250;  // if inputs stale → fault & close

    // Servo target angles (degrees)
    uint8_t open_angle_deg  = 60;   //temp
    uint8_t close_angle_deg = 0;
};

class DrsController {
public:
    DrsController() {}

    // One-time init; optionally pass config. Safe to call again to reset.
    void init(const DrsConfig& cfg);

    // Update config on the fly (optional).
    void set_config(const DrsConfig& cfg);
    const DrsConfig& get_config() const { return cfg_; }

    // Feed latest inputs before calling tick_10ms().
    void set_inputs(const DrsInputs& in);

    // Inform controller whether the servo reports being at its target.
    // Your servo driver should compute this based on current angle feedback
    // or elapsed time since command.
    void notify_servo_at_target(bool at_target);

    // Main state machine step, called every 10 ms.
    void tick_10ms();

    // Accessors
    DrsState   state() const       { return state_; }
    uint16_t   time_in_state_ms() const { return ticks_in_state_ * 10u; }
    DrsOutputs get_outputs() const { return out_; }

    // Emergency actions
    void force_close();   // immediately begin closing (sets state & outputs)
    void disable();       // equivalent to set_inputs({enable=false,...})
    void clear_faults();  // clears internal fault bits; does not change state

private:
    // --- Helpers (implemented in .cpp) ---
    void enter_state(DrsState s);
    void update_faults(uint16_t dt_ms);
    bool inputs_valid() const;

    // --- State & config ---
    DrsConfig cfg_{};
    DrsState  state_ = DrsState::CLOSED;

    // Input latches (last provided to set_inputs)
    DrsInputs in_{};

    // Output latched per tick
    DrsOutputs out_{};

    uint16_t ticks_in_state_ = 0;      // counts 10 ms ticks
    uint16_t ticks_since_inputs_ = 0;  // for input timeout
    bool     servo_at_target_ = false; // last servo feedback

    // Internal fault register
    uint8_t  faults_ = DRS_FAULT_NONE;
};

#endif // DRS_CONTROLLER_HPP


