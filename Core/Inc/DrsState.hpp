#ifndef DRS_STATE_HPP
#define DRS_STATE_HPP

#include <stdint.h>


enum class DrsState : uint8_t {
    CLOSED = 0,    // Flap fully closed, resting position
    OPENING,       // Servo is moving toward open position
    OPEN,          // Flap fully open, DRS active
    CLOSING        // Servo is moving back to closed position
};

inline DrsState drs_state_from_u8(uint8_t val)
{
    switch (val) {
        case 0: return DrsState::CLOSED;
        case 1: return DrsState::OPENING;
        case 2: return DrsState::OPEN;
        case 3: return DrsState::CLOSING;
        default: return DrsState::CLOSED;
    }
}

#endif 
