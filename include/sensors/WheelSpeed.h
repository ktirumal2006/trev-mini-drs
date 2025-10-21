#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <Arduino.h>
#include "DAQ/DigitalSensor.h"

class WheelSpeed : public DigitalSensor<wsp_type> {

private:
    static constexpr uint16_t ROLLING_RADIUS = 900;  // Hundredths of inches
    static constexpr uint8_t SPOKES = 16;
    /*
        Some realistic times between pulses:
        1mph ~200,000us
        40mph ~5,000us
        100mph ~2,000us
    */

    /*
        This beautiful constant was derived by Caleb Lee and yields extremely accurate
        results while maintaning purely integer math without overflowing.
        derived from (2*pi)(us per hour) / (hundredths of inches per mile) = 3569.992
    */
    static constexpr uint32_t CALEBS_CONST = 3570;

    // 100x multiplier for mph, do the math yourself for proof
    static constexpr uint32_t WSPD_CONST = CALEBS_CONST * ROLLING_RADIUS * 100 / SPOKES;

    static constexpr uint32_t TIMEOUT = 1000000;

protected:
    uint32_t getTimeout() const override {
        return TIMEOUT;
    }

public:
    WheelSpeed() {}
    
    uint32_t getTicks() {
        return ticks;
    }

    boolean calculate(wsp_type* result) override {
        if (result == nullptr) return ERROR;  // Error: result pointer is null

        // Moving average of Speed in MPH x 100
        wsp_type mph = digital_avg.get_digital_average(WSPD_CONST);
        *result = mph;
        return NO_ERROR;
    }
};

#endif