#ifndef STEERING_ANGLE_H
#define STEERING_ANGLE_H

#include <Arduino.h>
#include "DAQ/AnalogSensor.h"

class SteeringAngle : public AnalogSensor<sa_type> {

private:
    // Calibration value for ADC zero point
    static constexpr uint16_t SA_ZERO = 512; // Adjust this value based on calibration

    // Value beyond these will output out of range angles
    static constexpr int16_t MIN_ANGLE = -1800; // -180 degrees * 10
    static constexpr int16_t MAX_ANGLE = 1799;  // +179.9 degrees * 10

public:
    SteeringAngle() {}

    boolean calculate(sa_type* result) override {
        if (result == nullptr) return ERROR;  // Error: result pointer is null

        uint16_t filtered_adc = analog_avg.get_analog_average();

        // Wrap around if 
        int16_t delta = filtered_adc - SA_ZERO;
        if (delta > 511) {
            delta -= 1024;
        } else if (delta < -512) {
            delta += 1024;
        }

        // Moving average of angle in Degrees x 10
        int16_t angle = map(delta, -512, 511, MIN_ANGLE, MAX_ANGLE);
        *result = angle; 
        return NO_ERROR;
    }
};

#endif // STEERING_ANGLE_H