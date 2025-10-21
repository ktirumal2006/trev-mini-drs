#ifndef BRAKE_PRESSURE_H
#define BRAKE_PRESSURE_H

#include <Arduino.h>
#include "DAQ/AnalogSensor.h"

class BrakePressure : public AnalogSensor<bp_type> {

private:
    static constexpr uint16_t MIN_MV = 500;
    static constexpr uint16_t MAX_MV = 4500;
    static constexpr uint16_t MIN_PSI = 0;
    static constexpr uint16_t MAX_PSI = 3000;

public:
    BrakePressure() {};
    
    boolean calculate(bp_type* result) override {
        if (result == nullptr) return ERROR;  // Error: result pointer is null

        uint16_t filtered_adc = analogRead(pin);//analog_avg.get_analog_average();
        uint16_t mV = map(filtered_adc, 0, 1023, 0, 5000);
        if (mV < MIN_MV || mV > MAX_MV) {
            *result = 0;
            return ERROR; // Error: out of range
        }

        // Moving Average of Pressure in PSI
        bp_type psi = map(mV, MIN_MV, MAX_MV, MIN_PSI, MAX_PSI);
        *result = psi;
        return NO_ERROR;
    }
};

#endif // BRAKE_PRESSURE_H