#ifndef DIGITAL_SENSOR_H
#define DIGITAL_SENSOR_H

#include <Arduino.h>
#include "MovingAvg.h"
#include "SensorTypes.h"

/*
    Currently, all Digital Sensors are Hall Effect and calculate values
    based on the time between pulses.
    This parent class handles the interrupt and moving average
    See note on get_digital_average!
*/
template<typename SensorType>
class DigitalSensor {
protected:
    uint8_t pin;
    MovingAvg<uint32_t, 4> digital_avg;

    void (*interruptFun)();
    volatile uint32_t lastPulseTime;
    volatile uint32_t delta;
    volatile bool newPulse;
    volatile uint32_t ticks;

    virtual uint32_t getTimeout() const = 0;

public:
    DigitalSensor() : pin(0), interruptFun(nullptr), lastPulseTime(0), delta(0), newPulse(false), ticks(0) {}

    virtual void init(uint8_t pin_num, uint8_t num_readings, void (*intFun)() = nullptr) {
        pin = pin_num;
        pinMode(pin, INPUT);
        digital_avg.reset();

        if (intFun) {
            interruptFun = intFun;
            attachInterrupt(digitalPinToInterrupt(pin), intFun, RISING);
        }
    }

    /*
        Updates the moving average with the latest pulse duration
        Resets the moving average if a timeout occurs
        Interrupts do not need to be closed because the intHandler 
        is set up to update all variables atomically.
    */
    void update() {
        // If pulse takes too long, reset moving avg to avoid keeping old values
        //if (micros() - lastPulseTime > getTimeout()) {  
        //    digital_avg.reset();
        //}
        // Only update moving sum if a pulse has arrived
        if(newPulse) {
            digital_avg.update(delta);
            newPulse = false;
        }
    }

    /*  
        Sensor-sepcific calculation logic 
        Returns success/error
        Make sure to use the correct sensor type defined in SensorTypes.h
    */
    virtual bool calculate(SensorType* result) = 0;

    void intHandler() {
        uint32_t now = micros();
        delta = now-lastPulseTime;
        lastPulseTime = now;
        newPulse = true;
        ticks++;
    }

};

#endif // DIGITAL_SENSOR_H