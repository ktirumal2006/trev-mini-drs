#ifndef MOVING_AVG_H
#define MOVING_AVG_H

#include <Arduino.h>

template<typename T, size_t N>
class MovingAvg {
    T buffer[N] = {0};
    uint32_t sum = 0;
    size_t index = 0;
    bool full = FALSE;

public:
    // General update: add new sample to buffer
    void update(T new_sample) {
        sum -= buffer[index];
        buffer[index] = new_sample;
        sum += new_sample;

        index = (index + 1) % N;
        if (index == 0) {
            full = TRUE; // Buffer is full after first N samples
        }
    }

    /*
        Analog average: sum / N
    */ 
    T get_analog_average() const {

        return (N == 0) ? ((T)0) : ( full ? ((T)(sum / N)) : ((T)(sum/(index + 1))) );
    }

    /*
        Digital average: (scaleConst * N) / sum
        NOTE: This function also converts to rate at the same time since count and sum
        are needed for intermediate calculations anyways.
        Use stoichiometry to determine the correct scale constant for your sensor.
    */
    T get_digital_average(uint32_t scaleConst) const {
        return (sum == 0 || N == 0) ? ((T)0) : (full ? (T)((scaleConst * N) / sum) : (T)((scaleConst * (index + 1)) / sum));
    }

    void reset() {
        for (size_t i = 0; i < N; i++) {
            buffer[i] = 0;
        }
        sum = 0;
        index = 0;
        full = FALSE;
    }
};

#endif // MOVING_AVG_H