#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <Arduino.h>

#define ERROR false
#define NO_ERROR true

typedef uint16_t bp_type; // 0 to 30000 psi*10

typedef int8_t ct_type; // -40 to 120 C

typedef uint16_t flow_type;

typedef uint16_t lp_type; // 0 to 500 mm*10

typedef uint16_t pitot_type;

typedef int16_t sa_type; // -1800 to 1800 degrees*10

typedef uint16_t wsp_type;

#endif // SENSOR_TYPES_H