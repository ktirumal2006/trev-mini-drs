#ifndef WHEEL_SPEED_SENSOR_H
#define WHEEL_SPEED_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Wheel speed sensor module
 *
 * - Measures time between pulses from a hall-effect or similar sensor.
 * - Uses integer math to compute wheel speed in MPH * 100.
 * - Based on original Arduino implementation:
 *      ROLLING_RADIUS in hundredths of inches
 *      SPOKES = number of teeth / magnets per revolution
 *      CALEBS_CONST ≈ (2π * us_per_hour) / (hundredths_of_inches_per_mile)
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration constants (can be overridden at compile time if needed) */
#define WSS_ROLLING_RADIUS_HUNDREDTHS  (900U)   /* 900 hundredths of inches */
#define WSS_SPOKES                     (16U)
#define WSS_CALEBS_CONST               (3570U)

/* Timeout in microseconds: if no pulse for this long, treat speed as 0 */
#define WSS_TIMEOUT_US                 (1000000UL) /* 1 second */

/* Size of the moving-average window over pulse intervals (deltas) */
#define WSS_MOVING_AVG_WINDOW          (4U)

/* Type for speed result: MPH * 100 (e.g. 1234 => 12.34 mph) */
typedef uint32_t wss_speed_t;

typedef struct {
    /* Config */
    uint32_t rolling_radius_hundredths;
    uint8_t  spokes;

    /* Precomputed constant: CALEBS_CONST * R * 100 / SPOKES */
    uint32_t wspd_const;

    /* Pulse timing state (filled from ISR hook) */
    uint32_t last_pulse_time_us;
    uint32_t last_delta_us;
    bool     new_pulse;
    uint32_t ticks;

    /* Moving average over deltas (time between pulses) */
    uint32_t window[WSS_MOVING_AVG_WINDOW];
    uint8_t  window_size;   /* number of valid entries if not full */
    uint8_t  index;         /* next index to overwrite */
    bool     window_full;
} WheelSpeedSensor;

/**
 * @brief Initialize wheel speed sensor instance.
 *
 * @param s   Pointer to sensor instance.
 * @param rolling_radius_hundredths  Rolling radius in hundredths of inches.
 * @param spokes Number of spokes / magnets / teeth per revolution.
 */
void WSS_Init(WheelSpeedSensor *s,
              uint32_t rolling_radius_hundredths,
              uint8_t spokes);

/**
 * @brief ISR hook called on each sensor pulse edge.
 *
 * Call this from your EXTI / timer capture ISR with a timestamp in microseconds.
 *
 * @param s      Pointer to sensor instance.
 * @param now_us Current time in microseconds (from a free-running timer).
 */
void WSS_OnPulse(WheelSpeedSensor *s, uint32_t now_us);

/**
 * @brief Periodic update; handles moving average and timeout.
 *
 * Call this regularly from the main loop / scheduler (e.g. every 1–10 ms).
 *
 * @param s      Pointer to sensor instance.
 * @param now_us Current time in microseconds.
 */
void WSS_Update(WheelSpeedSensor *s, uint32_t now_us);

/**
 * @brief Compute wheel speed in MPH * 100 using current moving average.
 *
 * @param s          Pointer to sensor instance.
 * @param speed_out  Output pointer for MPH * 100.
 * @return true if a valid speed could be computed, false if no data.
 *
 * When false is returned, *speed_out is left unchanged.
 */
bool WSS_GetSpeedMph100(const WheelSpeedSensor *s, wss_speed_t *speed_out);

/**
 * @brief Convenience function: get speed in floating MPH.
 *
 * @param s Pointer to sensor instance.
 * @return  Speed in MPH as float (0.0f if not enough data).
 */
float WSS_GetSpeedMphFloat(const WheelSpeedSensor *s);

/**
 * @brief Get total pulse count since initialization.
 */
static inline uint32_t WSS_GetTicks(const WheelSpeedSensor *s)
{
    return s->ticks;
}

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_SENSOR_H */
