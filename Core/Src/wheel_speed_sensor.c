#include "wheel_speed_sensor.h"

/* Internal helper: reset moving-average window */
static void WSS_ResetWindow(WheelSpeedSensor *s)
{
    for (uint8_t i = 0; i < WSS_MOVING_AVG_WINDOW; ++i) {
        s->window[i] = 0U;
    }
    s->window_size = 0U;
    s->index       = 0U;
    s->window_full = false;
}

/* Internal helper: push a new delta into the moving-average window */
static void WSS_AddDelta(WheelSpeedSensor *s, uint32_t delta_us)
{
    s->window[s->index] = delta_us;

    if (!s->window_full) {
        if (s->window_size < WSS_MOVING_AVG_WINDOW) {
            s->window_size++;
        }
        if (s->window_size == WSS_MOVING_AVG_WINDOW) {
            s->window_full = true;
        }
    }

    s->index++;
    if (s->index >= WSS_MOVING_AVG_WINDOW) {
        s->index = 0U;
    }
}

/* Internal helper: compute average delta in microseconds */
static bool WSS_GetAverageDeltaUs(const WheelSpeedSensor *s, uint32_t *avg_delta_us)
{
    uint8_t count = s->window_full ? WSS_MOVING_AVG_WINDOW : s->window_size;
    if (count == 0U) {
        return false; /* no data yet */
    }

    uint64_t sum = 0U;
    for (uint8_t i = 0; i < count; ++i) {
        sum += s->window[i];
    }

    *avg_delta_us = (uint32_t)(sum / count);
    return true;
}

void WSS_Init(WheelSpeedSensor *s,
              uint32_t rolling_radius_hundredths,
              uint8_t spokes)
{
    if (s == NULL) {
        return;
    }

    s->rolling_radius_hundredths = (rolling_radius_hundredths == 0U)
                                   ? WSS_ROLLING_RADIUS_HUNDREDTHS
                                   : rolling_radius_hundredths;

    s->spokes = (spokes == 0U) ? WSS_SPOKES : spokes;

    /*
     * WSPD_CONST = CALEBS_CONST * ROLLING_RADIUS * 100 / SPOKES
     * where:
     *   CALEBS_CONST ≈ (2π * us_per_hour) / (hundredths_of_inches_per_mile)
     * Result:
     *   speed_mph_100 = WSPD_CONST / avg_delta_us
     */
    uint64_t tmp = (uint64_t)WSS_CALEBS_CONST *
                   (uint64_t)s->rolling_radius_hundredths * 100ULL;

    if (s->spokes > 0U) {
        tmp /= (uint64_t)s->spokes;
    }

    s->wspd_const = (uint32_t)tmp;

    s->last_pulse_time_us = 0U;
    s->last_delta_us      = 0U;
    s->new_pulse          = false;
    s->ticks              = 0U;

    WSS_ResetWindow(s);
}

void WSS_OnPulse(WheelSpeedSensor *s, uint32_t now_us)
{
    if (s == NULL) {
        return;
    }

    if (s->last_pulse_time_us == 0U) {
        /* First pulse: just latch timestamp, no delta yet */
        s->last_pulse_time_us = now_us;
        return;
    }

    uint32_t delta = now_us - s->last_pulse_time_us;
    s->last_pulse_time_us = now_us;

    /* Store delta for processing in the non-ISR context */
    s->last_delta_us = delta;
    s->new_pulse     = true;
    s->ticks++;
}

void WSS_Update(WheelSpeedSensor *s, uint32_t now_us)
{
    if (s == NULL) {
        return;
    }

    /* Timeout check: if too long since last pulse, reset window */
    if (s->last_pulse_time_us != 0U) {
        uint32_t dt = now_us - s->last_pulse_time_us;
        if (dt > WSS_TIMEOUT_US) {
            WSS_ResetWindow(s);
            s->last_pulse_time_us = 0U;
        }
    }

    /* If a new pulse arrived since the last update, fold it into the average */
    if (s->new_pulse) {
        WSS_AddDelta(s, s->last_delta_us);
        s->new_pulse = false;
    }
}

bool WSS_GetSpeedMph100(const WheelSpeedSensor *s, wss_speed_t *speed_out)
{
    if (s == NULL || speed_out == NULL) {
        return false;
    }

    uint32_t avg_delta_us;
    if (!WSS_GetAverageDeltaUs(s, &avg_delta_us)) {
        return false; /* no data */
    }

    if (avg_delta_us == 0U) {
        return false; /* avoid division by zero */
    }

    /*
     * speed_mph_100 = WSPD_CONST / avg_delta_us
     * Optionally add half denominator for integer rounding.
     */
    uint64_t num = (uint64_t)s->wspd_const;
    uint32_t result = (uint32_t)(num / avg_delta_us);

    *speed_out = result;
    return true;
}

float WSS_GetSpeedMphFloat(const WheelSpeedSensor *s)
{
    wss_speed_t mph100;
    if (!WSS_GetSpeedMph100(s, &mph100)) {
        return 0.0f;
    }
    return (float)mph100 / 100.0f;
}
