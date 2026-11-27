
#pragma once

#include <stdint.h>

/* Measurement configuration */
typedef struct {
    uint32_t sample_interval_us;
    uint32_t samples_per_window;
    uint8_t num_harmonics;
} measurement_config_t;

/* Measurement results */
typedef struct {
    float vrms;
    float irms;
    float power;

    float vpin_rms_mv;
    float ibroad_rms_mv;

    float freq_hz;
    float phase_deg;

    float v_mean_mv;
    float i_mean_mv;

    int v_min_mv;
    int v_max_mv;

    int i_min_mv;
    int i_max_mv;

    int64_t duration_us;

} meas_t;

/* Initialize ADC and calibration */
void measurement_init(void);

/* Acquire and process one measurement window */
meas_t measurement_read(void);

/* Get the active measurement configuration */
measurement_config_t measurement_get_config(void);
