#pragma once

typedef struct {
    float real_power_w;
    float apparent_power_va;
    float power_factor;
} power_data_t;

/* Calculate real power, apparent power and power factor */
power_data_t power_calculate(float vrms, float irms, float measured_power);