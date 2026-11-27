#include <math.h>
#include "power_cal.h"

power_data_t power_calculate(float vrms, float irms, float measured_power)
{
    power_data_t result = {0};

    /* Consumption only */
    result.real_power_w = fabsf(measured_power);

    /* Apparent power */
    result.apparent_power_va = vrms * irms;

    /* Power factor */
    if (result.apparent_power_va > 0.0f) {
        result.power_factor =
            result.real_power_w / result.apparent_power_va;
    } else {
        result.power_factor = 0.0f;
    }

    /* Limit PF to a maximum of 1 */
    if (result.power_factor > 1.0f) {
        result.power_factor = 1.0f;
    }

    return result;
}