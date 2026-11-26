
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//Current Measurement Setup
static const char *TAG = "ACS712";

#define ADC_UNIT       ADC_UNIT_1
#define ADC_CH         ADC_CHANNEL_7                           // GPIO35 = ADC1_CH7
#define ADC_ATTEN      ADC_ATTEN_DB_12
#define DIV_RATIO      (22.0f / 32.0f)                        //voltage divider
#define SENSITIVITY    0.066f                                //Sensitivity of 30A ACS712
#define V_PER_A_PIN    (SENSITIVITY * DIV_RATIO)            //Corrected Sensitivity after V.Divider

#define CAL_FACTOR    0.98f
#define MAINS_V       243.0f

// Sampling configuration
#define SAMPLE_INTERVAL_US  500           // 2 kHz
#define SAMPLES_PER_WINDOW  800           // 400 ms

// Load detection thresholds
#define LOAD_ON_THRESHOLD   0.50f
#define LOAD_OFF_THRESHOLD  0.20f

// Timing diagnostics
#define MAX_WINDOW_US       450000

// Minimum current noise threshold
#define CURRENT_ZERO_THRESHOLD 0.15f

// ADC HANDLES
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;

// ADC INITIALIZATION
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&unit_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, ADC_CH, &ch_cfg)
    );

    // ADC calibration
     adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC line-fitting calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s), using rough conversion",
                 esp_err_to_name(ret));
    }

}
//Reading from ADC for mv
static inline int read_mv(void)
{
    int raw = 0;
    int mv = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(adc_handle, ADC_CH, &raw)
    );

    if (cali_handle != NULL) {

        ESP_ERROR_CHECK(
            adc_cali_raw_to_voltage(cali_handle, raw, &mv)
        );

    } else {

        // Rough fallback conversion
        mv = raw * 3100 / 4095;
    }

    return mv;
}
//Struct for Measurement Parameters
typedef struct {

    float irms;
    float mean_mv;

    int min_mv;
    int max_mv;

    uint32_t samples;

    int64_t duration_us;

} meas_t;

//RMS measurement 
static meas_t measure(void)
{
    double sum = 0.0;
    double sumsq = 0.0;

    int mn = 99999;
    int mx = 0;

    int64_t start = esp_timer_get_time();

    for (uint32_t i = 0; i < SAMPLES_PER_WINDOW; i++) {

        int mv = read_mv();

        sum += mv;
        sumsq += (double)mv * mv;

        if (mv < mn)
            mn = mv;

        if (mv > mx)
            mx = mv;

        // Schedule next sample at fixed interval
        int64_t target =
            start + ((int64_t)(i + 1) * SAMPLE_INTERVAL_US);

        int64_t now = esp_timer_get_time();

        if (target > now) {

            esp_rom_delay_us(
                (uint32_t)(target - now)
            );
        }
    }

    int64_t end = esp_timer_get_time();

    // Mean voltage
    double mean = sum / SAMPLES_PER_WINDOW;

    // Variance(DC Offset Removed)
    double variance =
        (sumsq / SAMPLES_PER_WINDOW) - (mean * mean);

    if (variance < 0.0)
        variance = 0.0;

    // RMS AC voltage at ADC pin
    float rms_mv = sqrtf((float)variance);

    //RMS Current from RMS Volatge of ADC
    float irms =
        (rms_mv / 1000.0f / V_PER_A_PIN) * CAL_FACTOR;

    meas_t m = {

        .irms = irms,

        .mean_mv = (float)mean,

        .min_mv = mn,

        .max_mv = mx,

        .samples = SAMPLES_PER_WINDOW,

        .duration_us = end - start
    };

    return m;
}

// =====================================================
// MAIN APPLICATION
// =====================================================

void app_main(void)
{
    adc_init();

    bool load_on = false;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "ACS712 CURRENT MONITOR STARTED");
    ESP_LOGI(TAG, "GPIO35 | ADC1_CH7");
    ESP_LOGI(TAG, "Sampling rate: 2 kHz");
    ESP_LOGI(TAG, "Measurement window: 400 ms");
    ESP_LOGI(TAG, "Load ON threshold: %.2f A",
             LOAD_ON_THRESHOLD);
    ESP_LOGI(TAG, "Load OFF threshold: %.2f A",
             LOAD_OFF_THRESHOLD);
    ESP_LOGI(TAG, "==================================");

    while (1) {

        // Perform measurement
        meas_t m = measure();

        float irms = m.irms;

        // Suppress small zero-current noise
        if (irms < CURRENT_ZERO_THRESHOLD)
            irms = 0.0f;

        //LOAD Detection using Hystersis
        if (!load_on && irms >= LOAD_ON_THRESHOLD) {

            load_on = true;

            ESP_LOGI(TAG, "LOAD DETECTED: ON");
        }

        else if (load_on && irms <= LOAD_OFF_THRESHOLD) {

            load_on = false;

            ESP_LOGI(TAG, "LOAD DETECTED: OFF");
        }

        float apparent_power = irms * MAINS_V;  //Apparant Power Calculation(P=VI) 


        ESP_LOGI(TAG,
                 "Load: %s | Current: %.2f A RMS | Apparent Power: %.0f VA",
                 load_on ? "ON" : "OFF",
                 irms,
                 apparent_power);

        ESP_LOGI(TAG,
                 "Mean: %.0f mV | Min: %d mV | Max: %d mV | Samples: %lu",
                 m.mean_mv,
                 m.min_mv,
                 m.max_mv,
                 (unsigned long)m.samples);

        ESP_LOGI(TAG,
                 "Window duration: %lld us",
                 (long long)m.duration_us);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
