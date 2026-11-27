
/* Measurement Module of SMART ENERGY MONITOR*/
/* Sensors
 -ACS712 Current Sensor-GPIO35 ADC_CH_7
 -ZMPT101b Voltage Sensor-GPIO34 ADC_CH_6
 
 *ADC Settings
    -Oneshot Sampling
    -Line Fitting Calibration

 *Measurement Features
      -RMS Calculation using Harmonic DFT
      -Fundamental phase Measurement
      -Mains  Frequency estimation
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "measurement.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "MEASUREMENT";

/* ---------------- Hardware setup ---------------- */

#define ADC_UNIT        ADC_UNIT_1
#define ADC_CH_I        ADC_CHANNEL_7
#define ADC_CH_V        ADC_CHANNEL_6
#define ADC_ATTEN       ADC_ATTEN_DB_12

#define DIV_RATIO       (22.0f / 32.0f)
#define SENSITIVITY     0.066f
#define V_PER_A_PIN     (SENSITIVITY * DIV_RATIO)

/*---------------- Calibration ----------------*/

#define I_CAL           0.95f
#define V_CAL           1018.0f

#define PHASE_CORR_DEG  0.0

/*These Values are acheived by repeated Calibration of Sensors against a Multimeter and 
Clamp meter as a refence measuring device*/

/*---------------- Sampling configuration ----------------*/

#define SAMPLE_INTERVAL_US   250
#define SAMPLES_PER_WINDOW   1600

/*---------------- Frequency analysis ----------------*/

#define MAINS_FREQ_HZ        50.0
#define FREQ_MIN_HZ          45.0
#define FREQ_MAX_HZ          55.0
#define NUM_HARMONICS        7
#define ZC_ARM_MV            80.0

/*---------------- ADC handles ----------------*/

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle = NULL;

/* Measurement buffers */

static int16_t v_buf[SAMPLES_PER_WINDOW];
static int16_t i_buf[SAMPLES_PER_WINDOW];


/*-----ADC INITIALIZATION------------------------*/

void measurement_init(void)
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
        adc_oneshot_config_channel(adc_handle, ADC_CH_I, &ch_cfg)
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, ADC_CH_V, &ch_cfg)
    );

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    esp_err_t ret =
        adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC line-fitting calibration enabled");
    } else {
        ESP_LOGW(TAG,
                 "ADC calibration unavailable (%s), using rough conversion",
                 esp_err_to_name(ret));
    }
}


/*--------------ADC READINGS INTO mv FOR VOLTAGE SPECIFICATION----------*/

static inline int read_mv(adc_channel_t ch)
{
    int raw = 0;
    int mv = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(adc_handle, ch, &raw)
    );

    if (cali_handle != NULL) {
        ESP_ERROR_CHECK(
            adc_cali_raw_to_voltage(cali_handle, raw, &mv)
        );
    } else {
        mv = raw * 3100 / 4095;
    }

    return mv;
}


/*------------------------MAINS FREQUENCY ESTIMATION-------------------------*/

static double estimate_freq(double mean_v)
{
    bool armed = false;

    int cnt = 0;

    double first = 0.0;
    double last = 0.0;

    for (int n = 1; n < SAMPLES_PER_WINDOW; n++) {

        double a = v_buf[n - 1] - mean_v;
        double b = v_buf[n] - mean_v;

        if (a < -ZC_ARM_MV) {
            armed = true;
        }

        if (armed && a < 0.0 && b >= 0.0) {

            double t = (n - 1) + (-a) / (b - a);

            if (cnt == 0) {
                first = t;
            }

            last = t;
            cnt++;

            armed = false;
        }
    }

    if (cnt < 5) {
        return MAINS_FREQ_HZ;
    }

    double period_samples =
        (last - first) / (double)(cnt - 1);

    double f =
        1e6 / (period_samples * SAMPLE_INTERVAL_US);

    if (f < FREQ_MIN_HZ || f > FREQ_MAX_HZ) {
        return MAINS_FREQ_HZ;
    }

    return f;
}


/*-----------------------SINGLE BIN HARMONIC DFT---------------------------*/

static void harmonic_dft(
    const int16_t *x,
    double mean,
    double theta1,
    int k,
    float *re,
    float *im)
{
    const float th = (float)(theta1 * k);

    const float cd = cosf(th);
    const float sd = sinf(th);
    const float m = (float)mean;

    float c = 1.0f;
    float s = 0.0f;

    float sr = 0.0f;
    float si = 0.0f;

    for (int n = 0; n < SAMPLES_PER_WINDOW; n++) {

        float xv = (float)x[n] - m;

        sr += xv * c;
        si += xv * s;

        float cn = c * cd - s * sd;

        s = s * cd + c * sd;
        c = cn;
    }

    *re = sr;
    *im = si;
}


/* -----------------COMPLETE MEASUREMENT --------------------------------*/

meas_t measurement_read(void)
{
    int64_t sum_v = 0;
    int64_t sum_i = 0;
    int64_t sum_ii = 0;

    int v_min = 99999;
    int v_max = 0;

    int i_min = 99999;
    int i_max = 0;

    int64_t start = esp_timer_get_time();

    /* -------- Synchronized voltage/current sampling -------- */

    for (uint32_t n = 0; n < SAMPLES_PER_WINDOW; n++) {

        int v = read_mv(ADC_CH_V);
        int c = read_mv(ADC_CH_I);

        v_buf[n] = (int16_t)v;
        i_buf[n] = (int16_t)c;

        sum_v += v;
        sum_i += c;
        sum_ii += (int64_t)c * c;

        if (v < v_min) v_min = v;
        if (v > v_max) v_max = v;

        if (c < i_min) i_min = c;
        if (c > i_max) i_max = c;

        int64_t target =
            start + ((int64_t)(n + 1) * SAMPLE_INTERVAL_US);

        int64_t now = esp_timer_get_time();

        if (target > now) {
            esp_rom_delay_us((uint32_t)(target - now));
        }
    }

    int64_t end = esp_timer_get_time();

    /* -------- Mean and broadband current RMS -------- */

    const double N = (double)SAMPLES_PER_WINDOW;

    double mean_v = (double)sum_v / N;
    double mean_i = (double)sum_i / N;

    double var_i =
        (double)sum_ii / N - mean_i * mean_i;

    if (var_i < 0.0) {
        var_i = 0.0;
    }

    /* -------- Frequency and fundamental angle -------- */

    double f_hz = estimate_freq(mean_v);

    double theta1 =
        2.0 * M_PI * f_hz * SAMPLE_INTERVAL_US * 1e-6;

    /* -------- Harmonic analysis -------- */

    double v2 = 0.0;
    double i2 = 0.0;
    double p2 = 0.0;

    double phase_deg = 0.0;

    for (int k = 1; k <= NUM_HARMONICS; k++) {

        float vre, vim;
        float ire, iim;

        harmonic_dft(v_buf, mean_v, theta1, k, &vre, &vim);
        harmonic_dft(i_buf, mean_i, theta1, k, &ire, &iim);

        if (k == 1) {

            double re =
                (double)vre * ire + (double)vim * iim;

            double im =
                (double)vre * iim - (double)vim * ire;

            phase_deg =
                atan2(im, re) * 180.0 / M_PI;

            if (phase_deg > 90.0) {
                phase_deg -= 180.0;
            }

            if (phase_deg < -90.0) {
                phase_deg += 180.0;
            }
        }

        /* Current phasor phase correction */

        double psi =
            k * PHASE_CORR_DEG * M_PI / 180.0;

        double cp = cos(psi);
        double sp = sin(psi);

        double ire2 = ire * cp + iim * sp;
        double iim2 = iim * cp - ire * sp;

        v2 +=
            (double)vre * vre + (double)vim * vim;

        i2 +=
            (double)ire * ire + (double)iim * iim;

        p2 +=
            (double)vre * ire2 + (double)vim * iim2;
    }

    /* -------- RMS and power scaling -------- */

    const double norm = 2.0 / (N * N);

    double vpin_rms_mv = sqrt(v2 * norm);
    double ipin_rms_mv = sqrt(i2 * norm);
    double p_mv2 = p2 * norm;

    const double v_scale = V_CAL / 1000.0;

    const double i_scale =
        I_CAL / 1000.0 / V_PER_A_PIN;

    /* -------- Return measurement structure -------- */

    meas_t m = {

        .vrms = (float)(vpin_rms_mv * v_scale),

        .irms = (float)(ipin_rms_mv * i_scale),

        .power = (float)(p_mv2 * v_scale * i_scale),

        .vpin_rms_mv = (float)vpin_rms_mv,

        .ibroad_rms_mv = (float)sqrt(var_i),

        .freq_hz = (float)f_hz,

        .phase_deg = (float)phase_deg,

        .v_mean_mv = (float)mean_v,

        .i_mean_mv = (float)mean_i,

        .v_min_mv = v_min,
        .v_max_mv = v_max,

        .i_min_mv = i_min,
        .i_max_mv = i_max,

        .duration_us = end - start
    };

    return m;


    
}
measurement_config_t measurement_get_config(void)
{
    measurement_config_t config = {
        .sample_interval_us = SAMPLE_INTERVAL_US,
        .samples_per_window = SAMPLES_PER_WINDOW,
        .num_harmonics = NUM_HARMONICS
    };

    return config;
}
