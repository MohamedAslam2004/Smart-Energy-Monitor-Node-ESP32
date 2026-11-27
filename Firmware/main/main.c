/*---- SMART ENERGY MONITOR ----
 * Sensors : ACS712 (current), ZMPT101B (voltage)
 * Sensor outputs are scaled by a voltage divider (10k top / 22k bottom)
 * with a 0.1 uF capacitor for filtering before the ADC.
 * Outputs : Vrms, Irms, real power (W), apparent power (VA), PF, energy (kWh)
 *
 * Measurement: instead of broadband RMS, the window is analysed only at the
 * mains frequency and its harmonics (single-bin DFT at the measured mains
 * frequency). Random ADC and supply noise outside those bins is rejected,
 * so the zero-load reading stays at zero.
 */

/* Mohamed Aslam H
 * Git: https://github.com/MohamedAslam2004
 * LinkedIn: https://www.linkedin.com/in/mdaslamh/
 */

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "lcd2004.h"
#include "measurement.h"
#include "power_cal.h"
#include "energy_storage.h"

static const char *TAG = "ENERGY";

#define MAX_WINDOW_US 450000 //MAX WINDOW FOR READING 

/* Phase correction applied to the current phasor */
#define PHASE_CAL_MODE        1                /* 1 = print the measured V-I phase offset */
#define PHASE_CAL_MIN_CURRENT 2.0f             /*Resistive Load Setting*/

/* ---------------- Thresholds ---------------- */
#define LOAD_ON_THRESHOLD       0.30f          /* A */
#define LOAD_OFF_THRESHOLD      0.15f          /* A */
#define CURRENT_ZERO_THRESHOLD  0.06f          /* A, below this = noise */
#define VOLTAGE_MIN_VALID       50.0f          /* V, below this = no mains */

/* ---------------- NVS (energy storage) ---------------- */
#define SAVE_INTERVAL_S      300               /* save kWh every 5 minutes */
#define RESET_ENERGY_ON_BOOT 0                 /* set to 1 once to zero the kWh */
#define DEBUG_PIN_DATA       1                 /* 1 = print raw pin data line */

/* ---------------- Main ---------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(energy_storage_init());
    measurement_init();
    measurement_config_t config = measurement_get_config();
    lcd_start();

#if RESET_ENERGY_ON_BOOT
    ESP_ERROR_CHECK(energy_storage_save(0));
    ESP_LOGW(TAG, "Energy counter reset to 0");
#endif
    uint64_t saved_mwh = energy_storage_load();
    double energy_wh   = (double)saved_mwh / 1000.0;
    
    bool    load_on      = false;
    int64_t last_us      = esp_timer_get_time();
    int64_t last_save_us = last_us;

    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "ENERGY MONITOR STARTED");
    ESP_LOGI(TAG, "Current: GPIO35 (ADC1_CH7) | Voltage: GPIO34 (ADC1_CH6)");
    ESP_LOGI(TAG, "Sampling rate: %lu Hz",(unsigned long)(1000000 / config.sample_interval_us));
    ESP_LOGI(TAG, "Measurement window: %lu ms",(unsigned long)(config.samples_per_window *config.sample_interval_us / 1000));
    ESP_LOGI(TAG, "Harmonics analysed: %u",(unsigned int)config.num_harmonics);
    ESP_LOGI(TAG, "Restored energy: %.4f kWh", energy_wh / 1000.0);
    ESP_LOGI(TAG, "==================================");

    while (1) {

        meas_t   m = measurement_read();
        int64_t now_us = esp_timer_get_time();
        double  dt_s   = (double)(now_us - last_us) / 1e6;
        last_us = now_us;

        float vrms  = m.vrms;
        float irms  = m.irms;
        float power = m.power;   /* consumption only */

        /* No mains: show 0 instead of the noise floor */
        if (vrms < VOLTAGE_MIN_VALID) {
            vrms  = 0.0f;
            power = 0.0f;
        }

        /* Remove zero-current noise */
        if (irms < CURRENT_ZERO_THRESHOLD) {
            irms  = 0.0f;
            power = 0.0f;
        }

        power_data_t p = power_calculate(vrms, irms, power);
        power = p.real_power_w;
        float apparent = p.apparent_power_va;
        float pf = p.power_factor;

        energy_wh += (double)power * dt_s / 3600.0;

        /* Load detection with hysteresis */
        bool load_changed = false;
        if (!load_on && irms >= LOAD_ON_THRESHOLD) {
            load_on = true;
            load_changed = true;
            ESP_LOGI(TAG, "LOAD DETECTED: ON");
        } else if (load_on && irms <= LOAD_OFF_THRESHOLD) {
            load_on = false;
            load_changed = true;
            ESP_LOGI(TAG, "LOAD DETECTED: OFF");
        }

        ESP_LOGI(TAG,
                 "Load: %s | V: %.1f V | I: %.2f A | P: %.0f W | S: %.0f VA | PF: %.2f | E: %.4f kWh",
                 load_on ? "ON" : "OFF", vrms, irms, power, apparent, pf,
                 energy_wh / 1000.0);

        lcd_data_t ld = {
            .vrms = vrms, .irms = irms, .power = power, .apparent = apparent,
            .pf = pf, .energy_kwh = (float)(energy_wh / 1000.0), .load_on = load_on
        };
        lcd_post(&ld);

#if DEBUG_PIN_DATA
        ESP_LOGI(TAG,
                 "V pin: mean %.0f min %d max %d RMS %.1f mV | "
                 "I pin: mean %.0f min %d max %d broadband %.1f mV | "
                 "f %.2f Hz | window %lld us",
                 m.v_mean_mv, m.v_min_mv, m.v_max_mv, m.vpin_rms_mv,
                 m.i_mean_mv, m.i_min_mv, m.i_max_mv, m.ibroad_rms_mv,
                 m.freq_hz, (long long)m.duration_us);
#endif

#if PHASE_CAL_MODE
        if (irms > PHASE_CAL_MIN_CURRENT) {
            ESP_LOGI(TAG, "PHASE CAL: V-I phase offset = %.1f deg (set PHASE_CORR_DEG to this value)",
                     m.phase_deg);
        }
#endif

        /* Save energy every SAVE_INTERVAL_S, and when the load switches off */
        bool timed_save = (now_us - last_save_us) >= ((int64_t)SAVE_INTERVAL_S * 1000000);
        bool off_save   = load_changed && !load_on;
        if (timed_save || off_save) {
            uint64_t mwh = (uint64_t)(energy_wh * 1000.0);
            if (mwh != saved_mwh) {
                esp_err_t save_ret = energy_storage_save(mwh);
                if (save_ret == ESP_OK) 
                {
                     saved_mwh = mwh;
                     ESP_LOGI(TAG, "Energy saved: %.4f kWh",energy_wh / 1000.0);
                }
                else {
                        ESP_LOGE(TAG, "Energy save failed: %s",
                        esp_err_to_name(save_ret));
                      }
            }
            last_save_us = now_us;
        }

        /* Diagnostics */
        if (m.duration_us > MAX_WINDOW_US) {
            ESP_LOGW(TAG, "Measurement window exceeded 450 ms!");
        }
        if (m.i_mean_mv < 1400 || m.i_mean_mv > 2000) {
            ESP_LOGW(TAG, "Current channel mean out of range. Check ACS712 5V and divider.");
        }
        if (m.v_mean_mv < 1400 || m.v_mean_mv > 2000) {
            ESP_LOGW(TAG, "Voltage channel mean out of range. Check ZMPT101B 5V, divider and GND.");
        }
        if (m.vrms < VOLTAGE_MIN_VALID) {
            ESP_LOGW(TAG, "No mains voltage signal. Check ZMPT101B wiring.");
        }
        if (m.v_min_mv < 150 || m.v_max_mv > 2900) {
            ESP_LOGW(TAG, "Voltage signal near ADC limits. Turn the trimpot down.");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}