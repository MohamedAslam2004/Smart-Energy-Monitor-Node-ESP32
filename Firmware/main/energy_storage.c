#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "energy_storage.h"

static const char *TAG = "ENERGY_NVS";

/* Keep the original NVS identifiers */
#define NVS_NAMESPACE "energy"
#define NVS_KEY_MWH   "e_mwh"

/* Initialize NVS */
esp_err_t energy_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

/* Restore previously stored energy in mWh */
uint64_t energy_storage_load(void)
{
    nvs_handle_t handle;
    uint64_t energy_mwh = 0;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u64(handle, NVS_KEY_MWH, &energy_mwh);
        nvs_close(handle);
    }

    return energy_mwh;
}

/* Save accumulated energy in mWh */
esp_err_t energy_storage_save(uint64_t energy_mwh)
{
    nvs_handle_t handle;

    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not open NVS to save energy");
        return ret;
    }

    ret = nvs_set_u64(handle, NVS_KEY_MWH, energy_mwh);

    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    return ret;
}