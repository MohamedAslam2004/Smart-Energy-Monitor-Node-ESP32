#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t energy_storage_init(void);
uint64_t energy_storage_load(void);
esp_err_t energy_storage_save(uint64_t energy_mwh);