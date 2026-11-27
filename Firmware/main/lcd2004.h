#pragma once

#include <stdbool.h>

typedef struct {
    float vrms;
    float irms;
    float power;        /* W  */
    float apparent;     /* VA */
    float pf;
    float energy_kwh;
    bool  load_on;
} lcd_data_t;

/* Init I2C + the 20x4 LCD and start the display task (pinned to core 1).
 * If the LCD is not found it logs an error and the rest of the firmware keeps running. */
void lcd_start(void);

/* Send the latest readings to the display (non-blocking, keeps only the newest). */
void lcd_post(const lcd_data_t *d);