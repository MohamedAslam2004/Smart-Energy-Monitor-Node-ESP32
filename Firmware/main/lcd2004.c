#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "lcd2004.h"

static const char *TAG = "LCD";

/* ---------------- Settings ---------------- */
#define LCD_SDA_GPIO    21
#define LCD_SCL_GPIO    22
#define LCD_I2C_HZ      100000
#define LCD_COLS        20
#define LCD_ROWS        4

#define LCD_RS          0x01
#define LCD_EN          0x04
#define LCD_BL          0x08

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;
static QueueHandle_t disp_q = NULL;

static esp_err_t pcf_write(const uint8_t *buf, size_t len)
{
    return i2c_master_transmit(dev, buf, len, 200);
}

static void lcd_nibble(uint8_t nib)
{
    uint8_t d = (nib & 0xF0) | LCD_BL;
    uint8_t b[2] = { (uint8_t)(d | LCD_EN), d };
    pcf_write(b, 2);
}

static size_t encode_byte(uint8_t *out, uint8_t val, uint8_t rs)
{
    uint8_t hi = (val & 0xF0) | LCD_BL | rs;
    uint8_t lo = ((val << 4) & 0xF0) | LCD_BL | rs;
    out[0] = hi | LCD_EN;
    out[1] = hi;
    out[2] = lo | LCD_EN;
    out[3] = lo;
    return 4;
}

static void lcd_cmd(uint8_t cmd, uint32_t delay_us)
{
    uint8_t b[4];
    encode_byte(b, cmd, 0);
    pcf_write(b, 4);
    esp_rom_delay_us(delay_us);
}

static void lcd_write_row(int row, const char *text)
{
    static const uint8_t row_addr[LCD_ROWS] = { 0x00, 0x40, 0x14, 0x54 };
    uint8_t buf[4 * (1 + LCD_COLS)];
    size_t n = 0;

    n += encode_byte(buf + n, 0x80 | row_addr[row], 0);
    for (int i = 0; i < LCD_COLS; i++) 
    {
        n += encode_byte(buf + n, (uint8_t)text[i], LCD_RS);
    }
    pcf_write(buf, n);
}

static void show(int row, const char *s)
{
    char line[LCD_COLS + 1];
    snprintf(line, sizeof line, "%-20.20s", s);
    lcd_write_row(row, line);
}

static void lcd_hw_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Wake-up sequence*/
    lcd_nibble(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_nibble(0x30);
    esp_rom_delay_us(200);
    lcd_nibble(0x30);
    esp_rom_delay_us(200);
    lcd_nibble(0x20);
    esp_rom_delay_us(200);

    lcd_cmd(0x28, 50);     
    lcd_cmd(0x0C, 50);      
    lcd_cmd(0x01, 2000);    
    lcd_cmd(0x06, 50);      
}

static bool lcd_attach(void)
{
    const uint8_t addrs[] = { 0x27, 0x3F };

    for (size_t k = 0; k < sizeof(addrs)/sizeof(addrs[0]); k++) 
    {
        if (i2c_master_probe(bus, addrs[k], 100) == ESP_OK) {
            i2c_device_config_t dev_cfg = 
            {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = addrs[k],
                .scl_speed_hz    = LCD_I2C_HZ,
            };
            if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) 
            {
                ESP_LOGI(TAG, "LCD found at 0x%02X", addrs[k]);
                return true;
            }
        }
    }
    return false;
}

/* ---------------- Display task ---------------- */

static void render(const lcd_data_t *d)
{
    char line[LCD_COLS + 1];

    show(0, "SMART ENERGY MONITOR");

    snprintf(line, sizeof(line),
             "V:%6.1fV I:%5.2fA",
             d->vrms, d->irms);
    show(1, line);

    snprintf(line, sizeof(line),
             "P:%5.0fW PF:%4.2f",
             d->power, d->pf);
    show(2, line);

    snprintf(line, sizeof(line),
             "E:%8.4f kWh L:%s",
             d->energy_kwh,
             d->load_on ? "ON" : "OFF");
    show(3, line);
}

static void display_task(void *arg)
{
    lcd_data_t d;

    while (1) 
    {
        if (xQueueReceive(disp_q, &d, portMAX_DELAY) == pdTRUE) 
        {
            render(&d);
        }
    }
}

/* ---------------- Public API ---------------- */
void lcd_start(void)
{
    i2c_master_bus_config_t bus_cfg = 
    {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = LCD_SDA_GPIO,
        .scl_io_num        = LCD_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) 
    {
        ESP_LOGE(TAG, "I2C bus init failed");
        return;
    }

    if (!lcd_attach()) 
    {
        ESP_LOGE(TAG, "LCD not found at 0x27 or 0x3F. Check SDA (GPIO%d), SCL (GPIO%d), power and GND.",
                 LCD_SDA_GPIO, LCD_SCL_GPIO);
        return;
    }

    lcd_hw_init();

    show(0, "SMART ENERGY MONITOR");
    show(1, "MONITOR STARTED");
    show(2, "POWER + ENERGY");
    show(3, "Initializing...");
vTaskDelay(pdMS_TO_TICKS(2000));
    disp_q = xQueueCreate(1, sizeof(lcd_data_t));
    if (disp_q == NULL) {
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    xTaskCreatePinnedToCore(display_task, "lcd", 4096, NULL, 2, NULL, 1);
}

void lcd_post(const lcd_data_t *d)
{
    if (disp_q != NULL) {
        xQueueOverwrite(disp_q, d);
    }
}