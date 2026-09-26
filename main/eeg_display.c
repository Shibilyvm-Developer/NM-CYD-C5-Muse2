/*
 * NM-CYD-C5-Muse2
 *
 * Copyright (c) 2026 Shibily VM. All rights reserved.
 *
 * PUBLIC-v1.0
 * Version 1.0.0
 *
 * Project-specific source code is protected by LICENSE.
 *
 * Third-party libraries, hardware, firmware, trademarks, fonts,
 * graphics, and other external material remain the property of
 * their respective owners.
 */
#include "eeg_display.h"
#include "mind_control.h"
#include "ppg_hr.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#define TAG "EEG_LCD"

#define LCD_HOST         SPI2_HOST
#define LCD_SCLK         6
#define LCD_MISO         2
#define LCD_MOSI         7
#define LCD_CS           23
#define LCD_DC           24
#define LCD_RST          -1
#define LCD_BK_LIGHT     25

#define LCD_H_RES        320
#define LCD_V_RES        240
#define LCD_PIXEL_CLK_HZ (20 * 1000 * 1000)

/* ============================================================
 * XPT2046 TOUCHSCREEN
 * ============================================================ */

#define TOUCH_CS            1
#define TOUCH_SPI_CLK_HZ    (2 * 1000 * 1000)

#define TOUCH_RAW_X_TOP     315
#define TOUCH_RAW_X_BOTTOM  3775
#define TOUCH_RAW_Y_LEFT    255
#define TOUCH_RAW_Y_RIGHT   3790

#define TOUCH_POLL_MS       20

static spi_device_handle_t touch_spi = NULL;
static TaskHandle_t touch_task_handle = NULL;

static esp_err_t xpt2046_read_raw(uint8_t command, uint16_t *value)
{
    if (touch_spi == NULL || value == NULL)
        return ESP_ERR_INVALID_STATE;

    uint8_t tx[3] = {
        command,
        0x00,
        0x00
    };

    uint8_t rx[3] = {0};

    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    esp_err_t err = spi_device_transmit(touch_spi, &t);

    if (err != ESP_OK)
        return err;

    uint16_t raw =
        ((uint16_t)rx[1] << 8) |
        rx[2];

    raw >>= 3;
    raw &= 0x0FFF;

    *value = raw;

    return ESP_OK;
}

static bool xpt2046_read(uint16_t *raw_x, uint16_t *raw_y)
{
    if (!raw_x || !raw_y)
        return false;

    uint16_t x = 0;
    uint16_t y = 0;

    if (xpt2046_read_raw(0xD0, &x) != ESP_OK)
        return false;

    if (xpt2046_read_raw(0x90, &y) != ESP_OK)
        return false;

    if (x < 100 || x > 4000 ||
        y < 100 || y > 4000) {
        return false;
    }

    *raw_x = x;
    *raw_y = y;

    return true;
}

static uint16_t touch_map_x(uint16_t raw_y)
{
    if (raw_y <= TOUCH_RAW_Y_LEFT)
        return 0;

    if (raw_y >= TOUCH_RAW_Y_RIGHT)
        return LCD_H_RES - 1;

    int32_t x =
        ((int32_t)(raw_y - TOUCH_RAW_Y_LEFT) *
         (LCD_H_RES - 1)) /
        (TOUCH_RAW_Y_RIGHT - TOUCH_RAW_Y_LEFT);

    if (x < 0)
        x = 0;

    if (x >= LCD_H_RES)
        x = LCD_H_RES - 1;

    return (uint16_t)x;
}

static uint16_t touch_map_y(uint16_t raw_x)
{
    if (raw_x <= TOUCH_RAW_X_TOP)
        return 0;

    if (raw_x >= TOUCH_RAW_X_BOTTOM)
        return LCD_V_RES - 1;

    int32_t y =
        ((int32_t)(raw_x - TOUCH_RAW_X_TOP) *
         (LCD_V_RES - 1)) /
        (TOUCH_RAW_X_BOTTOM - TOUCH_RAW_X_TOP);

    if (y < 0)
        y = 0;

    if (y >= LCD_V_RES)
        y = LCD_V_RES - 1;

    return (uint16_t)y;
}

static void touch_init(void)
{
    if (touch_spi != NULL)
        return;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = TOUCH_SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = TOUCH_CS,
        .queue_size = 1
    };

    ESP_ERROR_CHECK(
        spi_bus_add_device(
            LCD_HOST,
            &devcfg,
            &touch_spi
        )
    );

    ESP_LOGI(
        TAG,
        "XPT2046 initialized: CS=%d SPI2 2MHz",
        TOUCH_CS
    );
}

static void touch_task(void *arg)
{
    (void)arg;

    bool was_pressed = false;
    uint8_t stable_count = 0;

    while (true) {

        uint16_t raw_x = 0;
        uint16_t raw_y = 0;

        bool pressed = xpt2046_read(&raw_x, &raw_y);

        if (pressed) {

            uint16_t x = touch_map_x(raw_y);
            uint16_t y = touch_map_y(raw_x);

            if (!was_pressed) {

                if (stable_count < 3)
                    stable_count++;

                if (stable_count >= 2) {

                    was_pressed = true;
                    stable_count = 0;

                    ESP_LOGI(
                        "EEG_TOUCH",
                        "DOWN x=%u y=%u rawX=%u rawY=%u",
                        x,
                        y,
                        raw_x,
                        raw_y
                    );

                    eeg_display_touch(x, y, true);
                }

            } else {

                eeg_display_touch(x, y, true);
            }

        } else {

            stable_count = 0;

            if (was_pressed) {

                was_pressed = false;

                ESP_LOGI(
                    "EEG_TOUCH",
                    "UP"
                );

                eeg_display_touch(0, 0, false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
    }
}

static void touch_start_task(void)
{
    if (touch_task_handle != NULL)
        return;

    BaseType_t result = xTaskCreate(
        touch_task,
        "xpt2046_touch",
        4096,
        NULL,
        4,
        &touch_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create XPT2046 touch task");
        touch_task_handle = NULL;
        return;
    }

    ESP_LOGI(TAG, "XPT2046 touch polling started");
}


#define DISPLAY_TASK_STACK     8192
#define DISPLAY_TASK_PRIORITY  3
#define DISPLAY_PERIOD_MS      100

typedef struct {
    float delta;
    float theta;
    float alpha;
    float beta;
    float gamma;
    float peak;
    bool valid;
} eeg_ui_result_t;

static esp_lcd_panel_handle_t panel = NULL;
static SemaphoreHandle_t ui_mutex = NULL;
static eeg_ui_result_t ui_data[EEG_CH_COUNT];
static uint16_t framebuffer[LCD_H_RES * LCD_V_RES];

/* ============================================================
 * EEG TOUCH UI PAGE STATE
 * ============================================================ */

#define EEG_UI_PAGE_COUNT 4

/* ============================================================
 * PAGE 2 - HEART / PPG
 * ============================================================ */

static eeg_heart_metrics_t heart_metrics = {
    .bpm = 0.0f,
    .ibi_ms = 0.0f,
    .signal_quality = 0.0f,
    .ppg1 = 0.0f,
    .ppg2 = 0.0f,
    .ppg3 = 0.0f,
    .beat = false,
    .valid = false
};

static uint32_t heart_beat_flash_until = 0;



/* ============================================================
 * LIVE EEG WAVEFORM BUFFER
 * ============================================================ */

#define WAVEFORM_SAMPLES 256

static float waveform_data[EEG_CH_COUNT][WAVEFORM_SAMPLES];
static uint16_t waveform_write_index[EEG_CH_COUNT];
static uint16_t waveform_filled[EEG_CH_COUNT];


static void render_calibration_page(void);



static volatile int eeg_current_page = 0;
static bool calibration_touch_handled = false;

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) |
                      ((g & 0xFC) << 3) |
                      ((b & 0xF8) >> 3));
}

static void fb_clear(uint16_t color)
{
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++)
        framebuffer[i] = color;
}

static void fb_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_H_RES) w = LCD_H_RES - x;
    if (y + h > LCD_V_RES) h = LCD_V_RES - y;
    if (w <= 0 || h <= 0) return;

    for (int yy = y; yy < y + h; yy++) {
        uint16_t *row = &framebuffer[yy * LCD_H_RES + x];
        for (int xx = 0; xx < w; xx++)
            row[xx] = color;
    }
}

static void fb_pixel(int x, int y, uint16_t color)
{
    if (x < 0 || x >= LCD_H_RES || y < 0 || y >= LCD_V_RES) return;
    framebuffer[y * LCD_H_RES + x] = color;
}

/* ASCII 32..127, 5x7 font. */
static const uint8_t font5x7[96][5] = {
    {0,0,0,0,0},{0,0,95,0,0},{0,7,0,7,0},{20,127,20,127,20},
    {36,42,127,42,18},{35,19,8,100,98},{54,73,85,34,80},{0,0,7,0,0},
    {0,28,34,65,0},{0,65,34,28,0},{20,8,62,8,20},{8,8,62,8,8},
    {0,80,48,0,0},{8,8,8,8,8},{0,96,96,0,0},{32,16,8,4,2},
    {62,81,73,69,62},{0,66,127,64,0},{98,81,73,73,70},{34,65,73,73,54},
    {24,20,18,127,16},{39,69,69,69,57},{60,74,73,73,48},{1,113,9,5,3},
    {54,73,73,73,54},{6,73,73,41,30},{0,54,54,0,0},{0,86,54,0,0},
    {8,20,34,65,0},{20,20,20,20,20},{65,34,20,8,0},{2,1,81,9,6},
    {50,73,121,65,62},{126,17,17,17,126},{127,73,73,73,54},{62,65,65,65,34},
    {127,65,65,34,28},{127,73,73,73,65},{127,9,9,9,1},{62,65,73,73,122},
    {127,8,8,8,127},{0,65,127,65,0},{32,64,65,63,1},{127,8,20,34,65},
    {127,64,64,64,64},{127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},
    {127,9,9,9,6},{62,65,81,33,94},{127,9,25,41,70},{38,73,73,73,50},
    {1,1,127,1,1},{63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},
    {99,20,8,20,99},{3,4,120,4,3},{97,81,73,69,67},{0,127,65,65,0},
    {2,4,8,16,32},{0,65,65,127,0},{4,2,1,2,4},{64,64,64,64,64},
    {0,1,2,4,0},{32,84,84,84,120},{127,72,68,68,56},{56,68,68,68,0},
    {56,68,68,72,127},{56,84,84,84,24},{8,126,9,1,2},{12,82,82,82,62},
    {127,8,4,4,120},{0,68,125,64,0},{32,64,68,61,0},{127,16,40,68,0},
    {0,65,127,64,0},{124,4,120,4,120},{124,8,4,4,120},{56,68,68,68,56},
    {124,20,20,20,8},{8,20,20,24,124},{124,8,4,4,8},{72,84,84,84,32},
    {4,63,68,64,32},{60,64,64,32,124},{28,32,64,32,28},{60,64,48,64,60},
    {68,40,16,40,68},{12,80,80,80,60},{68,100,84,76,68},{0,8,54,65,0},
    {0,0,127,0,0},{0,65,54,8,0},{16,8,8,16,8},{0,0,0,0,0}
};

static void fb_char(int x, int y, char c, int scale, uint16_t color)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = font5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1U << row)) {
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        fb_pixel(x + col * scale + sx,
                                 y + row * scale + sy,
                                 color);
            }
        }
    }
}

static void fb_text(int x, int y, const char *text,
                    int scale, uint16_t color)
{
    while (*text) {
        fb_char(x, y, *text, scale, color);
        x += 6 * scale;
        text++;
    }
}




static void render_page_header(const char *title,
                               const uint16_t title_color)
{
    fb_text(7, 5, "MUSE 2 EEG", 2, title_color);

    char page_text[16];
    snprintf(
        page_text,
        sizeof(page_text),
        "LIVE EEG"
    );

    fb_text(285, 7, page_text, 1, rgb565(180, 185, 195));

    if (title != NULL) {
        fb_text(7, 25, title, 1, rgb565(155, 160, 170));
    }
}


static void render_calibration_page(void)
{
    mind_control_status_t status;
    char buf[64];

    fb_clear(rgb565(10, 12, 16));

    mind_control_get_status(&status);

    render_page_header(
        "CALIBRATION",
        rgb565(255, 190, 80)
    );

    switch (status.state) {
        case MIND_STATE_BASELINE:
            snprintf(buf, sizeof(buf), "BASELINE");
            break;

        case MIND_STATE_BUTTON_1:
            snprintf(buf, sizeof(buf), "CAPTURE B1");
            break;

        case MIND_STATE_BUTTON_2:
            snprintf(buf, sizeof(buf), "CAPTURE B2");
            break;

        case MIND_STATE_BUTTON_3:
            snprintf(buf, sizeof(buf), "CAPTURE B3");
            break;

        case MIND_STATE_BUTTON_4:
            snprintf(buf, sizeof(buf), "CAPTURE B4");
            break;

        case MIND_STATE_READY:
            snprintf(buf, sizeof(buf), "READY");
            break;

        default:
            snprintf(buf, sizeof(buf), "IDLE");
            break;
    }

    fb_text(
        10, 35,
        buf, 2,
        rgb565(255, 255, 255)
    );

    if (status.state == MIND_STATE_BASELINE ||
        status.state == MIND_STATE_BUTTON_1 ||
        status.state == MIND_STATE_BUTTON_2 ||
        status.state == MIND_STATE_BUTTON_3 ||
        status.state == MIND_STATE_BUTTON_4) {

        snprintf(
            buf,
            sizeof(buf),
            "%.1f / %.1f s",
            (double)status.elapsed_ms / 1000.0,
            (double)status.target_ms / 1000.0
        );

    } else {
        snprintf(
            buf,
            sizeof(buf),
            "3.0 s"
        );
    }

    fb_text(
        220, 38,
        buf, 2,
        rgb565(220, 225, 235)
    );

    fb_rect(
        10, 70, 145, 27,
        status.baseline_valid
            ? rgb565(30, 75, 55)
            : rgb565(35, 38, 45)
    );

    fb_text(
        18, 78,
        status.baseline_valid
            ? "BASE READY"
            : "BASELINE",
        1,
        status.baseline_valid
            ? rgb565(120, 255, 170)
            : rgb565(180, 185, 195)
    );

    for (int i = 0; i < MIND_BUTTON_COUNT; ++i) {
        int x = 165 + (i % 2) * 75;
        int y = 70 + (i / 2) * 31;

        char label[12];

        snprintf(
            label,
            sizeof(label),
            "B%d %s",
            i + 1,
            status.button_valid[i] ? "OK" : "---"
        );

        fb_rect(
            x, y, 68, 27,
            status.button_valid[i]
                ? rgb565(30, 75, 55)
                : rgb565(35, 38, 45)
        );

        fb_text(
            x + 7, y + 8,
            label,
            1,
            status.button_valid[i]
                ? rgb565(120, 255, 170)
                : rgb565(150, 155, 165)
        );
    }

    fb_rect(10, 140, 145, 30, rgb565(35, 55, 80));
    fb_text(25, 150, "BASELINE", 1, rgb565(120, 220, 255));

    fb_rect(165, 140, 65, 30, rgb565(35, 55, 80));
    fb_text(182, 150, "B1", 1, rgb565(120, 220, 255));

    fb_rect(240, 140, 65, 30, rgb565(35, 55, 80));
    fb_text(257, 150, "B2", 1, rgb565(120, 220, 255));

    fb_rect(165, 178, 65, 30, rgb565(35, 55, 80));
    fb_text(182, 188, "B3", 1, rgb565(120, 220, 255));

    fb_rect(240, 178, 65, 30, rgb565(35, 55, 80));
    fb_text(257, 188, "B4", 1, rgb565(120, 220, 255));

    fb_rect(10, 178, 70, 30, rgb565(45, 75, 55));
    fb_text(27, 188, "SAVE", 1, rgb565(130, 255, 175));

    fb_rect(90, 178, 65, 30, rgb565(75, 45, 45));
    fb_text(105, 188, "CLEAR", 1, rgb565(255, 150, 150));

    fb_text(
        10, 218,
        "3 SECOND CAPTURE PER STEP",
        1,
        rgb565(145, 150, 160)
    );
}


static bool mind_control_calibration_touch(
    uint16_t x,
    uint16_t y
)
{
    if (eeg_current_page != 3)
        return false;

    if (x >= 10 && x < 155 &&
        y >= 140 && y < 170) {
        ESP_LOGI("EEG_TOUCH", "CAL: BASELINE");
        mind_control_start_baseline();
        return true;
    }

    if (x >= 165 && x < 230 &&
        y >= 140 && y < 170) {
        ESP_LOGI("EEG_TOUCH", "CAL: B1");
        mind_control_start_button(1);
        return true;
    }

    if (x >= 240 && x < 305 &&
        y >= 140 && y < 170) {
        ESP_LOGI("EEG_TOUCH", "CAL: B2");
        mind_control_start_button(2);
        return true;
    }

    if (x >= 165 && x < 230 &&
        y >= 178 && y < 208) {
        ESP_LOGI("EEG_TOUCH", "CAL: B3");
        mind_control_start_button(3);
        return true;
    }

    if (x >= 240 && x < 305 &&
        y >= 178 && y < 208) {
        ESP_LOGI("EEG_TOUCH", "CAL: B4");
        mind_control_start_button(4);
        return true;
    }

    if (x >= 10 && x < 80 &&
        y >= 178 && y < 208) {
        ESP_LOGI("EEG_TOUCH", "CAL: SAVE");
        mind_control_save();
        return true;
    }

    if (x >= 90 && x < 155 &&
        y >= 178 && y < 208) {
        ESP_LOGI("EEG_TOUCH", "CAL: CLEAR");
        mind_control_clear();
        return true;
    }

    return false;
}



/* ============================================================
 * PAGE 0 - EEG OVERVIEW
 * ============================================================ */




/* ============================================================
 * PAGE 1 - BRAIN BANDS
 * ============================================================ */




/* ============================================================
 * PAGE 2 - CHANNEL DETAILS
 * ============================================================ */




/* ============================================================
 * PAGE 3 - SYSTEM STATUS
 * ============================================================ */




/* ============================================================
 * MAIN EEG RENDERER
 * ============================================================ */



/* ============================================================
 * FOUR CHANNEL LIVE EEG WAVEFORMS
 *
 * 256 samples/channel = approximately 1 second at 256 Hz.
 * ============================================================ */

static void render_page_waveforms(void)
{
    /*
     * Snapshot the waveform buffers so the DSP task never gets
     * blocked while the LCD framebuffer is being rendered.
     */
    float snapshot[EEG_CH_COUNT][WAVEFORM_SAMPLES];
    uint16_t filled[EEG_CH_COUNT];
    uint16_t write_index[EEG_CH_COUNT];

    memset(snapshot, 0, sizeof(snapshot));
    memset(filled, 0, sizeof(filled));
    memset(write_index, 0, sizeof(write_index));

    if (ui_mutex != NULL &&
        xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {

        memcpy(snapshot, waveform_data, sizeof(snapshot));
        memcpy(filled, waveform_filled, sizeof(filled));
        memcpy(write_index,
               waveform_write_index,
               sizeof(write_index));

        xSemaphoreGive(ui_mutex);
    }

    const uint16_t bg       = rgb565(5, 8, 12);
    const uint16_t grid     = rgb565(35, 42, 50);
    const uint16_t center   = rgb565(65, 72, 82);
    const uint16_t text     = rgb565(225, 230, 238);
    const uint16_t subtitle = rgb565(130, 140, 152);
    const uint16_t live     = rgb565(40, 230, 150);

    const uint16_t wave_colors[EEG_CH_COUNT] = {
        rgb565(0, 235, 190),
        rgb565(60, 180, 255),
        rgb565(255, 205, 60),
        rgb565(220, 100, 255)
    };

    const char *names[EEG_CH_COUNT] = {
        "TP9", "AF7", "AF8", "TP10"
    };

    fb_clear(bg);

    /*
     * Header.
     */
    fb_text(
        6,
        4,
        "LIVE EEG",
        2,
        text
    );

    fb_text(
        276,
        7,
        "LIVE",
        1,
        live
    );

    /*
     * Four compact waveform windows.
     */
    const int graph_x = 34;
    const int graph_w = 282;
    const int graph_h = 44;

    const int graph_y[EEG_CH_COUNT] = {
        29, 79, 129, 179
    };

    for (int ch = 0; ch < EEG_CH_COUNT; ch++) {

        const int y = graph_y[ch];
        const int center_y = y + graph_h / 2;

        /*
         * Channel label.
         */
        fb_text(
            2,
            y + 17,
            names[ch],
            1,
            text
        );

        /*
         * Graph background.
         */
        fb_rect(
            graph_x + 1,
            y + 1,
            graph_w - 2,
            graph_h - 2,
            bg
        );

        /*
         * Border.
         */
        for (int x = graph_x;
             x < graph_x + graph_w;
             x++) {

            fb_pixel(x, y, grid);
            fb_pixel(x, y + graph_h - 1, grid);
        }

        for (int yy = y;
             yy < y + graph_h;
             yy++) {

            fb_pixel(graph_x, yy, grid);
            fb_pixel(graph_x + graph_w - 1, yy, grid);
        }

        /*
         * Horizontal center/reference line.
         */
        for (int x = graph_x + 1;
             x < graph_x + graph_w - 1;
             x += 4) {

            fb_pixel(x, center_y, center);
        }

        /*
         * Vertical timing grid.
         */
        for (int x = graph_x + 1;
             x < graph_x + graph_w - 1;
             x += 35) {

            for (int yy = y + 2;
                 yy < y + graph_h - 2;
                 yy += 4) {

                fb_pixel(x, yy, grid);
            }
        }

        /*
         * Not enough data yet.
         */
        if (filled[ch] < 2) {
            fb_text(
                graph_x + 8,
                y + 16,
                "WAITING...",
                1,
                subtitle
            );
            continue;
        }

        /*
         * Find signal range.
         *
         * Keep the existing automatic scaling for now.
         * This makes the EEG immediately visible regardless
         * of the raw ADC offset.
         */
        float min_v = snapshot[ch][0];
        float max_v = snapshot[ch][0];

        for (int i = 1; i < filled[ch]; i++) {

            if (snapshot[ch][i] < min_v)
                min_v = snapshot[ch][i];

            if (snapshot[ch][i] > max_v)
                max_v = snapshot[ch][i];
        }

        float range = max_v - min_v;

        if (range < 0.000001f) {
            range = 0.000001f;
            min_v -= 0.0000005f;
        }

        /*
         * Prevent the trace from constantly touching
         * the borders.
         */
        float margin = range * 0.10f;

        min_v -= margin;
        range += 2.0f * margin;

        int previous_x = graph_x + 1;
        int previous_y = center_y;

        for (int i = 0; i < filled[ch]; i++) {

            int source_index;

            if (filled[ch] < WAVEFORM_SAMPLES) {
                source_index = i;
            } else {
                source_index =
                    (write_index[ch] + i) %
                    WAVEFORM_SAMPLES;
            }

            float v = snapshot[ch][source_index];

            float normalized =
                (v - min_v) / range;

            if (normalized < 0.0f)
                normalized = 0.0f;

            if (normalized > 1.0f)
                normalized = 1.0f;

            int px =
                graph_x + 1 +
                (i * (graph_w - 3)) /
                (filled[ch] - 1);

            int py =
                y + graph_h - 2 -
                (int)(normalized * (graph_h - 3));

            if (py < y + 1)
                py = y + 1;

            if (py > y + graph_h - 2)
                py = y + graph_h - 2;

            if (i > 0) {

                int x0 = previous_x;
                int y0 = previous_y;
                int x1 = px;
                int y1 = py;

                int dx = abs(x1 - x0);
                int sx = (x0 < x1) ? 1 : -1;

                int dy = -abs(y1 - y0);
                int sy = (y0 < y1) ? 1 : -1;

                int err = dx + dy;

                while (true) {

                    fb_pixel(
                        x0,
                        y0,
                        wave_colors[ch]
                    );

                    if (x0 == x1 && y0 == y1)
                        break;

                    int e2 = 2 * err;

                    if (e2 >= dy) {
                        err += dy;
                        x0 += sx;
                    }

                    if (e2 <= dx) {
                        err += dx;
                        y0 += sy;
                    }
                }
            }

            previous_x = px;
            previous_y = py;
        }
    }

    /*
     * Footer.
     */
    fb_text(
        66,
        230,
        "4 CH  |  256 SAMPLES",
        1,
        subtitle
    );

    /*
     * Send completed framebuffer to ST7789.
     */
    if (panel != NULL) {

        esp_err_t err = esp_lcd_panel_draw_bitmap(
            panel,
            0,
            0,
            LCD_H_RES,
            LCD_V_RES,
            framebuffer
        );

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "LCD draw_bitmap failed: %s",
                esp_err_to_name(err)
            );
        }
    }
}


/* ============================================================
 * Push decoded EEG samples into waveform ring buffers.
 *
 * Called by the DSP task, never from the NimBLE callback.
 * ============================================================ */

void eeg_display_push_samples(
    eeg_channel_t channel,
    const float *samples,
    size_t count
)
{
    if (channel >= EEG_CH_COUNT ||
        samples == NULL ||
        count == 0 ||
        ui_mutex == NULL) {
        return;
    }

    /*
     * Never make the DSP task wait for the display.
     */
    if (xSemaphoreTake(ui_mutex, 0) != pdTRUE)
        return;

    for (size_t i = 0; i < count; i++) {

        waveform_data[channel][waveform_write_index[channel]] =
            samples[i];

        waveform_write_index[channel]++;

        if (waveform_write_index[channel] >= WAVEFORM_SAMPLES)
            waveform_write_index[channel] = 0;

        if (waveform_filled[channel] < WAVEFORM_SAMPLES)
            waveform_filled[channel]++;
    }

    xSemaphoreGive(ui_mutex);
}









static void heart_text(
    int x,
    int y,
    const char *text,
    uint16_t color,
    uint8_t scale
)
{
    fb_text(x, y, text, scale, color);
}

static void render_page_heart(void)
{
{
    ppg_hr_metrics_t m;
    ppg_hr_get_metrics(&m);

    const uint16_t bg    = rgb565(8, 10, 14);
    const uint16_t text  = rgb565(220, 225, 235);
    const uint16_t dim   = rgb565(130, 140, 155);
    const uint16_t accent = rgb565(0, 220, 180);

    fb_rect(
        0,
        0,
        LCD_H_RES,
        LCD_V_RES,
        bg
    );

    heart_text(
        8,
        5,
        "HEART / PPG",
        text,
        1
    );

    heart_text(
        268,
        5,
        "2/2",
        dim,
        1
    );

    char line[64];

    /* --------------------------------------------------------
     * Large heart rate
     * -------------------------------------------------------- */

    if (m.valid) {
        snprintf(
            line,
            sizeof(line),
            "%3d",
            (int)(m.bpm + 0.5f)
        );

        heart_text(
            18,
            28,
            line,
            accent,
            3
        );

        heart_text(
            92,
            39,
            "BPM",
            text,
            2
        );
    } else {
        heart_text(
            18,
            28,
            "--",
            dim,
            3
        );

        heart_text(
            92,
            39,
            "BPM",
            dim,
            2
        );
    }

    fb_rect(
        8,
        72,
        304,
        1,
        dim
    );

    /* --------------------------------------------------------
     * Status
     * -------------------------------------------------------- */

    heart_text(
        10,
        80,
        m.valid ? "STATUS  LOCKED" : "STATUS  SEARCHING",
        m.valid ? accent : dim,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "IBI      %4.0f ms",
        m.valid ? m.ibi_ms : 0.0f
    );

    heart_text(
        10,
        101,
        m.valid ? line : "IBI      --",
        text,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "QUALITY  %3d%%",
        (int)(m.quality + 0.5f)
    );

    heart_text(
        10,
        122,
        line,
        text,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "BEATS    %lu",
        (unsigned long)m.beats
    );

    heart_text(
        170,
        101,
        line,
        text,
        1
    );

    const char *active =
        (m.active_channel == 0) ? "PPG1" :
        (m.active_channel == 1) ? "PPG2" :
        (m.active_channel == 2) ? "PPG3" :
        "---";

    snprintf(
        line,
        sizeof(line),
        "ACTIVE   %s",
        active
    );

    heart_text(
        170,
        122,
        line,
        text,
        1
    );

    /* --------------------------------------------------------
     * Raw PPG channel values
     * -------------------------------------------------------- */

    snprintf(
        line,
        sizeof(line),
        "PPG1     %lu",
        (unsigned long)m.raw[0]
    );

    heart_text(
        10,
        147,
        line,
        text,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "PPG2     %lu",
        (unsigned long)m.raw[1]
    );

    heart_text(
        10,
        168,
        line,
        text,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "PPG3     %lu",
        (unsigned long)m.raw[2]
    );

    heart_text(
        10,
        189,
        line,
        text,
        1
    );

    snprintf(
        line,
        sizeof(line),
        "SAMPLES  %lu",
        (unsigned long)m.sample_count
    );

    heart_text(
        170,
        147,
        line,
        text,
        1
    );

    heart_text(
        170,
        168,
        "FS       64 Hz",
        text,
        1
    );

    heart_text(
        170,
        189,
        "PPG      3 CH",
        text,
        1
    );

    fb_rect(
        8,
        211,
        304,
        1,
        dim
    );

    heart_text(
        10,
        218,
        "TOUCH: NEXT / PREVIOUS",
        dim,
        1
    );
}
}


/* ============================================================
 * New NVS-backed MIND CONTROL display
 * ============================================================ */
/* ============================================================
 * Mind-control appliance switch state
 * B1 = LIGHT, B2 = FAN, B3 = DIM LIGHT, B4 = AC
 *
 * The latch ensures one detected command produces one toggle.
 * The existing EEG -> mind_control_update_features() path is
 * unchanged.
 * ============================================================ */
static bool mind_switch_on[MIND_BUTTON_COUNT] = {
    false, false, false, false
};

static uint8_t mind_last_detection = 0;

static void update_mind_switches(
    const mind_control_status_t *status)
{
    uint8_t detected = status->detected_button;

    if (detected >= 1 && detected <= MIND_BUTTON_COUNT) {

        /*
         * Only toggle when this is a newly detected command.
         * Do not repeatedly toggle while detection remains active.
         */
        /*
         * Toggle once for each newly detected button.
         * This handles both:
         *   NONE -> B1
         *   B1 -> NONE -> B1
         *   B1 -> B2
         *
         * Repeated frames of the same detection do not
         * toggle the switch again.
         */
        if (detected != mind_last_detection) {
            mind_switch_on[detected - 1] =
                !mind_switch_on[detected - 1];
        }

        mind_last_detection = detected;

    } else {
        /*
         * Detection cleared. The next command is allowed
         * to toggle again.
         */
        mind_last_detection = 0;
    }
}


static void render_page_mind_control_new(void)
{
    mind_control_status_t status;
    char buf[64];

    fb_clear(rgb565(10, 12, 16));

    mind_control_get_status(&status);
    update_mind_switches(&status);

    render_page_header(
        "MIND CONTROL",
        rgb565(100, 220, 255)
    );

    /*
     * Detection / profile information.
     */
    if (status.detected_button >= 1 &&
        status.detected_button <= MIND_BUTTON_COUNT) {

        static const char *const names[MIND_BUTTON_COUNT] = {
            "LIGHT",
            "FAN",
            "DIM LIGHT",
            "AC"
        };

        snprintf(
            buf,
            sizeof(buf),
            "DETECTED: %s",
            names[status.detected_button - 1]
        );

    } else {
        snprintf(buf, sizeof(buf), "DETECTED: NONE");
    }

    fb_text(
        10, 34,
        buf, 1,
        rgb565(255, 255, 255)
    );

    snprintf(
        buf,
        sizeof(buf),
        "CONF %.2f  DIST %.2f",
        (double)status.confidence,
        (double)status.distance
    );

    fb_text(
        10, 51,
        buf, 1,
        rgb565(190, 195, 205)
    );

    snprintf(
        buf,
        sizeof(buf),
        "PROFILES %s",
        status.profiles_saved ? "SAVED" : "NOT SAVED"
    );

    fb_text(
        205, 34,
        buf, 1,
        status.profiles_saved
            ? rgb565(120, 255, 170)
            : rgb565(180, 185, 195)
    );

    /*
     * Four appliance toggle switches.
     */
    static const char *const names[MIND_BUTTON_COUNT] = {
        "LIGHT",
        "FAN",
        "DIM LIGHT",
        "AC"
    };

    for (int i = 0; i < MIND_BUTTON_COUNT; ++i) {

        int y = 75 + i * 37;

        bool on = mind_switch_on[i];

        /*
         * Whole switch body:
         * green = ON
         * red   = OFF
         */
        fb_rect(
            10, y, 300, 31,
            on
                ? rgb565(30, 105, 65)
                : rgb565(110, 40, 40)
        );

        fb_text(
            20, y + 9,
            names[i],
            1,
            rgb565(255, 255, 255)
        );

        /*
         * ON/OFF indicator.
         */
        fb_rect(
            245, y + 4, 55, 23,
            on
                ? rgb565(60, 180, 95)
                : rgb565(180, 55, 55)
        );

        fb_text(
            on ? 259 : 257,
            y + 10,
            on ? "ON" : "OFF",
            1,
            rgb565(255, 255, 255)
        );
    }

    fb_text(
        10, 226,
        "B1 LIGHT  B2 FAN  B3 DIM  B4 AC",
        1,
        rgb565(140, 145, 155)
    );
}

static void render_dashboard(void)
{
    switch (eeg_current_page) {
        case 0:
            render_page_waveforms();
            break;

        case 1:
            render_page_heart();
            break;

        case 2:
            render_page_mind_control_new();
            break;

        case 3:
            render_calibration_page();
            break;

        default:
            eeg_current_page = 0;
            render_page_waveforms();
            break;
    }

    if (panel != NULL) {
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            panel,
            0,
            0,
            LCD_H_RES,
            LCD_V_RES,
            framebuffer
        );

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "LCD draw_bitmap failed: %s",
                esp_err_to_name(err)
            );
        }
    }
}




static void display_task(void *arg)
{
    (void)arg;

    while (true) {
        render_dashboard();
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

static void lcd_init(void)
{
    gpio_config_t bk_cfg = {
        .pin_bit_mask = 1ULL << LCD_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&bk_cfg));
    gpio_set_level(LCD_BK_LIGHT, 1);

    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2 + 8
    };

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            LCD_HOST, &buscfg, SPI_DMA_CH_AUTO
        )
    );

    esp_lcd_panel_io_handle_t io = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi(
            (esp_lcd_spi_bus_handle_t)LCD_HOST,
            &io_config,
            &io
        )
    );

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_st7789(
            io, &panel_config, &panel
        )
    );

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    ESP_LOGI(TAG, "ST7789 initialized: 320x240 SPI2 20MHz");
}

void eeg_display_start(void)
{
    if (panel != NULL) return;

    memset(ui_data, 0, sizeof(ui_data));

    ui_mutex = xSemaphoreCreateMutex();
    if (ui_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UI mutex");
        return;
    }

    lcd_init();


    touch_init();

    if (xTaskCreate(
            display_task,
            "eeg_lcd",
            DISPLAY_TASK_STACK,
            NULL,
            DISPLAY_TASK_PRIORITY,
            NULL
        ) != pdPASS) {

        ESP_LOGE(TAG, "Failed to create EEG LCD task");
        return;
    }

    ESP_LOGI(TAG, "EEG color dashboard started");

    touch_start_task();
}

void eeg_display_update(
    eeg_channel_t channel,
    float delta,
    float theta,
    float alpha,
    float beta,
    float gamma,
    float peak_frequency)
{
    if (channel >= EEG_CH_COUNT || ui_mutex == NULL)
        return;

    /*
     * Non-blocking. DSP never waits for the LCD.
     */
    if (xSemaphoreTake(ui_mutex, 0) == pdTRUE) {
        ui_data[channel].delta = delta;
        ui_data[channel].theta = theta;
        ui_data[channel].alpha = alpha;
        ui_data[channel].beta = beta;
        ui_data[channel].gamma = gamma;
        ui_data[channel].peak = peak_frequency;
        ui_data[channel].valid = true;

        xSemaphoreGive(ui_mutex);
    }
}



/* ========================================================================= */
/* SD EEG CALIBRATION / SAMPLE DATABASE                                     */
/* ========================================================================= */

/* TOUCH SWIPE ENGINE                                                       */
/* ========================================================================= */

#define SWIPE_TAG "EEG_TOUCH"

#define SWIPE_MIN_DISTANCE       50
#define SWIPE_TAP_DISTANCE       20

static bool touch_active = false;

static uint16_t touch_start_x = 0;
static uint16_t touch_start_y = 0;

static uint16_t touch_last_x = 0;
static uint16_t touch_last_y = 0;


/*
 * Gesture action.
 *
 * LEFT  = next EEG page
 * RIGHT = previous EEG page
 *
 * UP/DOWN are currently reported and can be assigned later.
 */


/* ============================================================
 * EEG PAGE NAVIGATION
 * ============================================================ */


void eeg_display_next_page(void)
{
    eeg_current_page++;

    if (eeg_current_page >= EEG_UI_PAGE_COUNT)
        eeg_current_page = 0;

    ESP_LOGI(
        SWIPE_TAG,
        "NEXT PAGE -> %d",
        eeg_current_page + 1
    );
}

void eeg_display_previous_page(void)
{
    eeg_current_page--;

    if (eeg_current_page < 0)
        eeg_current_page = EEG_UI_PAGE_COUNT - 1;

    ESP_LOGI(
        SWIPE_TAG,
        "PREVIOUS PAGE -> %d",
        eeg_current_page + 1
    );
}


static void eeg_handle_gesture(touch_gesture_t gesture)
{
    switch (gesture) {

        case TOUCH_GESTURE_LEFT:

            ESP_LOGI(
                SWIPE_TAG,
                "SWIPE LEFT -> NEXT PAGE"
            );

            eeg_display_next_page();

            break;


        case TOUCH_GESTURE_RIGHT:

            ESP_LOGI(
                SWIPE_TAG,
                "SWIPE RIGHT -> PREVIOUS PAGE"
            );

            eeg_display_previous_page();

            break;


        case TOUCH_GESTURE_UP:

            ESP_LOGI(
                SWIPE_TAG,
                "SWIPE UP"
            );

            break;


        case TOUCH_GESTURE_DOWN:

            ESP_LOGI(
                SWIPE_TAG,
                "SWIPE DOWN"
            );

            break;


        case TOUCH_GESTURE_TAP:

            ESP_LOGI(
                SWIPE_TAG,
                "TAP"
            );

            if (mind_control_calibration_touch(
                    touch_last_x,
                    touch_last_y)) {
                break;
            }

            break;


        default:
            break;
    }
}


/*
 * Feed calibrated touchscreen coordinates into this function.
 *
 * pressed = true:
 *     finger currently touching screen
 *
 * pressed = false:
 *     finger released
 */
void eeg_display_touch(uint16_t x,
                       uint16_t y,
                       bool pressed)
{
    /*
     * Finger DOWN.
     */
    if (pressed && !touch_active) {

        touch_active = true;

        touch_start_x = x;
        touch_start_y = y;

        touch_last_x = x;
        touch_last_y = y;

        /*
         * Calibration controls are direct buttons.
         * Handle them on finger-down so they do not depend
         * on the swipe/tap gesture thresholds.
         */
        calibration_touch_handled =
            mind_control_calibration_touch(x, y);

        return;
    }


    /*
     * Finger MOVING.
     */
    if (pressed && touch_active) {

        touch_last_x = x;
        touch_last_y = y;

        return;
    }


    /*
     * Finger UP.
     */
    if (!pressed && touch_active) {

        touch_active = false;

        /*
         * If Page 3 already handled this touch as a button,
         * do not process the release as a second TAP.
         */
        if (calibration_touch_handled) {
            calibration_touch_handled = false;
            return;
        }

        int dx =
            (int)touch_last_x -
            (int)touch_start_x;

        int dy =
            (int)touch_last_y -
            (int)touch_start_y;

        int abs_dx = dx < 0 ? -dx : dx;
        int abs_dy = dy < 0 ? -dy : dy;


        /*
         * Small movement = TAP.
         */
        if (abs_dx < SWIPE_TAP_DISTANCE &&
            abs_dy < SWIPE_TAP_DISTANCE) {

            eeg_handle_gesture(
                TOUCH_GESTURE_TAP
            );

            return;
        }


        /*
         * Select the dominant axis.
         *
         * This prevents diagonal movement from producing
         * multiple gestures.
         */
        if (abs_dx > abs_dy) {

            if (abs_dx >= SWIPE_MIN_DISTANCE) {

                if (dx < 0) {

                    eeg_handle_gesture(
                        TOUCH_GESTURE_LEFT
                    );

                } else {

                    eeg_handle_gesture(
                        TOUCH_GESTURE_RIGHT
                    );
                }
            }

        } else {

            if (abs_dy >= SWIPE_MIN_DISTANCE) {

                if (dy < 0) {

                    eeg_handle_gesture(
                        TOUCH_GESTURE_UP
                    );

                } else {

                    eeg_handle_gesture(
                        TOUCH_GESTURE_DOWN
                    );
                }
            }
        }
    }
}



void eeg_display_set_heart_metrics(
    const eeg_heart_metrics_t *metrics)
{
    if (metrics == NULL) {
        return;
    }

    heart_metrics = *metrics;

    if (metrics->beat) {
        heart_beat_flash_until = esp_log_timestamp() + 120;
    }
}





