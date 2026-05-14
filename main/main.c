#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "lvgl.h"
#include "bsp/esp-bsp.h"

#define MODEM_UART_NUM UART_NUM_1

/*
   Funktionierende Verdrahtung:

   ESP32-P4 GPIO21 TX -> SIM7600 RXD
   ESP32-P4 GPIO22 RX <- SIM7600 TXD
   ESP32-P4 GND       -> SIM7600 GND

   SIM7600 UART-Jumper:
   Position B

   LCD:
   PWM / Backlight -> GPIO23
   RST_LCD         -> GPIO27
*/
#define MODEM_TX_GPIO 21
#define MODEM_RX_GPIO 22

#define MODEM_BAUDRATE 115200
#define UART_BUF_SIZE 2048

static const char *TAG = "GPS_LCD";

typedef struct {
    double lat;
    double lon;
    bool valid;
} gps_position_t;

static SemaphoreHandle_t gps_mutex;
static gps_position_t gps_pos = {0};
static int gps_updates = 0;

static lv_obj_t *label_status;
static lv_obj_t *label_lat;
static lv_obj_t *label_lon;
static lv_obj_t *label_updates;
static lv_obj_t *label_raw;
static lv_obj_t *map_box;
static lv_obj_t *position_dot;

static void modem_send(const char *cmd)
{
    ESP_LOGI(TAG, "TX: %s", cmd);
    uart_write_bytes(MODEM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_NUM, "\r\n", 2);
}

static int read_uart(char *out, size_t out_size, int timeout_ms)
{
    int total = 0;
    int elapsed = 0;

    if (out_size == 0) {
        return 0;
    }

    out[0] = '\0';

    while (elapsed < timeout_ms && total < (int)out_size - 1) {
        uint8_t data[256];

        int len = uart_read_bytes(
            MODEM_UART_NUM,
            data,
            sizeof(data) - 1,
            pdMS_TO_TICKS(200)
        );

        if (len > 0) {
            if (total + len >= (int)out_size) {
                len = (int)out_size - 1 - total;
            }

            memcpy(out + total, data, len);
            total += len;
            out[total] = '\0';

            ESP_LOGI(TAG, "RX: %.*s", len, (char *)data);
        }

        elapsed += 200;
    }

    return total;
}

static double nmea_to_decimal(const char *value, char hemi)
{
    double raw = atof(value);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (hemi == 'S' || hemi == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

static bool parse_cgpsinfo(const char *response, gps_position_t *pos)
{
    const char *p = strstr(response, "+CGPSINFO:");
    if (!p) {
        return false;
    }

    p += strlen("+CGPSINFO:");

    while (*p == ' ') {
        p++;
    }

    char lat_raw[24] = {0};
    char lon_raw[24] = {0};
    char lat_hemi = 0;
    char lon_hemi = 0;

    int matched = sscanf(
        p,
        "%23[^,],%c,%23[^,],%c",
        lat_raw,
        &lat_hemi,
        lon_raw,
        &lon_hemi
    );

    if (matched != 4) {
        return false;
    }

    if (strlen(lat_raw) < 3 || strlen(lon_raw) < 4) {
        return false;
    }

    pos->lat = nmea_to_decimal(lat_raw, lat_hemi);
    pos->lon = nmea_to_decimal(lon_raw, lon_hemi);
    pos->valid = true;

    return true;
}

static void uart_init_modem(void)
{
    uart_config_t uart_config = {
        .baud_rate = MODEM_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        MODEM_UART_NUM,
        UART_BUF_SIZE,
        UART_BUF_SIZE,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(MODEM_UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        MODEM_UART_NUM,
        MODEM_TX_GPIO,
        MODEM_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    uart_flush(MODEM_UART_NUM);
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0b0f14), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ESP32-P4 GPS Tracker");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Fix: -");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xff4444), 0);
    lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 35, 70);

    label_lat = lv_label_create(scr);
    lv_label_set_text(label_lat, "Lat: -");
    lv_obj_set_style_text_color(label_lat, lv_color_hex(0xffffff), 0);
    lv_obj_align(label_lat, LV_ALIGN_TOP_LEFT, 35, 105);

    label_lon = lv_label_create(scr);
    lv_label_set_text(label_lon, "Lon: -");
    lv_obj_set_style_text_color(label_lon, lv_color_hex(0xffffff), 0);
    lv_obj_align(label_lon, LV_ALIGN_TOP_LEFT, 35, 140);

    label_updates = lv_label_create(scr);
    lv_label_set_text(label_updates, "Updates: 0");
    lv_obj_set_style_text_color(label_updates, lv_color_hex(0xffffff), 0);
    lv_obj_align(label_updates, LV_ALIGN_TOP_LEFT, 35, 175);

    label_raw = lv_label_create(scr);
    lv_label_set_text(label_raw, "Warte auf GPS-Daten...");
    lv_obj_set_style_text_color(label_raw, lv_color_hex(0xffffff), 0);
    lv_obj_set_width(label_raw, 900);
    lv_obj_align(label_raw, LV_ALIGN_TOP_LEFT, 35, 215);

    map_box = lv_obj_create(scr);
    lv_obj_set_size(map_box, 900, 320);
    lv_obj_align(map_box, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(map_box, lv_color_hex(0x101820), 0);
    lv_obj_set_style_border_color(map_box, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(map_box, 4, 0);
    lv_obj_set_style_radius(map_box, 8, 0);
    lv_obj_clear_flag(map_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *map_label = lv_label_create(map_box);
    lv_label_set_text(map_label, "Lokale Track-Ansicht");
    lv_obj_set_style_text_color(map_label, lv_color_hex(0xffffff), 0);
    lv_obj_align(map_label, LV_ALIGN_TOP_MID, 0, 20);

    position_dot = lv_obj_create(map_box);
    lv_obj_set_size(position_dot, 18, 18);
    lv_obj_set_style_radius(position_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(position_dot, lv_color_hex(0xffcc00), 0);
    lv_obj_set_style_border_width(position_dot, 0, 0);
    lv_obj_center(position_dot);
    lv_obj_add_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
}

static void update_ui(void)
{
    gps_position_t local_pos = {0};
    int local_updates = 0;

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        local_pos = gps_pos;
        local_updates = gps_updates;
        xSemaphoreGive(gps_mutex);
    }

    char updates_buf[64];
    snprintf(updates_buf, sizeof(updates_buf), "Updates: %d", local_updates);
    lv_label_set_text(label_updates, updates_buf);

    if (local_pos.valid) {
        char lat_buf[64];
        char lon_buf[64];

        snprintf(lat_buf, sizeof(lat_buf), "Lat: %.6f", local_pos.lat);
        snprintf(lon_buf, sizeof(lon_buf), "Lon: %.6f", local_pos.lon);

        lv_obj_set_style_text_color(label_status, lv_color_hex(0x44ff44), 0);
        lv_label_set_text(label_status, "Fix: JA");
        lv_label_set_text(label_lat, lat_buf);
        lv_label_set_text(label_lon, lon_buf);
        lv_label_set_text(label_raw, "GPS-Fix aktiv. Position wird lokal angezeigt.");

        lv_obj_clear_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_center(position_dot);
    } else {
        lv_obj_set_style_text_color(label_status, lv_color_hex(0xff4444), 0);
        lv_label_set_text(label_status, "Fix: NEIN");
        lv_label_set_text(label_lat, "Lat: -");
        lv_label_set_text(label_lon, "Lon: -");
        lv_label_set_text(label_raw, "GNSS aktiv. Warte auf gueltigen Fix...");
        lv_obj_add_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
    }
}

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (bsp_display_lock(0)) {
        update_ui();
        bsp_display_unlock();
    }
}

static void modem_task(void *arg)
{
    (void)arg;

    char response[UART_BUF_SIZE];

    vTaskDelay(pdMS_TO_TICKS(2000));

    modem_send("AT");
    read_uart(response, sizeof(response), 2000);

    modem_send("ATE0");
    read_uart(response, sizeof(response), 2000);

    modem_send("ATI");
    read_uart(response, sizeof(response), 3000);

    modem_send("AT+CGPS?");
    read_uart(response, sizeof(response), 3000);

    modem_send("AT+CGPS=1");
    read_uart(response, sizeof(response), 3000);

    while (1) {
        modem_send("AT+CGPSINFO");

        int len = read_uart(response, sizeof(response), 5000);

        if (len > 0) {
            gps_position_t parsed = {0};

            if (parse_cgpsinfo(response, &parsed)) {
                ESP_LOGI(TAG, "GPS FIX: lat=%.6f lon=%.6f", parsed.lat, parsed.lon);

                if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    gps_pos = parsed;
                    gps_updates++;
                    xSemaphoreGive(gps_mutex);
                }
            } else {
                ESP_LOGI(TAG, "No valid GPS fix yet");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting GPS LCD tracker");
    ESP_LOGI(TAG, "ESP GPIO%d TX -> SIM7600 RXD", MODEM_TX_GPIO);
    ESP_LOGI(TAG, "ESP GPIO%d RX <- SIM7600 TXD", MODEM_RX_GPIO);

    gps_mutex = xSemaphoreCreateMutex();

    uart_init_modem();

    bsp_display_start();

    ESP_LOGI(TAG, "Turning LCD backlight on");
    esp_err_t bl_err = bsp_display_backlight_on();
    ESP_LOGI(TAG, "bsp_display_backlight_on: %s", esp_err_to_name(bl_err));

    esp_err_t br_err = bsp_display_brightness_set(100);
    ESP_LOGI(TAG, "bsp_display_brightness_set(100): %s", esp_err_to_name(br_err));

    if (bsp_display_lock(0)) {
        create_ui();
        bsp_display_unlock();
    }

    lv_timer_create(ui_timer_cb, 1000, NULL);

    xTaskCreate(modem_task, "modem_task", 8192, NULL, 5, NULL);
}
