#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

   Flash / Monitor:
   /dev/cu.SLAB_USBtoUART
*/

#define MODEM_TX_GPIO 21
#define MODEM_RX_GPIO 22

#define MODEM_BAUDRATE 115200
#define UART_BUF_SIZE 2048

static const char *TAG = "GPS_STABLE";

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

    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, "LCD OK - SIM7600 GPS aktiv");
    lv_obj_set_style_text_color(status, lv_color_hex(0x44ff44), 0);
    lv_obj_align(status, LV_ALIGN_TOP_LEFT, 35, 75);

    lv_obj_t *uart = lv_label_create(scr);
    lv_label_set_text(uart, "UART: GPIO21 TX -> RXD | GPIO22 RX <- TXD");
    lv_obj_set_style_text_color(uart, lv_color_hex(0xffffff), 0);
    lv_obj_align(uart, LV_ALIGN_TOP_LEFT, 35, 115);

    lv_obj_t *gps = lv_label_create(scr);
    lv_label_set_text(gps, "GPS-Daten erscheinen aktuell im Terminal.");
    lv_obj_set_style_text_color(gps, lv_color_hex(0xffffff), 0);
    lv_obj_align(gps, LV_ALIGN_TOP_LEFT, 35, 155);

    lv_obj_t *info = lv_label_create(scr);
    lv_label_set_text(info, "Stabile Basisversion ohne Track-Zeichnung.");
    lv_obj_set_style_text_color(info, lv_color_hex(0xffffff), 0);
    lv_obj_align(info, LV_ALIGN_TOP_LEFT, 35, 195);

    lv_obj_t *map_box = lv_obj_create(scr);
    lv_obj_set_size(map_box, 900, 320);
    lv_obj_align(map_box, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(map_box, lv_color_hex(0x101820), 0);
    lv_obj_set_style_border_color(map_box, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(map_box, 4, 0);
    lv_obj_set_style_radius(map_box, 8, 0);
    lv_obj_clear_flag(map_box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *map_label = lv_label_create(map_box);
    lv_label_set_text(map_label, "Kartenbereich - wird spaeter wieder aktiviert");
    lv_obj_set_style_text_color(map_label, lv_color_hex(0xffffff), 0);
    lv_obj_center(map_label);
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
        read_uart(response, sizeof(response), 5000);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting stable GPS LCD version");
    ESP_LOGI(TAG, "ESP GPIO%d TX -> SIM7600 RXD", MODEM_TX_GPIO);
    ESP_LOGI(TAG, "ESP GPIO%d RX <- SIM7600 TXD", MODEM_RX_GPIO);

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

    xTaskCreate(modem_task, "modem_task", 8192, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
