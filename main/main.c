#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#define MODEM_UART_NUM UART_NUM_1

/*
   Verdrahtung:

   ESP32-P4 GPIO7  TX  -> SIM7600 RXD
   ESP32-P4 GPIO8  RX  <- SIM7600 TXD
   ESP32-P4 GND        -> SIM7600 GND

   SIM7600 UART-Jumper:
   Position B
*/
#define MODEM_TX_GPIO 7
#define MODEM_RX_GPIO 8

#define UART_BUF_SIZE 2048

static const char *TAG = "SIM7600_GPS";

static void modem_send_raw(const char *data)
{
    uart_write_bytes(MODEM_UART_NUM, data, strlen(data));
}

static void modem_send_cmd(const char *cmd)
{
    ESP_LOGI(TAG, "TX: %s", cmd);
    modem_send_raw(cmd);
    modem_send_raw("\r\n");
}

static bool read_response(int timeout_ms)
{
    uint8_t data[UART_BUF_SIZE];
    int64_t start_ms = esp_timer_get_time() / 1000;
    bool got_anything = false;

    while ((esp_timer_get_time() / 1000) - start_ms < timeout_ms) {
        int len = uart_read_bytes(
            MODEM_UART_NUM,
            data,
            sizeof(data) - 1,
            pdMS_TO_TICKS(200)
        );

        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "RX: %s", (char *)data);
            got_anything = true;
        }
    }

    return got_anything;
}

static void uart_setup(int baudrate)
{
    uart_driver_delete(MODEM_UART_NUM);

    uart_config_t uart_config = {
        .baud_rate = baudrate,
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

static bool test_baudrate(int baudrate)
{
    ESP_LOGI(TAG, "Testing baudrate: %d", baudrate);

    uart_setup(baudrate);

    vTaskDelay(pdMS_TO_TICKS(500));

    modem_send_cmd("AT");

    if (read_response(2000)) {
        ESP_LOGI(TAG, "SUCCESS: SIM7600 responded at %d baud", baudrate);
        return true;
    }

    ESP_LOGW(TAG, "No response at %d baud", baudrate);
    return false;
}

static void run_modem_init(void)
{
    modem_send_cmd("ATE0");
    read_response(2000);

    modem_send_cmd("ATI");
    read_response(3000);

    modem_send_cmd("AT+CPIN?");
    read_response(3000);

    modem_send_cmd("AT+CSQ");
    read_response(3000);

    modem_send_cmd("AT+CREG?");
    read_response(3000);

    modem_send_cmd("AT+COPS?");
    read_response(3000);

    modem_send_cmd("AT+CGPS=1");
    read_response(3000);
}

static void gps_loop(void)
{
    while (1) {
        modem_send_cmd("AT+CGPSINFO");
        read_response(5000);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SIM7600 UART GPS test started");
    ESP_LOGI(TAG, "Expected wiring:");
    ESP_LOGI(TAG, "ESP GPIO%d TX -> SIM7600 RXD", MODEM_TX_GPIO);
    ESP_LOGI(TAG, "ESP GPIO%d RX <- SIM7600 TXD", MODEM_RX_GPIO);
    ESP_LOGI(TAG, "SIM7600 UART jumpers must be on B");
    ESP_LOGI(TAG, "GND between ESP32-P4 and SIM7600 must be connected");

    vTaskDelay(pdMS_TO_TICKS(3000));

    int baudrates[] = {
        115200,
        9600,
        57600,
        38400,
        19200
    };

    while (1) {
        for (int i = 0; i < (int)(sizeof(baudrates) / sizeof(baudrates[0])); i++) {
            int baudrate = baudrates[i];

            if (test_baudrate(baudrate)) {
                run_modem_init();
                gps_loop();
            }
        }

        ESP_LOGW(TAG, "No baudrate worked.");
        ESP_LOGW(TAG, "Check: TX/RX, GND, jumper B, SIM7600 power, and correct header pins.");

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
