#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"

#define MODEM_UART_NUM UART_NUM_1

// Funktionierende Verdrahtung:
// ESP GPIO7  TX -> SIM7600 RXD
// ESP GPIO8  RX <- SIM7600 TXD
// GND            -> GND
#define MODEM_TX_GPIO 7
#define MODEM_RX_GPIO 8

#define MODEM_BAUDRATE 115200
#define UART_BUF_SIZE 2048

static const char *TAG = "SIM7600_WORKING";

static void modem_send(const char *cmd)
{
    ESP_LOGI(TAG, "TX: %s", cmd);
    uart_write_bytes(MODEM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_NUM, "\r\n", 2);
}

static void read_uart_for_ms(int timeout_ms)
{
    uint8_t data[UART_BUF_SIZE];
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        int len = uart_read_bytes(
            MODEM_UART_NUM,
            data,
            sizeof(data) - 1,
            pdMS_TO_TICKS(200)
        );

        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "RX: %s", (char *)data);
        }

        elapsed += 200;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SIM7600 simple UART test started");
    ESP_LOGI(TAG, "ESP GPIO%d TX -> SIM7600 RXD", MODEM_TX_GPIO);
    ESP_LOGI(TAG, "ESP GPIO%d RX <- SIM7600 TXD", MODEM_RX_GPIO);
    ESP_LOGI(TAG, "Baudrate: %d", MODEM_BAUDRATE);
    ESP_LOGI(TAG, "SIM7600 UART jumpers must be on B");

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

    vTaskDelay(pdMS_TO_TICKS(3000));

    modem_send("AT");
    read_uart_for_ms(2000);

    modem_send("ATI");
    read_uart_for_ms(3000);

    modem_send("AT+CPIN?");
    read_uart_for_ms(3000);

    modem_send("AT+CSQ");
    read_uart_for_ms(3000);

    modem_send("AT+CGPS=1");
    read_uart_for_ms(3000);

    while (1) {
        modem_send("AT+CGPSINFO");
        read_uart_for_ms(5000);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
