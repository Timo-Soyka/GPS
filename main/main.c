source ~/.espressif/tools/activate_idf_v6.0.sh
cd ~/esp32p4_projects/GPS

cat > main/main.c <<'EOF'
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "lvgl.h"
#include "bsp/esp-bsp.h"

#define MODEM_UART_NUM UART_NUM_1

// Funktionierende Verdrahtung:
// ESP GPIO7  TX -> SIM7600 RXD
// ESP GPIO8  RX <- SIM7600 TXD
// GND            -> GND
#define MODEM_TX_GPIO 7
#define MODEM_RX_GPIO 8

#define MODEM_BAUDRATE 115200
#define UART_BUF_SIZE 2048

#define MAX_TRACK_POINTS 200

#define MAP_W 900
#define MAP_H 340

static const char *TAG = "GPS_MAP";

typedef struct {
    double lat;
    double lon;
    bool valid;
} gps_point_t;

static gps_point_t current_pos = {0};
static gps_point_t track[MAX_TRACK_POINTS];
static int track_count = 0;

static SemaphoreHandle_t gps_mutex;

static lv_obj_t *label_status;
static lv_obj_t *label_lat;
static lv_obj_t *label_lon;
static lv_obj_t *label_points;
static lv_obj_t *label_info;
static lv_obj_t *map_box;
static lv_obj_t *track_line;
static lv_obj_t *position_dot;

static lv_point_precise_t line_points[MAX_TRACK_POINTS];

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
    if (value == NULL || value[0] == '\0') {
        return 0.0;
    }

    double raw = atof(value);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (hemi == 'S' || hemi == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

static bool parse_cgpsinfo(const char *response, gps_point_t *point)
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
    char lat_hemi = 0;
    char lon_raw[24] = {0};
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

    point->lat = nmea_to_decimal(lat_raw, lat_hemi);
    point->lon = nmea_to_decimal(lon_raw, lon_hemi);
    point->valid = true;

    return true;
}

static void add_track_point(gps_point_t p)
{
    if (!p.valid) {
        return;
    }

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_pos = p;

        if (track_count == 0 ||
            fabs(track[track_count - 1].lat - p.lat) > 0.000001 ||
            fabs(track[track_count - 1].lon - p.lon) > 0.000001) {

            if (track_count < MAX_TRACK_POINTS) {
                track[track_count++] = p;
            } else {
                memmove(&track[0], &track[1], sizeof(gps_point_t) * (MAX_TRACK_POINTS - 1));
                track[MAX_TRACK_POINTS - 1] = p;
            }
        }

        xSemaphoreGive(gps_mutex);
    }
}

static void update_track_view(void)
{
    gps_point_t local_track[MAX_TRACK_POINTS];
    int local_count = 0;

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        local_count = track_count;
        memcpy(local_track, track, sizeof(gps_point_t) * local_count);
        xSemaphoreGive(gps_mutex);
    }

    if (local_count <= 0) {
        lv_line_set_points(track_line, line_points, 0);
        lv_obj_add_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    double min_lat = local_track[0].lat;
    double max_lat = local_track[0].lat;
    double min_lon = local_track[0].lon;
    double max_lon = local_track[0].lon;

    for (int i = 1; i < local_count; i++) {
        if (local_track[i].lat < min_lat) min_lat = local_track[i].lat;
        if (local_track[i].lat > max_lat) max_lat = local_track[i].lat;
        if (local_track[i].lon < min_lon) min_lon = local_track[i].lon;
        if (local_track[i].lon > max_lon) max_lon = local_track[i].lon;
    }

    double lat_span = max_lat - min_lat;
    double lon_span = max_lon - min_lon;

    if (lat_span < 0.0001) lat_span = 0.0001;
    if (lon_span < 0.0001) lon_span = 0.0001;

    int margin = 20;

    for (int i = 0; i < local_count; i++) {
        double x_norm = (local_track[i].lon - min_lon) / lon_span;
        double y_norm = (local_track[i].lat - min_lat) / lat_span;

        line_points[i].x = margin + (int)(x_norm * (MAP_W - 2 * margin));
        line_points[i].y = MAP_H - margin - (int)(y_norm * (MAP_H - 2 * margin));
    }

    lv_line_set_points(track_line, line_points, local_count);

    lv_obj_clear_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(
        position_dot,
        line_points[local_count - 1].x - 6,
        line_points[local_count - 1].y - 6
    );
}

static void update_ui(void)
{
    gps_point_t p = {0};
    int count = 0;

    if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        p = current_pos;
        count = track_count;
        xSemaphoreGive(gps_mutex);
    }

    if (p.valid) {
        char lat_buf[64];
        char lon_buf[64];
        char points_buf[64];

        snprintf(lat_buf, sizeof(lat_buf), "Lat: %.6f", p.lat);
        snprintf(lon_buf, sizeof(lon_buf), "Lon: %.6f", p.lon);
        snprintf(points_buf, sizeof(points_buf), "Trackpunkte: %d", count);

        lv_label_set_text(label_status, "Fix: JA");
        lv_label_set_text(label_lat, lat_buf);
        lv_label_set_text(label_lon, lon_buf);
        lv_label_set_text(label_points, points_buf);
        lv_label_set_text(label_info, "Lokale Track-Ansicht");

        update_track_view();
    } else {
        lv_label_set_text(label_status, "Fix: NEIN");
        lv_label_set_text(label_lat, "Lat: -");
        lv_label_set_text(label_lon, "Lon: -");
        lv_label_set_text(label_points, "Trackpunkte: 0");
        lv_label_set_text(label_info, "Warte auf GPS-Fix...");

        update_track_view();
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

static void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0b0f14), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "ESP32-P4 GPS Tracker");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 15);

    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Fix: -");
    lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 35, 60);

    label_lat = lv_label_create(scr);
    lv_label_set_text(label_lat, "Lat: -");
    lv_obj_align(label_lat, LV_ALIGN_TOP_LEFT, 35, 95);

    label_lon = lv_label_create(scr);
    lv_label_set_text(label_lon, "Lon: -");
    lv_obj_align(label_lon, LV_ALIGN_TOP_LEFT, 35, 130);

    label_points = lv_label_create(scr);
    lv_label_set_text(label_points, "Trackpunkte: 0");
    lv_obj_align(label_points, LV_ALIGN_TOP_LEFT, 35, 165);

    label_info = lv_label_create(scr);
    lv_label_set_text(label_info, "Initialisiere...");
    lv_obj_align(label_info, LV_ALIGN_TOP_LEFT, 35, 200);

    map_box = lv_obj_create(scr);
    lv_obj_set_size(map_box, MAP_W, MAP_H);
    lv_obj_align(map_box, LV_ALIGN_BOTTOM_MID, 0, -25);
    lv_obj_set_style_bg_color(map_box, lv_color_hex(0x101820), 0);
    lv_obj_set_style_border_color(map_box, lv_color_hex(0x335566), 0);
    lv_obj_set_style_border_width(map_box, 2, 0);
    lv_obj_set_style_radius(map_box, 8, 0);
    lv_obj_clear_flag(map_box, LV_OBJ_FLAG_SCROLLABLE);

    track_line = lv_line_create(map_box);
    lv_obj_set_style_line_color(track_line, lv_color_hex(0x00ff88), 0);
    lv_obj_set_style_line_width(track_line, 3, 0);
    lv_line_set_points(track_line, line_points, 0);

    position_dot = lv_obj_create(map_box);
    lv_obj_set_size(position_dot, 12, 12);
    lv_obj_set_style_radius(position_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(position_dot, lv_color_hex(0xffcc00), 0);
    lv_obj_set_style_border_width(position_dot, 0, 0);
    lv_obj_add_flag(position_dot, LV_OBJ_FLAG_HIDDEN);
}

static void modem_task(void *arg)
{
    (void)arg;

    char response[UART_BUF_SIZE];

    modem_send("AT");
    read_uart(response, sizeof(response), 2000);

    modem_send("ATE0");
    read_uart(response, sizeof(response), 2000);

    modem_send("ATI");
    read_uart(response, sizeof(response), 3000);

    modem_send("AT+CGPS=1");
    read_uart(response, sizeof(response), 3000);

    while (1) {
        modem_send("AT+CGPSINFO");

        int len = read_uart(response, sizeof(response), 5000);

        if (len > 0) {
            gps_point_t p = {0};

            if (parse_cgpsinfo(response, &p)) {
                ESP_LOGI(TAG, "GPS FIX: lat=%.6f lon=%.6f", p.lat, p.lon);
                add_track_point(p);
            } else {
                ESP_LOGI(TAG, "No valid GPS fix yet");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
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

void app_main(void)
{
    ESP_LOGI(TAG, "Starting GPS tracker with local map view");
    ESP_LOGI(TAG, "ESP GPIO%d TX -> SIM7600 RXD", MODEM_TX_GPIO);
    ESP_LOGI(TAG, "ESP GPIO%d RX <- SIM7600 TXD", MODEM_RX_GPIO);

    gps_mutex = xSemaphoreCreateMutex();

    uart_init_modem();

    bsp_display_start();

    if (bsp_display_lock(0)) {
        create_ui();
        bsp_display_unlock();
    }

    lv_timer_create(ui_timer_cb, 1000, NULL);

    xTaskCreate(modem_task, "modem_task", 8192, NULL, 5, NULL);
}
EOF
