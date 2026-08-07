/* ============================================================
 *  gps_app.c  —  NEO-6M @ 10 Hz, NMEA parser, JSON emitter
 * ============================================================ */
#include "gps_app.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "sd_logger.h"

static const char *TAG = "GPS";

#define GPS_UART UART_NUM_2
#define GPS_RX_PIN 16 /* NEO-6M TX → ESP32 RX */
#define GPS_TX_PIN 17 /* NEO-6M RX ← ESP32 TX */
#define GPS_BAUD 9600
#define GPS_BUF 1024

/* ── UBX CFG-RATE: 100 ms measurement period = 10 Hz ─────────
 * Packet: header(2) + class(1) + id(1) + len(2) + payload(6) + ck(2)
 * Payload: measRate=0x0064 (100ms), navRate=0x0001, timeRef=0x0000
 * Checksum computed over class+id+len+payload.                */
static const uint8_t UBX_10HZ[] = {
    0xB5, 0x62, /* UBX sync chars              */
    0x06, 0x08, /* class=CFG, id=RATE          */
    0x06, 0x00, /* payload length = 6          */
    0x64, 0x00, /* measRate = 100 ms (10 Hz)   */
    0x01, 0x00, /* navRate  = 1 cycle          */
    0x00, 0x00, /* timeRef  = UTC              */
    0x79, 0x10  /* CK_A, CK_B                  */
};

static gps_fix_t s_fix = {0};
static SemaphoreHandle_t s_lock = NULL;

/* ── NMEA helpers ─────────────────────────────────────────── */
static double _nmea_coord(const char *raw, const char *dir)
{
    if (!raw || !raw[0])
        return 0.0;
    double v = atof(raw);
    int deg = (int)(v / 100);
    double min = v - (double)(deg * 100);
    double res = deg + (min / 60.0);
    if (dir[0] == 'S' || dir[0] == 'W')
        res = -res;
    return res;
}

/* $GPRMC / $GNRMC ─────────────────────────────────────────── */
static void _parse_rmc(char *f[], int n)
{
    /* f[0]=GPRMC f[1]=time f[2]=status f[3]=lat f[4]=N/S
       f[5]=lon  f[6]=E/W  f[7]=speed  f[8]=course  */
    if (n < 9 || f[2][0] != 'A')
        return; /* A=valid */

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_fix.lat = _nmea_coord(f[3], f[4]);
    s_fix.lon = _nmea_coord(f[5], f[6]);
    s_fix.speed_kmh = (float)(atof(f[7]) * 1.852);
    s_fix.heading_deg = (float)atof(f[8]);
    s_fix.valid = true;
    s_fix.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_fix.fix_count++;
    xSemaphoreGive(s_lock);
}

/* $GPGGA / $GNGGA ─────────────────────────────────────────── */
static void _parse_gga(char *f[], int n)
{
    /* f[0]=GPGGA f[1]=time f[2]=lat f[3]=N/S f[4]=lon f[5]=E/W
       f[6]=quality f[7]=sats f[8]=hdop f[9]=alt */
    if (n < 10)
        return;
    int q = atoi(f[6]);
    if (q == 0)
        return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_fix.fix_quality = (uint8_t)q;
    s_fix.satellites = (uint8_t)atoi(f[7]);
    s_fix.hdop = (float)atof(f[8]);
    s_fix.altitude_m = (float)atof(f[9]);
    if (!s_fix.valid)
    {
        s_fix.lat = _nmea_coord(f[2], f[3]);
        s_fix.lon = _nmea_coord(f[4], f[5]);
        s_fix.valid = true;
        s_fix.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }
    xSemaphoreGive(s_lock);
}

/* ── Emit JSON to serial ──────────────────────────────────── */
static void _emit(void)
{
    gps_fix_t f = gps_get_fix();
    if (!f.valid)
        return;

    /* 1. Print to serial */
    printf("{\"t\":%lu,\"type\":\"gps\","
           "\"lat\":%.7f,\"lon\":%.7f,"
           "\"spd\":%.1f,\"hdg\":%.1f,"
           "\"alt\":%.1f,\"sat\":%d,"
           "\"hdop\":%.1f,\"fix\":%lu}\n",
           (unsigned long)f.timestamp_ms,
           f.lat, f.lon,
           f.speed_kmh, f.heading_deg,
           f.altitude_m, f.satellites,
           f.hdop, (unsigned long)f.fix_count);
    fflush(stdout);

    /* 2. Feed to SD logger (converts gps_fix_t → gps_snapshot_t) */
    gps_snapshot_t snap = {
        .timestamp_ms = f.timestamp_ms,
        .lat = f.lat,
        .lon = f.lon,
        .speed_kmh = f.speed_kmh,
        .heading_deg = f.heading_deg,
        .altitude_m = f.altitude_m,
        .satellites = f.satellites,
        .hdop = f.hdop,
        .valid = f.valid,
    };
    sd_logger_log_gps(&snap);
}

/* ── Sentence processor ───────────────────────────────────── */
static void _process(char *sent)
{
    char *star = strrchr(sent, '*');
    if (!star)
        return;
    *star = '\0';
    char *body = sent + 1;

    /* Verify NMEA checksum */
    uint8_t ck = 0;
    for (char *p = body; *p; p++)
        ck ^= (uint8_t)*p;
    if (ck != (uint8_t)strtol(star + 1, NULL, 16))
        return;

    /* Split fields */
    char *flds[25];
    int nf = 0;
    char *tok = strtok(body, ",");
    while (tok && nf < 24)
    {
        flds[nf++] = tok;
        tok = strtok(NULL, ",");
    }
    if (nf < 1)
        return;

    /* Strip talker prefix (GP/GN/GL) — compare last 3 chars of sentence ID */
    const char *sid = flds[0];
    size_t slen = strlen(sid);
    const char *type = slen >= 3 ? sid + slen - 3 : sid;

    if (strcmp(type, "RMC") == 0)
    {
        _parse_rmc(flds, nf);
        _emit();
    }
    else if (strcmp(type, "GGA") == 0)
        _parse_gga(flds, nf);
}

/* ── GPS reader task (Core 1) ─────────────────────────────── */
static void _gps_task(void *arg)
{
    uint8_t raw[GPS_BUF];
    char line[128];
    int lp = 0;

    ESP_LOGI(TAG, "GPS task running on Core %d", xPortGetCoreID());

    for (;;)
    {
        int len = uart_read_bytes(GPS_UART, raw, sizeof(raw) - 1,
                                  pdMS_TO_TICKS(50));
        for (int i = 0; i < len; i++)
        {
            char c = (char)raw[i];
            if (c == '$')
                lp = 0;
            if (lp < (int)sizeof(line) - 1)
                line[lp++] = c;
            if (c == '\n' && lp > 6)
            {
                line[lp] = '\0';
                _process(line);
                lp = 0;
            }
        }
        /* At 10Hz one NMEA cycle is ~100ms. 50ms read timeout
         * ensures we drain the UART buffer twice per cycle.   */
    }
}

/* ── Public API ───────────────────────────────────────────── */
esp_err_t gps_app_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock)
        return ESP_ERR_NO_MEM;

    /* Configure UART2 */
    uart_config_t cfg = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART,
                                        GPS_BUF * 2, 0, 0, NULL, 0));

    /* Send UBX CFG-RATE for 10 Hz. Send twice — NEO-6M sometimes
     * misses the first packet while its own UART is initialising. */
    vTaskDelay(pdMS_TO_TICKS(500));
    uart_write_bytes(GPS_UART, (char *)UBX_10HZ, sizeof(UBX_10HZ));
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(GPS_UART, (char *)UBX_10HZ, sizeof(UBX_10HZ));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "NEO-6M configured: 10 Hz (RX=GPIO16 TX=GPIO17)");

    /* Parser task: Core 1, priority 4 (above broadcasts, below CAN) */
    xTaskCreatePinnedToCore(_gps_task, "gps", 4096, NULL, 4, NULL, 1);
    return ESP_OK;
}

gps_fix_t gps_get_fix(void)
{
    gps_fix_t copy;
    if (s_lock)
        xSemaphoreTake(s_lock, portMAX_DELAY);
    copy = s_fix;
    if (s_lock)
        xSemaphoreGive(s_lock);
    return copy;
}

bool gps_is_fresh(void)
{
    gps_fix_t f = gps_get_fix();
    if (!f.valid)
        return false;
    return ((uint32_t)(esp_timer_get_time() / 1000) - f.timestamp_ms) < 3000;
}
