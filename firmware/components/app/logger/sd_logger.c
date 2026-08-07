/* ============================================================
 *  sd_logger.c  —  Minimal CAN-to-SD logger  (Step 14)
 *
 *  SPI host assignment (DO NOT CHANGE):
 *    MCP2515 → SPI2_HOST (HSPI)  GPIO 23/19/18  CS=5
 *    SD card → SPI3_HOST (VSPI)  GPIO 13/27/14  CS=26
 *  These are completely independent buses — no sharing.
 *
 *  This version writes two CSV files per session:
 *    raw.csv     – every raw CAN frame
 *    decoded.csv – decoded signal snapshot at ~2 Hz (dashboard rate)
 *
 *  GPS, dead-reckoning, UDS, and serial streaming are stubbed out.
 *  They compile and link correctly but do nothing. Add them back
 *  one feature at a time once raw + decoded logging is verified.
 *
 *  Flush strategy: fflush() + fsync() every FLUSH_MS.
 *  fsync() forces the FAT layer to commit to the physical
 *  card. Without it, files appear empty after a power cut even
 *  though fflush() returned success.
 *
 *  Session counter is also fsync'd before files are opened so that
 *  a power cut between reboots cannot overwrite a previous session.
 * ============================================================ */

#include "sd_logger.h"
#include "esp_log.h"
#include "esp_timer.h"
/* fsync() is implemented in ESP-IDF's newlib but the declaration in
 * <unistd.h> is gated behind _POSIX_C_SOURCE which ESP-IDF's CMake
 * does not set by default. Forward-declare it here — the linker will
 * find the implementation in libc regardless of the header situation. */
extern int fsync(int fd);
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>

static const char *TAG = "SD_LOG";

/* ── SD SPI pins — SPI3_HOST (VSPI) ────────────────────────────
 * GPIO12 and GPIO15 are strapping pins — never use for SD.
 * GPIO12 HIGH at boot → 1.8V flash → instant crash.
 * GPIO15 affects boot logging / JTAG.                          */
#define SD_MOSI 13
#define SD_MISO 27 /* add 10K pull-up to 3.3V on the wire   */
#define SD_CLK 14
#define SD_CS 26
#define SD_MOUNT "/sdcard"

/* ── Tuning ─────────────────────────────────────────────────── */
#define QUEUE_DEPTH 512 /* entries in the writer queue        */
#define BATCH_SIZE 32   /* frames processed per task wake     */
#define FLUSH_MS 250    /* fflush + fsync interval in ms      */

/* ── Queue entry ────────────────────────────────────────────── */
typedef enum
{
    LT_RAW = 1,
    LT_DEC
} log_type_t;

typedef struct
{
    log_type_t type;
    union
    {
        struct
        {
            uint32_t ts_ms;
            can_frame_t frame;
            uint8_t flags;
        } raw;
        decoded_snapshot_t dec;
    };
} log_entry_t;

/* ── Module state ────────────────────────────────────────────── */
static QueueHandle_t s_q = NULL;
static FILE *s_raw_fp = NULL;
static FILE *s_dec_fp = NULL;
static bool s_sd_ok = false;
static uint32_t s_sess = 0;
static uint32_t s_wr_raw = 0;
static uint32_t s_wr_dec = 0;
static uint32_t s_dropped = 0;

static inline uint32_t _ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ── SD mount — SPI3_HOST (VSPI) ────────────────────────────────
 * MCP2515 already owns SPI2_HOST. SPI3_HOST is free.
 * Clock kept at 400 kHz — safe for Catalex v1.0 at any wire length.
 * format_if_mount_failed = true auto-repairs a corrupted FAT from
 * previous power yanks.                                         */
static esp_err_t _mount_sd(void)
{
    ESP_LOGI(TAG, "SD: MOSI=%d MISO=%d CLK=%d CS=%d  (SPI3/VSPI)",
             SD_MOSI, SD_MISO, SD_CLK, SD_CS);

    spi_bus_config_t bus = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t r = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "SPI3 bus init failed: %s", esp_err_to_name(r));
        return r;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    host.max_freq_khz = 400;

    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.gpio_cs = SD_CS;
    dev.host_id = SPI3_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mnt = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16384,
    };

    sdmmc_card_t *card = NULL;
    r = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &dev, &mnt, &card);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "Mount failed: %s (0x%x)", esp_err_to_name(r), r);
        if (r == ESP_FAIL)
            ESP_LOGE(TAG, "  FAT error — format FAT32 on PC");
        else if (r == ESP_ERR_NO_MEM)
            ESP_LOGE(TAG, "  Low heap");
        else if (r == 0x107)
            ESP_LOGE(TAG, "  0x107: no SPI response — check MISO pull-up and card seated");
        else if (r == 0x108)
            ESP_LOGE(TAG, "  0x108: invalid response — check VCC=3.3V");
        return r;
    }

    /* Quick write test — catches write-protected or read-only cards */
    FILE *tf = fopen(SD_MOUNT "/_test.tmp", "w");
    if (!tf)
    {
        ESP_LOGE(TAG, "Write test failed (errno=%d) — check lock switch", errno);
        return ESP_FAIL;
    }
    fprintf(tf, "sd_logger write test\n");
    fclose(tf);
    remove(SD_MOUNT "/_test.tmp");

    double gb = (double)((uint64_t)card->csd.capacity *
                         card->csd.sector_size) /
                1073741824.0;
    ESP_LOGI(TAG, "✓ %s  %.2f GB", card->cid.name, gb);
    return ESP_OK;
}

/* ── Open session files ─────────────────────────────────────── */
static esp_err_t _open_files(void)
{
    mkdir(SD_MOUNT "/can", 0755);

    /* Read session counter from card */
    FILE *sf = fopen(SD_MOUNT "/can/.sess", "r");
    if (sf)
    {
        fscanf(sf, "%lu", (unsigned long *)&s_sess);
        fclose(sf);
    }
    s_sess++;

    /* Commit counter to card BEFORE creating session directory.
     * Without fsync() here, a power cut between sessions
     * leaves the counter un-incremented, and the next boot
     * re-uses the same session directory, truncating the old files. */
    sf = fopen(SD_MOUNT "/can/.sess", "w");
    if (sf)
    {
        fprintf(sf, "%lu\n", (unsigned long)s_sess);
        fflush(sf);
        fsync(fileno(sf));
        fclose(sf);
    }

    /* Create session directory */
    char dir[72];
    snprintf(dir, sizeof(dir), SD_MOUNT "/can/s%05lu", (unsigned long)s_sess);
    mkdir(dir, 0755);
    ESP_LOGI(TAG, "Session dir: %s", dir);

    /* raw.csv */
    char path[88];
    snprintf(path, sizeof(path), "%s/raw.csv", dir);
    s_raw_fp = fopen(path, "w");
    if (!s_raw_fp)
    {
        ESP_LOGE(TAG, "Cannot open %s (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    fprintf(s_raw_fp, "timestamp_ms,id_hex,dlc,b0,b1,b2,b3,b4,b5,b6,b7\n");
    fflush(s_raw_fp);
    ESP_LOGI(TAG, "raw.csv open OK");

    /* decoded.csv */
    snprintf(path, sizeof(path), "%s/decoded.csv", dir);
    s_dec_fp = fopen(path, "w");
    if (!s_dec_fp)
    {
        ESP_LOGE(TAG, "Cannot open %s (errno=%d)", path, errno);
        return ESP_FAIL;
    }
    fprintf(s_dec_fp, "timestamp_ms,rpm,steer_deg_x10,wfl,wfr,wrl,wrr,brake_bar,valid_mask\n");
    fflush(s_dec_fp);
    ESP_LOGI(TAG, "decoded.csv open OK");

    ESP_LOGI(TAG, "✓ Session %lu ready — both files open", (unsigned long)s_sess);
    return ESP_OK;
}

/* ── SD writer task — Core 1, priority 2 ────────────────────── */
static void _writer_task(void *pv)
{
    log_entry_t e;
    uint32_t last_flush = 0;

    for (;;)
    {
        int n = 0;
        while (n < BATCH_SIZE &&
               xQueueReceive(s_q, &e, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            n++;
            if (!s_sd_ok)
                continue;

            switch (e.type)
            {

            case LT_RAW:
                if (s_raw_fp)
                {
                    uint8_t *d = e.raw.frame.data;
                    fprintf(s_raw_fp,
                            "%lu,0x%03lX,%d,"
                            "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
                            (unsigned long)e.raw.ts_ms,
                            (unsigned long)e.raw.frame.id,
                            e.raw.frame.dlc,
                            d[0], d[1], d[2], d[3],
                            d[4], d[5], d[6], d[7]);
                    s_wr_raw++;
                }
                break;

            case LT_DEC:
                if (s_dec_fp)
                {
                    fprintf(s_dec_fp,
                            "%lu,%d,%d,%d,%d,%d,%d,%d,%d\n",
                            (unsigned long)e.dec.timestamp_ms,
                            e.dec.rpm,
                            e.dec.steer_deg_x10,
                            e.dec.wheel_fl, e.dec.wheel_fr,
                            e.dec.wheel_rl, e.dec.wheel_rr,
                            e.dec.brake_bar,
                            e.dec.valid_mask);
                    s_wr_dec++;
                }
                break;
            }
        }

        /* Flush every FLUSH_MS.
         * fflush pushes the app-level stdio buffer to the FAT layer.
         * fsync pushes the FAT layer's buffer to the SD card.
         * Both are needed — fflush alone leaves data in RAM on power cut. */
        uint32_t now = _ms();
        if (s_sd_ok && (now - last_flush) >= FLUSH_MS)
        {
            if (s_raw_fp)
            {
                fflush(s_raw_fp);
                fsync(fileno(s_raw_fp));
            }
            if (s_dec_fp)
            {
                fflush(s_dec_fp);
                fsync(fileno(s_dec_fp));
            }
            last_flush = now;
        }
    }
}

/* ============================================================
 *  Public API — active functions
 * ============================================================ */

esp_err_t sd_logger_init(void)
{
    ESP_LOGI(TAG, "=== SD Logger Init ===");

    s_q = xQueueCreate(QUEUE_DEPTH, sizeof(log_entry_t));
    if (!s_q)
    {
        ESP_LOGE(TAG, "Queue alloc failed — heap too small?");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Queue depth=%d  entry_size=%d B  heap_free=%lu B",
             QUEUE_DEPTH, (int)sizeof(log_entry_t),
             (unsigned long)esp_get_free_heap_size());

    esp_err_t r = _mount_sd();
    if (r == ESP_OK)
    {
        r = _open_files();
        s_sd_ok = (r == ESP_OK);
    }

    if (s_sd_ok)
    {
        ESP_LOGI(TAG, "✓ SD logging ACTIVE");
    }
    else
    {
        ESP_LOGW(TAG, "✗ SD unavailable — serial-only mode");
        ESP_LOGW(TAG, "  Check: card seated, MISO pull-up, VCC=3.3V, FAT32 format");
    }

    xTaskCreatePinnedToCore(_writer_task, "sd_wr", 4096,
                            NULL, 2, NULL, 1);
    ESP_LOGI(TAG, "Logger running  SD=%s  flush=%dms",
             s_sd_ok ? "YES" : "NO", FLUSH_MS);
    return ESP_OK; /* non-fatal: serial-only mode is acceptable */
}

void sd_logger_log_raw(const can_frame_t *f, uint8_t flags)
{
    if (!s_q)
        return;
    log_entry_t e;
    e.type = LT_RAW;
    e.raw.ts_ms = _ms();
    e.raw.frame = *f;
    e.raw.flags = flags;
    if (xQueueSend(s_q, &e, 0) != pdTRUE)
        s_dropped++;
}

void sd_logger_log_decoded(const decoded_snapshot_t *s)
{
    if (!s_q)
        return;
    log_entry_t e;
    e.type = LT_DEC;
    e.dec = *s;
    if (xQueueSend(s_q, &e, 0) != pdTRUE)
        s_dropped++;
}

sd_logger_stats_t sd_logger_get_stats(void)
{
    return (sd_logger_stats_t){
        .queue_used = s_q ? (uint16_t)uxQueueMessagesWaiting(s_q) : 0,
        .queue_cap = QUEUE_DEPTH,
        .dropped = s_dropped,
        .written_raw = s_wr_raw,
        .written_decoded = s_wr_dec,
        .written_gps = 0,
        .written_dr = 0,
        .written_uds = 0,
        .session_num = s_sess,
        .sd_ready = s_sd_ok,
    };
}

void sd_logger_deinit(void)
{
    ESP_LOGI(TAG, "=== SD Logger Shutdown ===");
    s_sd_ok = false;

    /* Final flush before closing */
    if (s_raw_fp)
    {
        fflush(s_raw_fp);
        fsync(fileno(s_raw_fp));
        fclose(s_raw_fp);
        s_raw_fp = NULL;
    }
    if (s_dec_fp)
    {
        fflush(s_dec_fp);
        fsync(fileno(s_dec_fp));
        fclose(s_dec_fp);
        s_dec_fp = NULL;
    }

    esp_vfs_fat_sdcard_unmount(SD_MOUNT, NULL);
    ESP_LOGI(TAG, "✓ SD card safely unmounted");
}

/* ============================================================
 *  Stub functions — compile and link, do nothing.
 *  GPS, DR, UDS, and serial streaming will be added back
 *  once raw + decoded logging is verified working on the car.
 * ============================================================ */

void sd_logger_log_gps(const gps_snapshot_t *g) { (void)g; }
void sd_logger_log_dr(const dr_snapshot_t *dr) { (void)dr; }
void sd_logger_log_uds(uint16_t did, const uint8_t *data,
                       size_t len, uint32_t elapsed_ms, bool corrupt)
{
    (void)did;
    (void)data;
    (void)len;
    (void)elapsed_ms;
    (void)corrupt;
}
void sd_logger_serial_emit_decoded(const decoded_snapshot_t *s) { (void)s; }
void sd_logger_serial_emit_gps(const gps_snapshot_t *g) { (void)g; }
void sd_logger_serial_emit_dr(const dr_snapshot_t *dr) { (void)dr; }
void sd_logger_serial_emit_uds(uint16_t did, const uint8_t *data,
                               size_t len, uint32_t elapsed_ms, bool corrupt)
{
    (void)did;
    (void)data;
    (void)len;
    (void)elapsed_ms;
    (void)corrupt;
}
void sd_logger_serial_emit_stats(void) {}