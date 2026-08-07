/**
 * @file signal_db.c
 * @brief Signal Database loader — cJSON + SPIFFS
 *
 * cJSON is bundled with ESP-IDF 5.x as component "json".
 * Add "json" to REQUIRES in CMakeLists.txt — no extra library needed.
 *
 * SPIFFS mount point: /spiffs
 * Upload command:     pio run --target uploadfs
 */

#include "signal_db.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SIGNAL_DB";

/* ── SPIFFS helpers ─────────────────────────────────────────────────────── */

#define SPIFFS_BASE_PATH  "/spiffs"
#define SPIFFS_MAX_FILES  8u
#define JSON_READ_BUF_MAX (32u * 1024u)   /* 32 KB — largest expected JSON */

static bool s_spiffs_mounted = false;

static esp_err_t _mount_spiffs(void)
{
    if (s_spiffs_mounted) return ESP_OK;

    esp_vfs_spiffs_conf_t conf = {
        .base_path              = SPIFFS_BASE_PATH,
        .partition_label        = NULL,   /* use default "spiffs" partition */
        .max_files              = SPIFFS_MAX_FILES,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE /* already mounted */) {
        s_spiffs_mounted = true;

        size_t total = 0, used = 0;
        esp_spiffs_info(NULL, &total, &used);
        ESP_LOGI(TAG, "SPIFFS mounted — %zu KB total, %zu KB used",
                 total / 1024, used / 1024);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
    return ret;
}

static char *_read_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open: %s", path);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    if (size <= 0 || size >= JSON_READ_BUF_MAX) {
        ESP_LOGE(TAG, "File size %ld out of range (max %u)", size, JSON_READ_BUF_MAX);
        fclose(f);
        return NULL;
    }

    char *buf = (char *)malloc((size_t)size + 1u);
    if (!buf) {
        ESP_LOGE(TAG, "OOM reading JSON (%ld bytes)", size);
        fclose(f);
        return NULL;
    }

    size_t read = fread(buf, 1, (size_t)size, f);
    fclose(f);
    buf[read] = '\0';

    ESP_LOGI(TAG, "Read %zu bytes from %s", read, path);
    return buf;
}

/* ── JSON field helpers ─────────────────────────────────────────────────── */

static const char *_str(const cJSON *obj, const char *key, const char *def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return (item && cJSON_IsString(item)) ? item->valuestring : def;
}

static double _num(const cJSON *obj, const char *key, double def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    return (item && cJSON_IsNumber(item)) ? item->valuedouble : def;
}

/**
 * Parse a hex string like "0x7E8" or a plain integer.
 * Returns 0 on failure.
 */
static uint32_t _parse_id(const cJSON *obj, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!item) return 0u;

    if (cJSON_IsNumber(item)) {
        return (uint32_t)item->valuedouble;
    }
    if (cJSON_IsString(item)) {
        return (uint32_t)strtoul(item->valuestring, NULL, 0);
    }
    return 0u;
}

static signal_byte_order_t _parse_byte_order(const cJSON *obj)
{
    const char *s = _str(obj, "byte_order", "big_endian");
    return (strcmp(s, "little_endian") == 0)
           ? BYTE_ORDER_LITTLE_ENDIAN
           : BYTE_ORDER_BIG_ENDIAN;
}

static signal_value_type_t _parse_value_type(const cJSON *obj)
{
    const char *s = _str(obj, "value_type", "unsigned");
    if (strcmp(s, "signed")  == 0) return VALUE_TYPE_SIGNED;
    if (strcmp(s, "string")  == 0) return VALUE_TYPE_STRING;
    return VALUE_TYPE_UNSIGNED;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void signal_db_init(signal_db_t *db)
{
    if (!db) return;
    memset(db, 0, sizeof(*db));
}

esp_err_t signal_db_load(signal_db_t *db, const char *json_path)
{
    if (!db || !json_path) return ESP_ERR_INVALID_ARG;

    /* Mount SPIFFS if needed */
    esp_err_t ret = _mount_spiffs();
    if (ret != ESP_OK) return ret;

    /* Read file into heap buffer */
    char *raw = _read_file(json_path);
    if (!raw) return ESP_ERR_NOT_FOUND;

    /* Parse JSON */
    cJSON *root = cJSON_Parse(raw);
    free(raw);

    if (!root) {
        const char *err = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "JSON parse error near: %s", err ? err : "unknown");
        return ESP_FAIL;
    }

    /* Top-level metadata */
    snprintf(db->vehicle, sizeof(db->vehicle), "%s",
             _str(root, "vehicle", "unknown"));
    snprintf(db->vin, sizeof(db->vin), "%s",
             _str(root, "vin", ""));
    db->version = (uint8_t)_num(root, "version", 1);

    /* Signals array */
    const cJSON *signals = cJSON_GetObjectItemCaseSensitive(root, "signals");
    if (!cJSON_IsArray(signals)) {
        ESP_LOGE(TAG, "JSON missing 'signals' array");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    uint16_t idx = 0;
    const cJSON *sig = NULL;

    cJSON_ArrayForEach(sig, signals) {
        if (idx >= SIGNAL_DB_MAX_SIGNALS) {
            ESP_LOGW(TAG, "Signal pool full at %u — remaining entries skipped",
                     SIGNAL_DB_MAX_SIGNALS);
            break;
        }

        signal_def_t *d = &db->signals[idx];

        /* Skip signals with frame_id == 0x000 (pending discovery) */
        uint32_t fid = _parse_id(sig, "frame_id");
        if (fid == 0u) {
            ESP_LOGW(TAG, "Skipping '%s' — frame_id=0x000 (pending Phase 8 discovery)",
                     _str(sig, "name", "?"));
            continue;
        }

        snprintf(d->name, sizeof(d->name), "%s", _str(sig, "name", ""));
        snprintf(d->desc, sizeof(d->desc), "%s", _str(sig, "description", ""));
        snprintf(d->unit, sizeof(d->unit), "%s", _str(sig, "unit", ""));
        snprintf(d->ecu,  sizeof(d->ecu),  "%s", _str(sig, "ecu", ""));

        d->frame_id   = fid;
        d->pid        = (uint16_t)_parse_id(sig, "pid");
        d->did        = (uint16_t)_parse_id(sig, "did");
        d->service    = (uint8_t) _parse_id(sig, "service");
        d->start_bit  = (uint8_t) _num(sig, "start_bit", 0);
        d->length     = (uint8_t) _num(sig, "length",    8);
        d->byte_order = _parse_byte_order(sig);
        d->value_type = _parse_value_type(sig);
        d->factor     = (float)   _num(sig, "factor",  1.0);
        d->offset     = (float)   _num(sig, "offset",  0.0);
        d->min        = (float)   _num(sig, "min",     0.0);
        d->max        = (float)   _num(sig, "max",     0.0);
        d->valid      = true;

        idx++;
    }

    db->count  = idx;
    db->loaded = true;

    cJSON_Delete(root);

    ESP_LOGI(TAG, "Loaded %u signals for %s (VIN: %s)",
             db->count, db->vehicle, db->vin);
    return ESP_OK;
}

const signal_def_t *signal_db_find_by_name(const signal_db_t *db,
                                             const char *name)
{
    if (!db || !name) return NULL;
    for (uint16_t i = 0; i < db->count; i++) {
        if (db->signals[i].valid &&
            strcmp(db->signals[i].name, name) == 0) {
            return &db->signals[i];
        }
    }
    return NULL;
}

uint8_t signal_db_find_by_frame_id(const signal_db_t *db,
                                    uint32_t frame_id,
                                    const signal_def_t **out_defs,
                                    uint8_t max_out)
{
    if (!db || !out_defs || max_out == 0) return 0;
    uint8_t found = 0;
    for (uint16_t i = 0; i < db->count && found < max_out; i++) {
        if (db->signals[i].valid && db->signals[i].frame_id == frame_id) {
            out_defs[found++] = &db->signals[i];
        }
    }
    return found;
}

const signal_def_t *signal_db_find_by_pid(const signal_db_t *db,
                                           uint16_t pid)
{
    if (!db || pid == 0) return NULL;
    for (uint16_t i = 0; i < db->count; i++) {
        if (db->signals[i].valid && db->signals[i].pid == pid) {
            return &db->signals[i];
        }
    }
    return NULL;
}

const signal_def_t *signal_db_find_by_did(const signal_db_t *db,
                                           uint16_t did)
{
    if (!db || did == 0) return NULL;
    for (uint16_t i = 0; i < db->count; i++) {
        if (db->signals[i].valid && db->signals[i].did == did) {
            return &db->signals[i];
        }
    }
    return NULL;
}

void signal_db_print(const signal_db_t *db)
{
    if (!db) return;
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  Signal DB: %s  VIN: %s  v%u  (%u signals)",
             db->vehicle, db->vin, db->version, db->count);
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  %-24s %-8s %-6s %-5s %-4s %-12s %s",
             "Name", "Frame", "PID", "Bits", "BO", "Factor+Off", "Unit");
    ESP_LOGI(TAG, "  ─────────────────────────────────────────────────────────");

    for (uint16_t i = 0; i < db->count; i++) {
        const signal_def_t *d = &db->signals[i];
        if (!d->valid) continue;

        char bo = (d->byte_order == BYTE_ORDER_BIG_ENDIAN) ? 'B' : 'L';
        char vt = (d->value_type == VALUE_TYPE_SIGNED) ? 'S' :
                  (d->value_type == VALUE_TYPE_STRING) ? 'A' : 'U';

        ESP_LOGI(TAG, "  %-24s 0x%03"PRIX32" %5s  @%2u+%2u  %c%c  *%.4f+%.2f  %s",
                 d->name, d->frame_id,
                 (d->pid ? "OBD" : "DID"),
                 d->start_bit, d->length,
                 bo, vt,
                 (double)d->factor, (double)d->offset,
                 d->unit);
    }
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════════");
}