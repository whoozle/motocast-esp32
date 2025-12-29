#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#define TAG "MAIN"
#define DEVICE_NAME "Motocast"
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

// Maximum MTU size (ESP32 supports up to 517 bytes)
#define MAX_MTU_SIZE 517
#define PREFERRED_MTU 517  // Request maximum MTU

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

#define PREPARE_BUF_MAX_SIZE (4096)  // Increased buffer size

static uint16_t gatts_if_id = 0;
static uint16_t conn_id = 0;
static uint16_t gatts_handle = 0;
static uint16_t current_mtu = 23;

// Throughput monitoring
static uint32_t bytes_received = 0;
static int64_t last_report_time = 0;

// Optimized advertising parameters for faster connection
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,        // 20ms
    .adv_int_max = 0x40,        // 40ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_config = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0006,  // Set both to same value for consistent interval
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0006,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint16_t char_uuid = GATTS_CHAR_UUID;
static const uint8_t char_value[1] = {0x00};

// Attribute table
static uint16_t service_uuid = GATTS_SERVICE_UUID;

static const esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE] = {
    // Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, 
        ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(service_uuid), 
        (uint8_t *)&service_uuid}
    },
    
    // Characteristic Declaration
    [1] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, 
        ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), 
        (uint8_t *)&char_prop_write}
    },
    
    // Characteristic Value
    [2] = {{ESP_GATT_AUTO_RSP}, {
        ESP_UUID_LEN_16, (uint8_t *)&char_uuid, 
        ESP_GATT_PERM_WRITE, MAX_MTU_SIZE, sizeof(char_value), 
        (uint8_t *)char_value}
    },
};

typedef struct {
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

prepare_type_env_t prepare_write_env = {};

static void report_throughput(void) {
    int64_t now = esp_timer_get_time();
    if (last_report_time == 0) {
        last_report_time = now;
        return;
    }
    
    int64_t elapsed_us = now - last_report_time;
    if (elapsed_us >= 1000000) {  // Report every second
        float throughput_kbps = (bytes_received * 8.0f) / (elapsed_us / 1000.0f);
        ESP_LOGI(TAG, "Throughput: %.2f kbps (%.2f KB/s), MTU: %d", 
                 throughput_kbps, bytes_received / 1024.0f, current_mtu);
        bytes_received = 0;
        last_report_time = now;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATTS Register, app_id: %d, status: %d", 
                     param->reg.app_id, param->reg.status);
            gatts_if_id = gatts_if;
            
            // Set device name
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(TAG, "Set device name failed, error code = %x", set_dev_name_ret);
            }
            
            // Configure advertising data
            esp_err_t adv_ret = esp_ble_gap_config_adv_data(&adv_config);
            if (adv_ret) {
                ESP_LOGE(TAG, "Config adv data failed, error code = %x", adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            
            // Configure scan response data
            esp_err_t scan_ret = esp_ble_gap_config_adv_data(&scan_rsp_config);
            if (scan_ret) {
                ESP_LOGE(TAG, "Config scan response data failed, error code = %x", scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            
            // Create attribute table
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE, 0);
            break;
            
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Attribute table created, handles: %d", param->add_attr_tab.num_handle);
                gatts_handle = param->add_attr_tab.handles[2]; // Characteristic value handle
                esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
            } else {
                ESP_LOGE(TAG, "Create attribute table failed, error: 0x%x", param->add_attr_tab.status);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            bytes_received = 0;
            last_report_time = 0;
            
            // Request maximum MTU immediately
            esp_ble_gatt_set_local_mtu(PREFERRED_MTU);
            
            // Request aggressive connection parameters for maximum throughput
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            // Connection interval: 7.5ms (minimum allowed by BLE spec for data transfer)
            conn_params.min_int = 0x06;  // 6 * 1.25ms = 7.5ms
            conn_params.max_int = 0x06;  // Keep it fixed for consistent performance
            conn_params.latency = 0;     // No slave latency - respond to every event
            conn_params.timeout = 400;   // 4 seconds supervision timeout
            
            esp_ble_gap_update_conn_params(&conn_params);
            
            ESP_LOGI(TAG, "Requesting connection interval: 7.5ms, latency: 0");
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, reason: 0x%x", param->disconnect.reason);
            current_mtu = 23;
            bytes_received = 0;
            last_report_time = 0;
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_MTU_EVT:
            current_mtu = param->mtu.mtu;
            ESP_LOGI(TAG, "MTU Exchange complete: %d bytes (payload: %d bytes)", 
                     current_mtu, current_mtu - 3);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // Regular write - count bytes for throughput
                bytes_received += param->write.len;
                report_throughput();
            }
            
            esp_gatt_status_t status = ESP_GATT_OK;
            
            if (param->write.need_rsp) {
                if (param->write.is_prep) {
                    // Prepare write handling
                    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_OFFSET;
                    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_ATTR_LEN;
                    }
                    
                    if (status == ESP_GATT_OK && prepare_write_env.prepare_buf == NULL) {
                        prepare_write_env.prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE);
                        prepare_write_env.prepare_len = 0;
                        if (prepare_write_env.prepare_buf == NULL) {
                            ESP_LOGE(TAG, "Prep buffer allocation failed");
                            status = ESP_GATT_NO_RESOURCES;
                        }
                    }
                    
                    // Send response quickly to minimize round-trip time
                    esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)calloc(1, sizeof(esp_gatt_rsp_t));
                    if (gatt_rsp) {
                        gatt_rsp->attr_value.len = param->write.len;
                        gatt_rsp->attr_value.handle = param->write.handle;
                        gatt_rsp->attr_value.offset = param->write.offset;
                        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                        
                        esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, 
                            param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                        
                        if (response_err != ESP_OK) {
                            ESP_LOGE(TAG, "Send response error: %x", response_err);
                        }
                        free(gatt_rsp);
                    } else {
                        ESP_LOGE(TAG, "Response allocation failed");
                        status = ESP_GATT_NO_RESOURCES;
                    }
                    
                    if (status != ESP_GATT_OK) {
                        return;
                    }
                    
                    // Copy data to prepare buffer
                    memcpy(prepare_write_env.prepare_buf + param->write.offset,
                           param->write.value, param->write.len);
                    prepare_write_env.prepare_len += param->write.len;
                    
                } else {
                    // Regular write with response
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                                param->write.trans_id, status, NULL);
                }
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
                ESP_LOGI(TAG, "Long write complete: %u bytes", prepare_write_env.prepare_len);
                bytes_received += prepare_write_env.prepare_len;
                report_throughput();
            } else {
                ESP_LOGI(TAG, "Long write cancelled");
            }
            
            if (prepare_write_env.prepare_buf) {
                free(prepare_write_env.prepare_buf);
                prepare_write_env.prepare_buf = NULL;
            }
            prepare_write_env.prepare_len = 0;
            
            // Send response
            esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, 
                                       param->exec_write.trans_id, ESP_GATT_OK, NULL);
            break;
            
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                ESP_LOGI(TAG, "Starting advertising...");
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                ESP_LOGI(TAG, "Starting advertising...");
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection params updated: status=%d, interval=%.2fms, latency=%d, timeout=%dms",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int * 1.25,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout * 10);
            break;
            
        default:
            break;
    }
}

extern void waveshare_init(void);

void app_main(void) {
    esp_err_t ret;

    waveshare_init();
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize Bluetooth controller with default settings
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Set maximum MTU early
    esp_ble_gatt_set_local_mtu(PREFERRED_MTU);
    
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    
    esp_ble_gatts_app_register(0);
    
    ESP_LOGI(TAG, "BLE High-Throughput Receiver initialized");
    ESP_LOGI(TAG, "Device: %s | Max MTU: %d | Payload: %d bytes", 
             DEVICE_NAME, PREFERRED_MTU, PREFERRED_MTU - 3);
    ESP_LOGI(TAG, "Target connection interval: 7.5ms");
}