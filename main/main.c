#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "MAIN"
#define DEVICE_NAME "Motocast"
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

// Maximum MTU size (ESP32 supports up to 517 bytes)
#define MAX_MTU_SIZE 517
#define PREFERRED_MTU 512

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

#define PREPARE_BUF_MAX_SIZE (1024)

static uint16_t gatts_if_id = 0;
static uint16_t conn_id = 0;
static uint16_t gatts_handle = 0;
static uint16_t current_mtu = 23; // Default MTU

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_config = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = false,
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
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

prepare_type_env_t prepare_write_env = {};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "gatts_event_handler %d %d %p", event, gatts_if, param);
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
            
            // Request MTU exchange to maximum size
            esp_ble_gatt_set_local_mtu(PREFERRED_MTU);
            
            // Optionally update connection parameters for better throughput
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x06;  // 7.5ms
            conn_params.max_int = 0x06;  // 7.5ms
            conn_params.latency = 0;
            conn_params.timeout = 400;   // 4 seconds
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, reason: 0x%x", param->disconnect.reason);
            current_mtu = 23; // Reset to default
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_MTU_EVT:
            current_mtu = param->mtu.mtu;
            ESP_LOGI(TAG, "MTU Exchange: %d bytes", current_mtu);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(TAG, "accepting %u bytes, no prep ", param->write.len);
                // ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
#if 0
                if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2*/){
                    uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                            ESP_LOGI(TAG, "Notification enable");
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            //the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                        }
                    }else if (descr_value == 0x0002){
                        if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(TAG, "Indication enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            //the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
                    }else{
                        ESP_LOGE(GATTS_TAG, "Unknown value");
                    }
                }
#endif
            }
            esp_gatt_status_t status = ESP_GATT_OK;
            ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT need_rsp: %d, is_prep: %d", param->write.need_rsp, param->write.is_prep);
            if (param->write.need_rsp){
                if (param->write.is_prep) {
                    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_OFFSET;
                    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                        status = ESP_GATT_INVALID_ATTR_LEN;
                    }
                    if (status == ESP_GATT_OK && prepare_write_env.prepare_buf == NULL) {
                        prepare_write_env.prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                        prepare_write_env.prepare_len = 0;
                        if (prepare_write_env.prepare_buf == NULL) {
                            ESP_LOGE(TAG, "Gatt_server prep no mem");
                            status = ESP_GATT_NO_RESOURCES;
                        }
                    }

                    // Security fix: Use calloc to ensure memory is zero-initialized
                    esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)calloc(1, sizeof(esp_gatt_rsp_t));
                    if (gatt_rsp) {
                        gatt_rsp->attr_value.len = param->write.len;
                        gatt_rsp->attr_value.handle = param->write.handle;
                        gatt_rsp->attr_value.offset = param->write.offset;
                        gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                        memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                        esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                        if (response_err != ESP_OK){
                            ESP_LOGE(TAG, "Send response error\n");
                        }
                        free(gatt_rsp);
                    } else {
                        ESP_LOGE(TAG, "malloc failed, no resource to send response error\n");
                        status = ESP_GATT_NO_RESOURCES;
                    }
                    if (status != ESP_GATT_OK){
                        ESP_LOGW(TAG, "error %d", status);
                        return;
                    }
                    memcpy(prepare_write_env.prepare_buf + param->write.offset,
                        param->write.value,
                        param->write.len);
                    prepare_write_env.prepare_len += param->write.len;

                }else{
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
                }
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
                // ESP_LOG_BUFFER_HEX(TAG, prepare_write_env.prepare_buf, prepare_write_env.prepare_len);
                ESP_LOGI(TAG, "accepting %u bytes of data", prepare_write_env.prepare_len);
            } else{
                ESP_LOGI(TAG,"ESP_GATT_PREP_WRITE_CANCEL");
            }
            if (prepare_write_env.prepare_buf) {
                free(prepare_write_env.prepare_buf);
                prepare_write_env.prepare_buf = NULL;
            }
            prepare_write_env.prepare_len = 0;

            break;
            
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "GAP event: %d", event);
    
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ADV_DATA_SET_COMPLETE, status: %d", param->adv_data_cmpl.status);
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                ESP_LOGI(TAG, "Starting advertising...");
                esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
                if (ret) {
                    ESP_LOGE(TAG, "Start advertising failed: %x", ret);
                }
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "SCAN_RSP_DATA_SET_COMPLETE, status: %d", param->scan_rsp_data_cmpl.status);
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                ESP_LOGI(TAG, "Starting advertising...");
                esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
                if (ret) {
                    ESP_LOGE(TAG, "Start advertising failed: %x", ret);
                }
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
            ESP_LOGI(TAG, "Connection params updated: status=%d, min_int=%d, max_int=%d, latency=%d, timeout=%d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
            
        default:
            break;
    }
}
extern void waveshare_init(void);

void app_main(void) {
    esp_err_t ret;

    waveshare_init();
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize Bluetooth controller
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
    
    // Initialize Bluedroid stack
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
    
    // Set maximum MTU
    esp_ble_gatt_set_local_mtu(PREFERRED_MTU);
    
    // Register callbacks
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    
    // Register application
    esp_ble_gatts_app_register(0);
    
    ESP_LOGI(TAG, "BLE Maximum MTU receiver initialized");
    ESP_LOGI(TAG, "Device name: %s", DEVICE_NAME);
    ESP_LOGI(TAG, "Maximum MTU: %d bytes", MAX_MTU_SIZE);
}
