#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "check/esp_utils_check.h"

void waveshare_init(void);

#define MOTOCAST_APP_ID (0)
#define MOTOCAST_NUM_APPS (1)

static const char *TAG = "main";

esp_bt_uuid_t parse_uuid(const char *value) {
    esp_bt_uuid_t uuid = {};
    size_t length = strlen(value);
    if (length == 4) {
		uuid.len         = ESP_UUID_LEN_16;
		uuid.uuid.uuid16 = 0;
		for(int i=0;i < length;){
			uint8_t MSB = value[i];
			uint8_t LSB = value[i+1];

			if(MSB > '9') MSB -= 7;
			if(LSB > '9') LSB -= 7;
			uuid.uuid.uuid16 += (((MSB&0x0F) <<4) | (LSB & 0x0F))<<(2-i)*4;
			i+=2;
		}
	} else if (length == 8) {
		uuid.len         = ESP_UUID_LEN_32;
		uuid.uuid.uuid32 = 0;
		for(int i=0;i<length;){
			uint8_t MSB = value[i];
			uint8_t LSB = value[i+1];

			if(MSB > '9') MSB -= 7; 
			if(LSB > '9') LSB -= 7;
			uuid.uuid.uuid32 += (((MSB&0x0F) <<4) | (LSB & 0x0F))<<(6-i)*4;
			i+=2;
		}
	} else if (length == 36) {
		// If the length of the string is 36 bytes then we will assume it is a long hex string in
		// UUID format.
		uuid.len = ESP_UUID_LEN_128;
		int n = 0;
		for(int i=0;i<length;){
			if(value[i] == '-')
				i++;
			uint8_t MSB = value[i];
			uint8_t LSB = value[i+1];

			if(MSB > '9') MSB -= 7;
			if(LSB > '9') LSB -= 7;
			uuid.uuid.uuid128[15-n++] = ((MSB&0x0F) <<4) | (LSB & 0x0F);
			i+=2;
		}
	} else {
		ESP_LOGE(TAG, "uuid length is not 2, 4 or 36 bytes");
	}
    return uuid;
}

static void gatts_motocast_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst gl_profile_tab[MOTOCAST_NUM_APPS] = {
    [MOTOCAST_APP_ID] = {
        .gatts_cb = gatts_motocast_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "ble_gap_callback %d %p", event, param);
    switch(event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

static void gatts_motocast_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "gatts_motocast_event_handler %d %d %p", event, gatts_if, param);
    struct gatts_profile_inst * profile = &gl_profile_tab[MOTOCAST_APP_ID];
    switch(event) {
        case ESP_GATTS_CREATE_EVT:
            {
                static uint8_t char_value[] = {'Y', 'E', 'S'};
                profile->service_handle = param->create.service_handle;
                profile->char_uuid = parse_uuid("511f350c-5158-4853-81b8-4ab95a687168");
                static esp_attr_value_t gatts_char_val =
                {
                    .attr_max_len = 0x40,
                    .attr_len     = 3,
                    .attr_value   = char_value,
                };
                ESP_ERROR_CHECK(esp_ble_gatts_start_service(param->create.service_handle));
                ESP_ERROR_CHECK(esp_ble_gatts_add_char(profile->service_handle, &profile->char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                                            &gatts_char_val, NULL));
            }
            break;
        default:
            break;
    }
}

static void ble_gatts_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGI(TAG, "ble_gatts_callback %d %d %p", event, gatts_if, param);
    switch(event) {
        case ESP_GATTS_REG_EVT:
        ESP_UTILS_CHECK_FALSE_EXIT(param->reg.status == ESP_GATT_OK, "GATTS registration failed");
        ESP_UTILS_CHECK_FALSE_EXIT(param->reg.app_id == MOTOCAST_APP_ID, "wrong app id");
        {
            struct gatts_profile_inst* profile = gl_profile_tab + param->reg.app_id;
            profile->gatts_if = gatts_if;
            profile->service_id.is_primary = true;
            profile->service_id.id.inst_id = 0;
            profile->service_id.id.uuid = parse_uuid("0b61c95e-f4e2-4080-9188-9d8ddb294925");
            ESP_ERROR_CHECK(esp_ble_gap_set_device_name("Motocast"));

            adv_config_done = adv_config_done | scan_rsp_config_flag;
            // The length of adv data must be less than 31 bytes
            //static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
            //adv data
            static esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = false,
                .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
                .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
                .appearance = 0x00,
                .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
                .p_manufacturer_data =  NULL, //&test_manufacturer[0],
                .service_data_len = 0,
                .p_service_data = NULL,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            adv_data.service_uuid_len = profile->service_id.id.uuid.len;
            adv_data.p_service_uuid = profile->service_id.id.uuid.uuid.uuid128;
            // scan response data
            static esp_ble_adv_data_t scan_rsp_data = {
                .set_scan_rsp = true,
                .include_name = true,
                .include_txpower = true,
                //.min_interval = 0x0006,
                //.max_interval = 0x0010,
                .appearance = 0x00,
                .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
                .p_manufacturer_data =  NULL, //&test_manufacturer[0],
                .service_data_len = 0,
                .p_service_data = NULL,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };

            scan_rsp_data.service_uuid_len = profile->service_id.id.uuid.len;
            scan_rsp_data.p_service_uuid = profile->service_id.id.uuid.uuid.uuid128;

            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));

            ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &profile->service_id, 4)); //FIXME: what is this 4???
        }
        break;
    default:
        do {
            for (int idx = 0; idx < MOTOCAST_NUM_APPS; ++idx) {
                if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                        gatts_if == gl_profile_tab[idx].gatts_if) {
                    if (gl_profile_tab[idx].gatts_cb) {
                        gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                    }
                }
            }
        } while (0);
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    waveshare_init();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // ESP_ERROR_CHECK(esp_ble_gap_set_device_name("Motocast"));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(&ble_gatts_callback));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(&ble_gap_callback));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(MOTOCAST_APP_ID));
}
