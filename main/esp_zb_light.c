/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_color_dimmable_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "Arduino.h"
#include "esp_zb_light.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#define LED_BUILTIN 15

static const int REMOTE_SWITCH_1_PIN = 17;
static const int ONBOARD_BUTTON_PIN = 1;
static const int REMOTE_SWITCH_ON_TIME_MS = 1000;
static const int64_t DEBOUNCE_TIME_MS = 20;//ms
static const int64_t BUTTON_COOLDOWN_TIME_MS = 2000;//ms

esp_timer_handle_t RemoteSwitchOffTimer;
int ButtonPressed = 0;

int64_t FirstInterruptSeqTime = 0;
int64_t SwitchCooldownStart = 0;

static const char *TAG = "ESP_ZB_COLOR_DIMM_LIGHT";
/********************* Define functions **************************/
static esp_err_t deferred_driver_init(void)
{
    //light_driver_init(LIGHT_DEFAULT_OFF);
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            //ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        // https://github.com/espressif/esp-zigbee-sdk/issues/66#issuecomment-1667314481
        esp_zb_zdo_signal_leave_params_t *leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params) {
            if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                ESP_LOGI(TAG, "ZDO leave: with reset, status: %s", esp_err_to_name(err_status));
                esp_zb_nvram_erase_at_start(true); // erase previous network information.
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING); // steering a new network.
            } else {
                ESP_LOGI(TAG, "ZDO leave: leave_type: %d, status: %s", leave_params->leave_type, esp_err_to_name(err_status));
            }
        } else {
            ESP_LOGI(TAG, "ZDO leave: (no params), status: %s", esp_err_to_name(err_status));
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void trigger_remote_1()
{
  ESP_LOGW(TAG, "trigger_remote_1");
  esp_timer_stop(RemoteSwitchOffTimer);
  esp_timer_start_once(RemoteSwitchOffTimer, REMOTE_SWITCH_ON_TIME_MS*1000);
  
  digitalWrite(REMOTE_SWITCH_1_PIN, HIGH);
  
  digitalWrite(LED_BUILTIN, LOW);
}


static void turnoff_remote_1()
{
  ESP_LOGW(TAG, "turnoff_remote_1");
  
  digitalWrite(REMOTE_SWITCH_1_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
    uint16_t light_color_x = 0;
    uint16_t light_color_y = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_COLOR_DIMMABLE_LIGHT_ENDPOINT) {
        switch (message->info.cluster) {
        case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                //light_driver_set_power(light_state);
                trigger_remote_1();
            } else {
                ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                light_color_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : light_color_x;
                light_color_y = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                                                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
                                     ->data_p;
                ESP_LOGI(TAG, "Light color x changes to 0x%x", light_color_x);
            } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
                       message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                light_color_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : light_color_y;
                light_color_x = *(uint16_t *)esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster,
                                                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
                                     ->data_p;
                ESP_LOGI(TAG, "Light color y changes to 0x%x", light_color_y);
            } else {
                ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            light_driver_set_color_xy(light_color_x, light_color_y);
            break;
        case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                light_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : light_level;
                //light_driver_set_level((uint8_t)light_level);
                ESP_LOGI(TAG, "Light level changes to %d", light_level);
            } else {
                ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
            }
            break;
        default:
            ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
  
    esp_zb_secur_network_min_join_lqi_set(0);
  
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // allow joining the Philips Hue network(s)
    esp_zb_enable_joining_to_distributed(true);
    uint8_t secret_zll_trust_center_key[] = {
        // FIXME: this is not the correct key, replace it with the proper one
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
    };
    esp_zb_secur_TC_standard_distributed_key_set(secret_zll_trust_center_key);

    esp_zb_color_dimmable_light_cfg_t light_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_color_dimmable_light_ep = NULL;
    esp_zb_color_dimmable_light_ep = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_COLOR_DIMMABLE_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = 0x0102,
        .app_device_version = 1, // maybe important for Hue? Oh HELL yes.
    };
    esp_zb_ep_list_add_ep(esp_zb_color_dimmable_light_ep, esp_zb_color_dimmable_light_clusters_create(&light_cfg), endpoint_config);
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_color_dimmable_light_ep, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT, &info);

    // https://github.com/espressif/esp-zigbee-sdk/issues/457#issuecomment-2426128314
    uint16_t on_off_on_time = 0;
    bool on_off_global_scene_control = 0;
    esp_zb_cluster_list_t *cluster_list = esp_zb_ep_list_get_ep(esp_zb_color_dimmable_light_ep, HA_COLOR_DIMMABLE_LIGHT_ENDPOINT);
    esp_zb_attribute_list_t *onoff_attr_list =
        esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_add_attr(onoff_attr_list, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &on_off_on_time);
    esp_zb_on_off_cluster_add_attr(onoff_attr_list, ESP_ZB_ZCL_ATTR_ON_OFF_GLOBAL_SCENE_CONTROL,
            &on_off_global_scene_control);
    // .

    esp_zb_device_register(esp_zb_color_dimmable_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void loopTask(void *pvParameters)
{
  bool trigger_issued = false;
  for (;;)
    {
      
      if (ButtonPressed)
      {
        //digitalWrite(LED_BUILTIN, LOW);
        if (!trigger_issued)
        {
          if (esp_timer_get_time()-SwitchCooldownStart > BUTTON_COOLDOWN_TIME_MS*1000)
          {
            trigger_remote_1();
            SwitchCooldownStart = esp_timer_get_time();
          }
          trigger_issued = true;
        }
      }
      else
      {
        trigger_issued = false;
        //digitalWrite(LED_BUILTIN, HIGH);
      }
      
      vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR button_high() {
  ets_printf("button_high %u %u\n", esp_timer_get_time(), FirstInterruptSeqTime);
  if (esp_timer_get_time()-FirstInterruptSeqTime < DEBOUNCE_TIME_MS*1000)
  {
    return;
  }
  
  if (ButtonPressed)
  {
    ButtonPressed = 0;
    ets_printf("ButtonPressed = 0;\n");
  }
  else
  {
    ButtonPressed = 1;
    ets_printf("ButtonPressed = 1;\n");
  }
  FirstInterruptSeqTime = esp_timer_get_time();
  
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(REMOTE_SWITCH_1_PIN, OUTPUT);
    digitalWrite(REMOTE_SWITCH_1_PIN, LOW);
    
    //create the timer for turning off the switch after 1 second
    esp_timer_create_args_t timer_off_args = {
      .callback = &turnoff_remote_1,
      .name = "remote_switch_turnoff"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_off_args, &RemoteSwitchOffTimer));
    
    ESP_LOGW(TAG, "attach interrupt to pin (0x%x)", ONBOARD_BUTTON_PIN);
    pinMode(ONBOARD_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(ONBOARD_BUTTON_PIN, button_high, RISING);
    //ESP_LOGI(TAG, "gogogo 6");
    
    xTaskCreate(loopTask, "loopTask", 4096, NULL, 1, NULL);
    
    
}