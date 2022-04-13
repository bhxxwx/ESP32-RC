/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_pm.h"
#include "esp_bt_main.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"
#include "CommandSystem.h"
#include "Servers.h"
#include "ADCServer.h"
#define TAG "EXAMPLE"

#define CID_ESP 0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x03, CID_ESP)
void OP_PROV_Func(void *arg);
nvs_handle_t NVS_USER_HANDLE;
static bool _is_proved = false;
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0xAA, 0x10, 0xBB};

static esp_ble_mesh_cfg_srv_t config_server = {
	.relay = ESP_BLE_MESH_RELAY_DISABLED,
	.beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
	.friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
	.friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
	.gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
	.default_ttl = 7,
	/* 3 transmissions with 20ms interval */
	.net_transmit = ESP_BLE_MESH_TRANSMIT(2, 500),
	.relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 500),
};

static esp_ble_mesh_model_t root_models[] = {
	ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
	ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
	ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
	ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
							  vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
	ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
	.cid = CID_ESP,
	.elements = elements,
	.element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
	.uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
	ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
	ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);

	//    board_led_operation(LED_G, LED_OFF);
}

void example_ble_mesh_send_vendor_message(uint8_t *data, uint8_t len)
{
	esp_ble_mesh_msg_ctx_t ctx = {0};
	uint32_t opcode;
	esp_err_t err;
	//	esp_bluedroid_enable();
	ctx.net_idx = 0;
	ctx.app_idx = 0;
	ctx.addr = 1;

	ESP_LOGI(TAG, "ctx.addr 0x%02x", ctx.addr);
	ctx.send_ttl = 7;
	ctx.send_rel = 0;
	err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], &ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS, len, (uint8_t *)data);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to send vendor message ");
		return;
	}
	//	mesh_example_info_store(); /* Store proper mesh example info */
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param)
{
	switch (event)
	{
	case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
		break;
	case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
		break;
	case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
				 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
		break;
	case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
				 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
		break;
	case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
		prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr, param->node_prov_complete.flags, param->node_prov_complete.iv_index);
		break;
	case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
		break;
	case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
		ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
		break;
	default:
		break;
	}
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event, esp_ble_mesh_cfg_server_cb_param_t *param)
{
	if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
	{
		switch (param->ctx.recv_op)
		{
		case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
			ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
			ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x", param->value.state_change.appkey_add.net_idx, param->value.state_change.appkey_add.app_idx);
			ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
			break;
		case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
			ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
			ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x", param->value.state_change.mod_app_bind.element_addr, param->value.state_change.mod_app_bind.app_idx, param->value.state_change.mod_app_bind.company_id, param->value.state_change.mod_app_bind.model_id);
			_is_proved = true;
			break;
		default:
			break;
		}
	}
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
	switch (event)
	{
	case ESP_BLE_MESH_MODEL_OPERATION_EVT:
		if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND)
		{
			uint16_t tid = *(uint16_t *)param->model_operation.msg;
			ESP_LOGI(TAG, "Recv 0x%06x, tid 0x%04x", param->model_operation.opcode, tid);
			esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS, sizeof(tid), (uint8_t *)&tid);
			if (err)
			{
				ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
			}
		}
		break;
	case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
		if (param->model_send_comp.err_code)
		{
			ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
			break;
		}
		ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
		break;
	default:
		break;
	}
}

static esp_err_t ble_mesh_init(void)
{
	esp_err_t err;

	esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
	esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
	esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

	err = esp_ble_mesh_init(&provision, &composition);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to initialize mesh stack");
		return err;
	}

	err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV );
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to enable mesh node");
		return err;
	}

	//    board_led_operation(LED_G, LED_ON);

	ESP_LOGI(TAG, "BLE Mesh Node initialized");

	return ESP_OK;
}
static void echo_task(void *arg);
static void hallSenor_task(void *arg);
static void ST_BLUT_FUNC(void *arg);
void app_main(void)
{
	esp_err_t err;
	uint8_t FactoryMode = 0xFF;
	esp_pm_config_esp32_t pm_config;
	pm_config.max_freq_mhz = 80;
	pm_config.min_freq_mhz = 10;
	pm_config.light_sleep_enable = true;
	ESP_LOGI(TAG, "Initializing...");

	err = nvs_flash_init_partition("nvs_user");
	if (err == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase_partition("nvs_user"));
		err = nvs_flash_init_partition("nvs_user");
	}
	ESP_ERROR_CHECK(err);

	nvs_open_from_partition("nvs_user", "user_infos", NVS_READWRITE, &NVS_USER_HANDLE);
	if (ReadFromNVS("FactoryMode", &FactoryMode, NVS_USER_HANDLE))
	{
		if (FactoryMode != 0)
		{
			nvs_flash_erase();
			dev_uuid[0] = 0xFA;
		}
	}
	else
	{
		nvs_flash_erase();
		dev_uuid[0] = 0xFA;
	}
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	board_init(FactoryMode);
	err = bluetooth_init();
	if (err)
	{
		ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
		return;
	}

	ble_mesh_get_dev_uuid(dev_uuid);
	dev_uuid[1] = 0x10;
	dev_uuid[2] = 0xBB;
	ESP_LOGI(TAG, "dev_uuid：%x,%x,%x", dev_uuid[0], dev_uuid[1], dev_uuid[2]);
	/* Initialize the Bluetooth Mesh Subsystem */
	err = ble_mesh_init();
	if (err)
	{
		ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
	}
	if (dev_uuid[0] == 0xFA)
		ESP_LOGI(TAG, "In Factory Mode");
	if (dev_uuid[0] == 0xAA)
		ESP_LOGI(TAG, "In Normal Mode");
	ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
		CommandSystemInit();
		CommandReg("OP_PROV", OP_PROV_Func);
	//	CommandReg("CL_PROV", CL_PROV_Func);
		CommandReg("RE_FACT", RE_FACT_Func);
		CommandReg("ST_BLUT", ST_BLUT_FUNC);
	//	CommandReg("PR_OPPN", OnOff_key);
	//	CommandReg("PR_MLLK", MainColorLightTog_key);
	//	CommandReg("PR_LLVK", LightLevel_key);
		xTaskCreatePinnedToCore(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(hallSenor_task, "hallSenor_task", 4096, NULL, 10, NULL, 0);
}

static void ST_BLUT_FUNC(void *arg)
{
	if (*(uint8_t *)arg == '$')
	{
		ESP_LOGI(TAG, "In decode");
		uint8_t data[2];
		DecodeCommandValue((char *)arg, data);
		ESP_LOGI(TAG, "In decode:%d", data[0]);
		*(char *)arg = data[0] + 48;
	}
	if (*(uint8_t *)arg == '0')
	{
		// esp_bluedroid_disable();
		// esp_bt_sleep_enable();
	}
	else
	{
		// esp_bluedroid_enable();
	}
}

void OP_PROV_Func(void *arg)
{
	ESP_LOGI(TAG, "arg1:%s", (char *)arg);
	if (*(uint8_t *)arg=='$')
	{
		ESP_LOGI(TAG, "In decode");
		uint8_t data[2];
		DecodeCommandValue((char *)arg,data);
		ESP_LOGI(TAG, "In decode:%d",data[0]);

		*(char *)arg = data[0]+48;
	}
	ESP_LOGI(TAG, "arg2:%c", *(char *)arg);
	if (*(uint8_t *)arg == '0')
	{
		LED_op(1);
		ESP_LOGI(TAG, "Close Factory Mode");
		uint8_t FactoryMode;
		if (ReadFromNVS("FactoryMode", &FactoryMode, NVS_USER_HANDLE))
		{
			if (FactoryMode != 0)
				WriteToNVS("FactoryMode", 0, NVS_USER_HANDLE);
		}
		else
			WriteToNVS("FactoryMode", 0, NVS_USER_HANDLE);
		}
	if (*(uint8_t *)arg == '1')
	{
		ESP_LOGI(TAG, "reset");
		nvs_flash_erase();
		esp_restart();
		// LED_op(0);
	}
}

static void echo_task(void *arg)
{
	uint8_t *data = (uint8_t*) malloc(BUF_SIZE);
	while (1)
	{
		// Read data from the UART
		int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 10 / portTICK_RATE_MS);
		if (len > 7)
		{
			data[len] = 0;
			ESP_LOGI("echo_task", "<<%s", data);
			if (DecodeCommandHead(len, (char*) data))
				uart_write_bytes(ECHO_UART_PORT_NUM, ">>CMD_OK\r\n", 10);
			bzero(data, BUF_SIZE);
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
}

static void hallSenor_task(void *arg)
{
	static int old_value = -1;

	while (1)
	{
		uint16_t value;
		value = ADC_GetValue();
		// ESP_LOGI(TAG, "OLD:%d,Cur:%d,Prov:%d", old_value, value, _is_proved);
		if (value < 100 && old_value > 1000)
		{
			ESP_LOGW(TAG, "No proved Falling edge, ready to Restart");
			OP_PROV_Func("0");
			vTaskDelay(100 / portTICK_PERIOD_MS);
			OP_PROV_Func("1");
		}
		if(value<100&&old_value==-1)
		{
			ESP_LOGW(TAG, "Restart finished and still check magnet, so wait PROV");
			while(!_is_proved)
			{
				ESP_LOGI(TAG, "Wait PROV LED Blink");
				LED_op(1);
				vTaskDelay(300 / portTICK_PERIOD_MS);
				LED_op(0);
				vTaskDelay(300 / portTICK_PERIOD_MS);
			}
			ESP_LOGW(TAG, "Proved");
		}
		old_value = value;
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
