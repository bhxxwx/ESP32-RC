/*
 * Servers.c
 *
 *  Created on: 2021年4月13日
 *      Author: wxujs
 */

#include "Servers.h"


/**
 * @brief 命令头的静态字符串数组,若增加新的命令需要修改此数组
 */
const char CMDheadstr[][8] = { "NULL_", "OP_PROV", "CL_PROV", "RE_FACT", "SE_MSGE", "OP_CDRT", "ST_NSUB" };

//void ProvSet(bool provState,bool factoryMode)
//{
//	uint8_t match[2]={0xAA,0x00};
//
//	if(provState==true)
//	{
//		match[1]=0x10;
//	}
//	else
//	{
//		ESP_LOGI("", "BLE Mesh Provisioner closed");
//	}
//
//	if(factoryMode==true)
//	{
//		match[0]=0xFA;
//		ESP_LOGI("", "BLE Mesh Provisioner Enabled-Factory Mode");
//	}
//	else
//	{
//		ESP_LOGI("", "BLE Mesh Provisioner Enabled-Normal Mode");
//	}
//	if (esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT) != ESP_OK)
//	{
//		ESP_LOGE("", "Failed to Enabled Provisioner");
//	}
//
//}

/**
 * @fn void FactoryReset()
 * @brief 恢复工厂设置
 * 			擦除NVS
 * 			重启ESP32
 *
 */
void FactoryReset()
{
	nvs_flash_erase();
	nvs_flash_erase_partition("nvs_user");
	esp_restart();
}


/**
 * @fn void ReadFromUserNVS(const char*, const char*, uint8_t*)
 * @brief 从NVS(非易失存储)中读取一个uint8_t数据
 *
 * @param tag 日志输出得标头,为"NULL"则不输出日志
 * @param key 存储的数据名称
 * @param data 存储的数据值
 * @return  1:读取成功
 * 			0:读取失败
 */
uint8_t ReadFromNVS(const char *key, uint8_t *data,nvs_handle_t nvs_handle)
{
	esp_err_t err;
	err = nvs_get_u8(nvs_handle, key, data);

	switch (err)
	{
		case ESP_OK:
			return 1;
		case ESP_ERR_NVS_NOT_FOUND:
			return 0;
		default:
			return 0;
	}
}


/**
 * @fn void WriteToNVS(const char*, uint8_t)
 * @brief 向NVS中写入一个uint8_t的数据
 *
 * @param key 须写入数据的名称
 * @param data 须写入数据的值
 * @param nvs_handle NVS句柄
 */
void WriteToNVS(const char *key, uint8_t data,nvs_handle_t nvs_handle)
{
	nvs_set_u8(nvs_handle, key, data);
	nvs_commit(nvs_handle);
}

/**
 * @fn void ReadFromNVS_blob(const char*, void*, uint16_t*)
 * @brief 从NVS中读取可变长度的二进制数据
 *
 * @param key 存储的数据名称
 * @param data 指向存储数据的指针
 * @param nvs_handle NVS句柄
 * @param len 指向uint16_t数据的指针,返回读取的数据长度
 */
void ReadFromNVS_blob(const char *key, void *data, uint16_t *len,nvs_handle_t nvs_handle)
{
	nvs_get_blob(nvs_handle, key, NULL, (size_t*) len);
	nvs_get_blob(nvs_handle, key, data, (size_t*)len);
}

/**
 * @fn void WriteToNVS_blob(const char*, void*, uint16_t)
 * @brief 向NVS中写入可变长度的二进制数据
 *
 * @param key 存储的数据名称
 * @param data 指向被存储数据的指针
 * @param len 可变长度的二进制数据的长度
 * @param nvs_handle NVS句柄
 */
void WriteToNVS_blob(const char *key, void *data, uint16_t len,nvs_handle_t nvs_handle)
{
	nvs_set_blob(nvs_handle, key, data, len);
	nvs_commit(nvs_handle);
}

/**
 * @fn void CleanArray(uint8_t*)
 * @brief 数组清零
 *
 * @param array 需要清零的数组地址
 */
void CleanArray(uint8_t *array)
{
	for (int i = 0; i < sizeof(array); i++)
		*(array + i) = 0;
}


void CL_PROV_Func(void *arg)
{
	ProvSet(false,false);
}

void RE_FACT_Func(void *arg)
{
	FactoryReset();
}

extern void example_ble_mesh_send_vendor_message(uint8_t *data, uint8_t len);
void SE_MSGE_Func(void *arg)
{
	uint8_t data[15] = { 0 };
	uint8_t count = DecodeCommandValue(arg, data);
	example_ble_mesh_send_vendor_message(data,count);
}
