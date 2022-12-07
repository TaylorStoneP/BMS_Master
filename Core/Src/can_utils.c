/*
 * can_utils.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Taylor Phillips
 */
#include "can_utils.h"
#if USB_EN == True
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "ascii_hex.h"
#endif
CAN_Message CAN_Messages[CAN_TX_N];

uint8_t CAN_UTIL_TRANSMITUSB_BUFFER[64];
uint8_t CAN_UTIL_TRANSMITUSB_BYTES[8];

void CAN_UTIL_Transmit(CAN_HandleTypeDef *hcan, CAN_TX message){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Tx_Mailbox;

	TxHeader.DLC = CAN_Messages[message].length;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.ExtId = CAN_Messages[message].id;

	HAL_CAN_AddTxMessage(hcan, &TxHeader, CAN_Messages[message].data, &Tx_Mailbox);
	CAN_UTIL_LED_Code(HAL_GPIO_WritePin(CAN_UTIL_LED_Port, CAN_UTIL_LED_Pin, GPIO_PIN_SET));
}
#if USB_EN == True
void CAN_UTIL_TransmitUSB(CAN_TX message){
	uint16_t length = CAN_UTIL_ToString(message, CAN_UTIL_TRANSMITUSB_BUFFER);
	CDC_Transmit_FS(CAN_UTIL_TRANSMITUSB_BUFFER, length);

}
#endif
void CAN_UTIL_SetID(CAN_TX message, uint32_t id){
	CAN_Messages[message].id = id;
}
void CAN_UTIL_SetLength(CAN_TX message, uint8_t length){
	CAN_Messages[message].length = length;
}

void CAN_UTIL_Setup(CAN_TX message, uint32_t id, uint8_t length){
	CAN_UTIL_SetID(message, id);
	CAN_UTIL_SetLength(message, length);
}

void CAN_UTIL_SetByte(CAN_TX message, uint8_t index, uint8_t data){
	if(index>7){return;}
	CAN_Messages[message].data[index]=data;
}
uint8_t CAN_UTIL_GetByte(CAN_TX message, uint8_t index){
	if(index>7){return 0;}
	return CAN_Messages[message].data[index];
}

uint16_t CAN_UTIL_ToString(CAN_TX message, uint8_t buffer[64]){
	uint16_t length = 0;
	buffer[length++] = 'x';
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_Messages[message].id, 4);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[2];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[3];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[4];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[5];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[6];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[7];
	buffer[length++] = '8';
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,0), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,1), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,2), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,3), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,4), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,5), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,6), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	number_to_ascii_hex(CAN_UTIL_TRANSMITUSB_BYTES, CAN_UTIL_GetByte(message,7), 1);
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[0];
	buffer[length++] = CAN_UTIL_TRANSMITUSB_BYTES[1];
	buffer[length++]='\r';
	return length;
}
