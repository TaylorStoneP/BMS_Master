#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include "main.h"
#include "stm32_utils.h"

///===CONFIG===
#define CAN_UTIL_LED_EN True
#define CAN_UTIL_LED_Port LED_CAN_GPIO_Port
#define CAN_UTIL_LED_Pin LED_CAN_Pin

#define USB_EN False

#define N_CANBUS 1

extern CAN_HandleTypeDef hcan1;
#if N_CANBUS == 2
extern CAN_HandleTypeDef hcan2;
#endif

typedef struct{
	uint8_t data[8];
	uint8_t length;
	uint32_t id;
}CAN_Message;

typedef enum CAN_TX{
	CAN_TX_Cells,
	CAN_TX_Temperature_Main,
	CAN_TX_Temperature_Segment,
	CAN_TX_Fan,
	CAN_TX_Internal_Temp,
	CAN_TX_Stopclk,
	CAN_TX_Configuration,
	CAN_TX_ConfigurationTX,
	CAN_TX_Voltage_Bounds,
	CAN_TX_KeepAlive,
	CAN_TX_WarnInfo,
	CAN_TX_BalanceResponse,
	CAN_TX_FaultsAB,
	CAN_TX_FaultsCD,
	CAN_TX_N
}CAN_TX;



extern CAN_Message CAN_Messages[CAN_TX_N];

#if CAN_UTIL_LED_EN == True
	#define CAN_UTIL_LED_Code(code) code
#else
	#define CAN_UTIL_LED_Code(code)
#endif

void CAN_RX_Handler();

#if USB_EN == True
void CAN_UTIL_TransmitUSB(CAN_TX message);
void CAN_UTIL_USB_Handler(uint8_t * buffer, uint8_t len);
#endif

void CAN_UTIL_Transmit(CAN_HandleTypeDef *hcan, CAN_TX message);
void CAN_UTIL_SetID(CAN_TX message, uint32_t id);
void CAN_UTIL_SetLength(CAN_TX message, uint8_t length);
void CAN_UTIL_Setup(CAN_TX message, uint32_t id, uint8_t length);
void CAN_UTIL_SetByte(CAN_TX message, uint8_t index, uint8_t data);
void CAN_UTIL_SetData(CAN_TX message, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7);
uint8_t CAN_UTIL_GetByte(CAN_TX message, uint8_t index);
uint16_t CAN_UTIL_ToString(CAN_TX message, uint8_t* buffer);

uint32_t ascii_string_hex_to_int(char* hex, char length);
uint8_t ascii_hex_to_int(char hex);

#endif


