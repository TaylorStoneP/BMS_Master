/*
 * can_utils.c
 *
 *  Created on: Nov 6, 2022
 *      Author: Taylor Phillips
 */
#include "can_utils.h"
#include "config_loader.h"
#include "control.h"
#include "fault_handler.h"
#if USB_EN == True
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "ascii_hex.h"
#endif

CAN_Message CAN_RX_Queue[32];
uint8_t CAN_RX_Queue_Count;

CAN_RxHeaderTypeDef CAN_RxHeader;
uint8_t CAN_Rx_Message[8];

void CAN_RX_Handler(){
	if(CAN_RX_Queue_Count==0){
		return;
	}
	CAN_RX_Queue_Count--;

    switch(CAN_RX_Queue[CAN_RX_Queue_Count].id){
    case  0xA22:
    	switch(CAN_RX_Queue[CAN_RX_Queue_Count].data[0]){
    	case 0:
    		//Initialise Configuration Sequence;
    		InitialiseConfigurationSequence();
    		break;
    	case 1:
    		//If initialised & expected data ID, retrieve and request.
    	{
    		uint16_t cfg_data_id=0;
    		uint32_t cfg_data=0;

    		cfg_data_id = (CAN_RX_Queue[CAN_RX_Queue_Count].data[1]<<8) | (CAN_RX_Queue[CAN_RX_Queue_Count].data[2]);
    		cfg_data = (CAN_RX_Queue[CAN_RX_Queue_Count].data[4]<<24) | (CAN_RX_Queue[CAN_RX_Queue_Count].data[5]<<16) | (CAN_RX_Queue[CAN_RX_Queue_Count].data[6]<<8) | (CAN_RX_Queue[CAN_RX_Queue_Count].data[7]);
    		ConfigurationSequenceDataIn(cfg_data_id, cfg_data);
    	}
    		break;
    	case 2:
    		//If completion expected, flash memory.
    		ConfigurationSequenceComplete();
    		break;
    	default:
    		break;
    	}
    	break;
	case  0xA23:
		switch(CAN_RX_Queue[CAN_RX_Queue_Count].data[0]){
		case 0:
			//Initialise Configuration Sequence;
			InitialiseConfigurationSendSequence();
			break;
		case 1:
			//If initialised & expected data ID, retrieve and request.
		{
			uint16_t cfg_data_id=0;

			cfg_data_id = (CAN_RX_Queue[CAN_RX_Queue_Count].data[1]<<8) | (CAN_RX_Queue[CAN_RX_Queue_Count].data[2]);

			if(cfg_data_id!=0xFFFF){
				ConfigurationSequenceDataSend(cfg_data_id);
			}else{
				if((CFG_Progress_ID+1)==N_CONFIG_WORDS){
					CFGTXS = CFGTXS_Finished;
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 0, 2);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 1, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 2, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 3, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 4, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 5, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 6, 0);
					CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 7, 0);
					CAN_UTIL_Transmit(&hcan1, CAN_TX_ConfigurationTX);
				}
			}
		}
			break;
		case 2:
			//If completion expected, flash memory.
			if(CFGTXS==CFGTXS_Finished){
				ConfigurationSequenceSendComplete();
			}
			break;
		default:
			break;
		}
		break;

	case 0x1C00:
		BITSET(DeviceConnections,DEVCON_Client);
		OUTPUT_SET(OUT_LED_WARN);
		ScheduleTask(SCH_TIMEOUT_Client, 1000, True,0);
		break;
	case 0x18FF50E5:
		BITSET(DeviceConnections, DEVCON_Charger);
		hchgr.voltage = BYTECONCAT16(CAN_RX_Queue[CAN_RX_Queue_Count].data[0], CAN_RX_Queue[CAN_RX_Queue_Count].data[1]);
		hchgr.current = BYTECONCAT16(CAN_RX_Queue[CAN_RX_Queue_Count].data[2], CAN_RX_Queue[CAN_RX_Queue_Count].data[3]);
		hchgr.state = CAN_RX_Queue[CAN_RX_Queue_Count].data[4];
		hchgr.temperature = CAN_RX_Queue[CAN_RX_Queue_Count].data[5];

		if(hchgr.state){
			FaultRaise(FAULT_C2_CHARGER_FAULT, FaultInfoBuff);
		}else{
			FaultLower(FAULT_C2_CHARGER_FAULT);
		}
		FaultLower(FAULT_D2_DEVCON_CHARGER);
		ScheduleTask(SCH_TIMEOUT_Charger, 2000, True,0);
		break;
	case 0x00000A84:
		BITSET(DeviceConnections, DEVCON_RCB);
		if(CAN_RX_Queue[CAN_RX_Queue_Count].data[0]==0x55){
			BMS_Data.ts_active = True;
			if(BITCHECK(DeviceConnections, DEVCON_Charger)){
				if(Status_Check(STATUS_CHARGE_RDY)){
					Status_Set(STATUS_CHARGE);
				}
			}else{
				if(Status_Check(STATUS_DISCHARGE_RDY)){
					Status_Set(STATUS_DISCHARGE);
				}
			}
		}else{
			if(BMS_Data.ts_active == True){
				if(BITCHECK(DeviceConnections, DEVCON_Charger)){
					Status_Set(STATUS_CHARGE_RDY);
				}else{
					Status_Set(STATUS_DISCHARGE_RDY);
				}
			}
			BMS_Data.ts_active = False;

		}
		FaultLower(FAULT_D1_DEVCON_RCB);
		ScheduleTask(SCH_TIMEOUT_RCB, 1500, True,0);
		break;
	case 0x1C38:
		{
		uint16_t cell_id = (CAN_RX_Queue[CAN_RX_Queue_Count].data[0]<<8) + CAN_RX_Queue[CAN_RX_Queue_Count].data[1];
			if(!CFG_Main[CFGID_BalancingEnable]){
				BMS_Data.DCC[cell_id/CELLS_PER_SEG][cell_id%CELLS_PER_SEG] = CAN_RX_Queue[CAN_RX_Queue_Count].data[2];
			}
		}
		break;
    default:
    	break;
    }
}
CAN_Message CAN_Messages[CAN_TX_N];

uint8_t CAN_UTIL_TRANSMITUSB_BUFFER[64];
uint8_t CAN_UTIL_RXUSB_BUFFER[64];
uint8_t CAN_UTIL_TRANSMITUSB_BYTES[8];


#if USB_EN == True
void CAN_UTIL_USB_Handler(uint8_t * buffer, uint8_t len){
	//if(len != 28)		{return;}
	if(buffer[0]!='x')	{return;}

	CAN_RX_Queue[CAN_RX_Queue_Count].length = 8;

	char id_buffer[8] = {buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]};
	uint32_t id = ascii_string_hex_to_int(id_buffer, 8);
	CAN_RX_Queue[CAN_RX_Queue_Count].id = id;

	char data_buffer[2] = {0,0};

	//1 + 8 + 1 + 16
	for(int i = 0;i<8;i++){
		data_buffer[0] = buffer[10+i*2];
		data_buffer[1] = buffer[11+i*2];
		CAN_RX_Queue[CAN_RX_Queue_Count].data[i] = ascii_string_hex_to_int(data_buffer, 2);
	}
	CAN_RX_Queue_Count++;
}


uint8_t USB_free = 1;

extern USBD_HandleTypeDef hUsbDeviceFS;

void CAN_UTIL_TransmitUSB(CAN_TX message){
	uint16_t length;

	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0){
		return;
	}
	length = CAN_UTIL_ToString(message, CAN_UTIL_TRANSMITUSB_BUFFER);
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, CAN_UTIL_TRANSMITUSB_BUFFER, length);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, CAN_UTIL_RXUSB_BUFFER);
	USBD_CDC_TransmitPacket(&hUsbDeviceFS);


}
#endif

void CAN_UTIL_Transmit(CAN_HandleTypeDef *hcan, CAN_TX message){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Tx_Mailbox;

	TxHeader.DLC = CAN_Messages[message].length;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.ExtId = CAN_Messages[message].id;

	HAL_CAN_AddTxMessage(hcan, &TxHeader, CAN_Messages[message].data, &Tx_Mailbox);
	CAN_UTIL_LED_Code(HAL_GPIO_TogglePin(CAN_UTIL_LED_Port, CAN_UTIL_LED_Pin));
}

void CAN_UTIL_TransmitMessage(CAN_HandleTypeDef *hcan, CAN_Message message){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Tx_Mailbox;

	TxHeader.DLC = message.length;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.ExtId = message.id;

	HAL_CAN_AddTxMessage(hcan, &TxHeader, message.data, &Tx_Mailbox);
	CAN_UTIL_LED_Code(HAL_GPIO_TogglePin(CAN_UTIL_LED_Port, CAN_UTIL_LED_Pin));
}

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
void CAN_UTIL_SetData(CAN_TX message, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7){
	CAN_Messages[message].data[0]=data_0;
	CAN_Messages[message].data[1]=data_1;
	CAN_Messages[message].data[2]=data_2;
	CAN_Messages[message].data[3]=data_3;
	CAN_Messages[message].data[4]=data_4;
	CAN_Messages[message].data[5]=data_5;
	CAN_Messages[message].data[6]=data_6;
	CAN_Messages[message].data[7]=data_7;
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
	buffer[length] = '\0';
	return length;
}

uint32_t ascii_string_hex_to_int(char* hex, char length)
{
    uint32_t out = 0;
    uint32_t inchar = 0;
    for(int i = 0;i<length;i++)
    {
        inchar = ascii_hex_to_int(hex[i]);
        if(inchar!=-1)
        {
            out+=inchar;
            if(inchar<0)
            {

            }
        }
        if(i<length-1)
        {
            out = out<<4;
        }
    }
    return out;
}

uint8_t ascii_hex_to_int(char hex)
{
    switch(hex){
    case '0':
        return 0;
    case '1':
        return 1;
    case '2':
        return 2;
    case '3':
        return 3;
    case '4':
        return 4;
    case '5':
        return 5;
    case '6':
        return 6;
    case '7':
        return 7;
    case '8':
        return 8;
    case '9':
        return 9;
    case 'A':
        return 10;
    case 'B':
        return 11;
    case 'C':
        return 12;
    case 'D':
        return 13;
    case 'E':
        return 14;
    case 'F':
        return 15;
    case 'a':
        return 10;
    case 'b':
        return 11;
    case 'c':
        return 12;
    case 'd':
        return 13;
    case 'e':
        return 14;
    case 'f':
        return 15;
    default:
        return -1;
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef * hcan){

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan){
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_Rx_Message);
	if(hcan == &hcan1){
		if(CAN_RxHeader.IDE==CAN_ID_EXT){
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.ExtId;
		}else{
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.StdId;
		}
	    CAN_RX_Queue[CAN_RX_Queue_Count].length = CAN_RxHeader.DLC;
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[0] = CAN_Rx_Message[0];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[1] = CAN_Rx_Message[1];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[2] = CAN_Rx_Message[2];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[3] = CAN_Rx_Message[3];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[4] = CAN_Rx_Message[4];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[5] = CAN_Rx_Message[5];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[6] = CAN_Rx_Message[6];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[7] = CAN_Rx_Message[7];
	    CAN_RX_Queue_Count++;

	}

#if N_CANBUS == 2
	if(hcan == &hcan2){
		if(CAN_RxHeader.IDE==CAN_ID_EXT){
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.ExtId;
		}else{
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.StdId;
		}
	    CAN_RX_Queue[CAN_RX_Queue_Count].length = CAN_RxHeader.DLC;
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[0] = CAN_Rx_Message[0];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[1] = CAN_Rx_Message[1];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[2] = CAN_Rx_Message[2];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[3] = CAN_Rx_Message[3];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[4] = CAN_Rx_Message[4];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[5] = CAN_Rx_Message[5];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[6] = CAN_Rx_Message[6];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[7] = CAN_Rx_Message[7];
	    CAN_UTIL_TransmitMessage(&hcan1, CAN_RX_Queue[CAN_RX_Queue_Count]);
	    CAN_RX_Queue_Count++;
	}
#endif

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan){
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_RxHeader, CAN_Rx_Message);
	if(hcan == &hcan1){
		if(CAN_RxHeader.IDE==CAN_ID_EXT){
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.ExtId;
		}else{
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.StdId;
		}
	    CAN_RX_Queue[CAN_RX_Queue_Count].length = CAN_RxHeader.DLC;
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[0] = CAN_Rx_Message[0];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[1] = CAN_Rx_Message[1];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[2] = CAN_Rx_Message[2];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[3] = CAN_Rx_Message[3];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[4] = CAN_Rx_Message[4];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[5] = CAN_Rx_Message[5];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[6] = CAN_Rx_Message[6];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[7] = CAN_Rx_Message[7];
	    CAN_RX_Queue_Count++;

	}

#if N_CANBUS == 2
	if(hcan == &hcan2){
		if(CAN_RxHeader.IDE==CAN_ID_EXT){
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.ExtId;
		}else{
			CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.StdId;
		}
	    CAN_RX_Queue[CAN_RX_Queue_Count].length = CAN_RxHeader.DLC;
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[0] = CAN_Rx_Message[0];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[1] = CAN_Rx_Message[1];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[2] = CAN_Rx_Message[2];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[3] = CAN_Rx_Message[3];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[4] = CAN_Rx_Message[4];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[5] = CAN_Rx_Message[5];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[6] = CAN_Rx_Message[6];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[7] = CAN_Rx_Message[7];
	    CAN_UTIL_TransmitMessage(&hcan1, CAN_RX_Queue[CAN_RX_Queue_Count]);
	    CAN_RX_Queue_Count++;
	}
#endif

}
