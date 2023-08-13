#include "config_loader.h"
#include "can_utils.h"

uint32_t CFG_flash_buffer[N_CONFIG_WORDS];
uint32_t CFG_Main[N_CONFIG_WORDS];
uint32_t CFG_Backup[N_CONFIG_WORDS];
uint16_t CFGE=CFGE_None;
uint16_t CFGS=CFGS_None;
uint16_t CFGTXS=CFGTXS_None;
uint32_t CFGFS=CFGFS_None;
uint16_t CFG_Progress_ID=0;
uint8_t CFG_Reset = False;
uint32_t CFG_Info[32];


void InitialiseConfigurationSequence(){
	if(CFGS!=CFGS_None){
		//error restarted
		CFGE=CFGE_Restarted;
	}
	if(CFGE){ConfigurationErrorHandler();return;}

	HAL_TIM_Base_Stop_IT(&SOFTCLK_TIMER_TYPE);
	HAL_TIM_Base_Stop(&STOPCLK_TIMER_TYPE);
	//DisableIRQ();

	CFGFS = CFGFS_Config;
	CFG_Info[0] = CFGFS;
	Flash_Write_Data(SECTOR5, CFG_Info, 1);
	CFGS = CFGS_Init;
	CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 1, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 2, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 3, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 4, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 5, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 6, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 7, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
	//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);
	CFG_Progress_ID = 0;

	for(int task = 0;task<SCHEDULE_N;task++){
		ScheduleTask(task, 0xFFFFFFFF, False,0);
	}

	//When Ready...
	CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 1);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 1, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 2, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
	//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);
}

void ConfigurationSequenceDataIn(uint16_t index, uint32_t data){
	if(CFG_Progress_ID!=index){
		//error id mismatch
		CFGE = CFGE_ID_Mismatch;
	}
	if(CFGS==CFGS_None){
		//error started before initialisation
		CFGE = CFGE_PreInit;
	}else if(CFGS==CFGS_ReadyToFlash){
		//error bms ready to flash early
		CFGE = CFGE_FlashReady_Early;
	}
	if(CFGE){ConfigurationErrorHandler();return;}

	CFG_flash_buffer[CFG_Progress_ID] = data;

	CFG_Progress_ID++;
	if(CFG_Progress_ID==(N_CONFIG_WORDS))
	{
		//ACK & Request Finish
		CFGS = CFGS_ReadyToFlash;
		CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 1);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 1, 0xFF);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 2, 0xFF);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 3, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 4, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 5, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 6, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 7, 0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
		//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);
	}else{
		//ACK & Request Data
		CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 1);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 1, BYTE1(CFG_Progress_ID));
		CAN_UTIL_SetByte(CAN_TX_Configuration, 2, BYTE0(CFG_Progress_ID));
		CAN_UTIL_SetByte(CAN_TX_Configuration, 3, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 4, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 5, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 6, 0);
		CAN_UTIL_SetByte(CAN_TX_Configuration, 7, 0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
		//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);
	}
}

void ConfigurationSequenceComplete(){
	if(CFGS!=CFGS_ReadyToFlash){
		//error finish before ready to flash
		CFGE = CFGE_FlashReady_Late;
	}
	if(CFGE){ConfigurationErrorHandler();return;}

	CFGFS = CFGFS_FlashMain;
	CFG_Info[0] = CFGFS;
	Flash_Write_Data(SECTOR5, CFG_Info, 1);

	//Flash Main Data
	Flash_Write_Data(SECTOR7, CFG_flash_buffer, N_CONFIG_WORDS);

	CFGFS = CFGFS_FlashBackup;
	CFG_Info[0] = CFGFS;
	Flash_Write_Data(SECTOR5, CFG_Info, 1);

	//Flash Backup Data
	Flash_Write_Data(SECTOR6, CFG_flash_buffer, N_CONFIG_WORDS);


	CFG_Progress_ID = 0;
	CFGS = CFGS_None;


	//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);

	CFGFS = CFGFS_None;
	CFG_Info[0] = CFGFS;
	Flash_Write_Data(SECTOR5, CFG_Info, 1);

	CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 2);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 1, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 2, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 3, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 4, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 5, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 6, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 7, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
	CFG_Reset = True;

}

void ConfigurationErrorHandler(){
	CAN_UTIL_SetByte(CAN_TX_Configuration, 0, 3);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 1, BYTE1(CFGE));
	CAN_UTIL_SetByte(CAN_TX_Configuration, 2, BYTE0(CFGE));
	CAN_UTIL_SetByte(CAN_TX_Configuration, 3, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 4, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 5, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 6, 0);
	CAN_UTIL_SetByte(CAN_TX_Configuration, 7, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_Configuration);
	//CAN_UTIL_TransmitUSB(CAN_TX_Configuration);
	CFG_Progress_ID = 0;
	CFGS = CFGS_Error;
}

void InitialiseConfigurationSendSequence(){
	if(CFGTXS!=CFGTXS_None){
		CFGE=CFGE_Restarted;
	}
	if(CFGE){ConfigurationErrorHandler();return;}

	HAL_TIM_Base_Stop_IT(&SOFTCLK_TIMER_TYPE);
	HAL_TIM_Base_Stop(&STOPCLK_TIMER_TYPE);

	//DisableIRQ();

	CFGTXS = CFGTXS_Init;

	//Init Ack
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 0, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 1, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 2, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 3, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 4, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 5, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 6, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 7, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_ConfigurationTX);
	CFG_Progress_ID = 0;

	//Prepare
	//

	CFGTXS = CFGTXS_Sending;
	//Ready Signal
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 0, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 1, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 2, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 3, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 4, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 5, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 6, 0);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 7, 0);
	CAN_UTIL_Transmit(&hcan1, CAN_TX_ConfigurationTX);

}
void ConfigurationSequenceDataSend(uint16_t index){
	CFG_Progress_ID = index;

	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 0, 1);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 1, BYTE1(CFG_Progress_ID));
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 2, BYTE0(CFG_Progress_ID));
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 3, 4);
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 4, BYTE3(CFG_Main[CFG_Progress_ID]));
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 5, BYTE2(CFG_Main[CFG_Progress_ID]));
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 6, BYTE1(CFG_Main[CFG_Progress_ID]));
	CAN_UTIL_SetByte(CAN_TX_ConfigurationTX, 7, BYTE0(CFG_Main[CFG_Progress_ID]));
	CAN_UTIL_Transmit(&hcan1, CAN_TX_ConfigurationTX);


}
void ConfigurationSequenceSendComplete(){
	CFGE=CFGE_None;
	CFGS=CFGS_None;
	CFGTXS=CFGTXS_None;
	CFGFS=CFGFS_None;
	CFG_Progress_ID=0;
	HAL_TIM_Base_Start_IT(&SOFTCLK_TIMER_TYPE);
	HAL_TIM_Base_Start(&STOPCLK_TIMER_TYPE);
	//EnableIRQ();
}
