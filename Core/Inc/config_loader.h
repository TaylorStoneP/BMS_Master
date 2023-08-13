#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include "main.h"
#include "flash_utils.h"

enum CFG_DATA_ID{
    CFGID_MinimumCellVoltage,
    CFGID_MaximumCellVoltage,
    CFGID_MaximumCellVoltageCharging,
    CFGID_MaximumCellDeviation,
    CFGID_MinimumCellVoltageBalancing,
    CFGID_MaximumCellVoltageBalancing,
    CFGID_StartBalancingVoltage,
    CFGID_CellBalanceWindow,
    CFGID_BalancingPeriod,
    CFGID_BalancingWaitPeriod,
    CFGID_MaximumChargeSOC,
    CFGID_MaximumChargeCurrent,
    CFGID_ThermalDerateUpperStartCharge,
    CFGID_ThermalDerateUpperEndCharge,
    CFGID_ThermalDerateLowerStartCharge,
    CFGID_ThermalDerateLowerEndCharge,
    CFGID_ThermalShutdownUpperCharge,
    CFGID_ThermalShutdownLowerCharge,
    CFGID_MinimumDischargeSOC,
    CFGID_MaximumDischargeCurrent,
    CFGID_ThermalDerateUpperStartDischarge,
    CFGID_ThermalDerateUpperEndDischarge,
    CFGID_ThermalDerateLowerStartDischarge,
    CFGID_ThermalDerateLowerEndDischarge,
    CFGID_ThermalShutdownUpperDischarge,
    CFGID_ThermalShutdownLowerDischarge,
    CFGID_FanTemperatureDC0,
    CFGID_FanTemperatureDC10,
    CFGID_FanTemperatureDC20,
    CFGID_FanTemperatureDC30,
    CFGID_FanTemperatureDC40,
    CFGID_FanTemperatureDC50,
    CFGID_FanTemperatureDC60,
    CFGID_FanTemperatureDC70,
    CFGID_FanTemperatureDC80,
    CFGID_FanTemperatureDC90,
    CFGID_FanTemperatureDC100,
    CFGID_FaultcodeFatalA,
    CFGID_FaultcodeFatalB,
    CFGID_FaultcodeFatalC,
    CFGID_FaultcodeFatalD,
    CFGID_BalancingEnable,
    CFGID_BalancingAutoTimingEnable,
    CFGID_BalancingMaximumTemperature,
	CFGID_FaultcodeResettableA,
	CFGID_FaultcodeResettableB,
	CFGID_FaultcodeResettableC,
	CFGID_FaultcodeResettableD,
	CFGID_CurrentSensorReversed,
	CFGID_MinCurrentCharging,
	CFGID_VoltageDerateStartCharging,
	CFGID_FanOverrideEnable,
	CFGID_FanOverrideDC,
	CFGID_MinimumVoltageDischarge,
	CFGID_BatteryImpedance,
	CFGID_MinimumPackVoltage,
    CFGID_N
};

enum CFG_STATE{
	CFGS_None,
	CFGS_Init,
	CFGS_ReadyToFlash,
	CFGS_Error
};

enum CFG_TX_STATE{
	CFGTXS_None,
	CFGTXS_Init,
	CFGTXS_Sending,
	CFGTXS_Finished
};

enum CFG_FLASHING_STATE{
	CFGFS_None,
	CFGFS_Config,
	CFGFS_FlashMain,
	CFGFS_FlashBackup
};

extern uint16_t CFGS;
extern uint16_t CFGTXS;
extern uint32_t CFGFS;
extern uint16_t CFGE;
extern uint16_t CFG_Progress_ID;
extern uint8_t CFG_Reset;

#define N_CONFIG_WORDS CFGID_N

extern uint32_t CFG_Main[N_CONFIG_WORDS];
extern uint32_t CFG_Backup[N_CONFIG_WORDS];
extern uint32_t CFG_Info[32];

void InitialiseConfigurationSequence();
void ConfigurationSequenceDataIn(uint16_t index, uint32_t data);
void ConfigurationSequenceComplete();

void ConfigurationErrorHandler();

void InitialiseConfigurationSendSequence();
void ConfigurationSequenceDataSend(uint16_t index);
void ConfigurationSequenceSendComplete();

enum config_error{
	CFGE_None,
	CFGE_Timeout,
	CFGE_ID_Mismatch,
	CFGE_Restarted,
	CFGE_FlashReady_Late,
	CFGE_FlashReady_Early,
	CFGE_PreInit
};
#endif
