
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef MOTEUR_TRANSLATION_2_H
#define MOTEUR_TRANSLATION_2_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 moteur_translation_2_valueRangeTest (UNS8 typeValue, void * value);
const indextable * moteur_translation_2_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data moteur_translation_2_Data;
extern UNS8 Config_GPIO_ucReserved_2000h1aze;		/* Mapped at index 0x2000, subindex 0x01 */
extern UNS8 Config_GPIO_ucReserved_2000h2aze;		/* Mapped at index 0x2000, subindex 0x02 */
extern UNS8 Config_GPIO_ucReserved_2000h3aze;		/* Mapped at index 0x2000, subindex 0x03 */
extern UNS8 Config_GPIO_ucConfig_IO_PolarityIn_2000h4aze;		/* Mapped at index 0x2000, subindex 0x04 */
extern UNS8 Config_GPIO_ucConfig_IO_PolarityOt_2000h5aze;		/* Mapped at index 0x2000, subindex 0x05 */
extern UNS8 DigitalInput_Configurations_ucHomeSw_Defined_2002h1aze;		/* Mapped at index 0x2002, subindex 0x01 */
extern UNS8 DigitalInput_Configurations_ucPosLimitSw_Defined_2002h2aze;		/* Mapped at index 0x2002, subindex 0x02 */
extern UNS8 DigitalInput_Configurations_ucNegLimitSw_Defined_2002h3aze;		/* Mapped at index 0x2002, subindex 0x03 */
extern UNS8 DigitalInput_Configurations_ucInhibitSw_Defined_2002h4aze;		/* Mapped at index 0x2002, subindex 0x04 */
extern UNS8 DigitalInput_Mask_ucDigInputMask_2004h1aze;		/* Mapped at index 0x2004, subindex 0x01 */
extern UNS8 DigitalInput_Mask_ucDigInputMask_2004h2aze;		/* Mapped at index 0x2004, subindex 0x02 */
extern UNS8 DigitalInput_Mask_ucDigInputMask_2004h3aze;		/* Mapped at index 0x2004, subindex 0x03 */
extern UNS8 DigitalInput_Mask_ucDigInputMask_2004h4aze;		/* Mapped at index 0x2004, subindex 0x04 */
extern UNS8 DigitalInput_FilterTime_ucDigInputFltTime_2006h1aze;		/* Mapped at index 0x2006, subindex 0x01 */
extern UNS8 DigitalInput_FilterTime_ucDigInputFltTime_2006h2aze;		/* Mapped at index 0x2006, subindex 0x02 */
extern UNS8 DigitalInput_FilterTime_ucDigInputFltTime_2006h3aze;		/* Mapped at index 0x2006, subindex 0x03 */
extern UNS8 DigitalInput_FilterTime_ucDigInputFltTime_2006h4aze;		/* Mapped at index 0x2006, subindex 0x04 */
extern UNS8 InhibitSwitch_ucInhibitSw_Action_2007h1aze;		/* Mapped at index 0x2007, subindex 0x01 */
extern UNS8 DigitalOutput_Configurations_ucBrakeSw_Defined_2008h1aze;		/* Mapped at index 0x2008, subindex 0x01 */
extern UNS8 DigitalOutput_Configurations_ucTargetReachedSw_Defined_2008h2aze;		/* Mapped at index 0x2008, subindex 0x02 */
extern UNS8 ucOut21_Fault_2009haze;		/* Mapped at index 0x2009, subindex 0x00*/
extern UNS16 AnalogIn_2010h_Analog_In_Readingaze;		/* Mapped at index 0x2010, subindex 0x01 */
extern UNS8 AnalogIn_2010h_bAnalogInConfig_2010h2aze;		/* Mapped at index 0x2010, subindex 0x02 */
extern UNS8 AnalogIn_2010h_bAnalogInFilterLevel_2010h3aze;		/* Mapped at index 0x2010, subindex 0x03 */
extern UNS16 AuxPower_usAuxPower_x10_Voltage_2014h1aze;		/* Mapped at index 0x2014, subindex 0x01 */
extern UNS16 AuxPower_usAuxPower_Warning_Low_2014h2aze;		/* Mapped at index 0x2014, subindex 0x02 */
extern UNS16 AuxPower_usAuxPower_Warning_High_2014h3aze;		/* Mapped at index 0x2014, subindex 0x03 */
extern UNS16 Vin_usVin_x10_Voltage_2015h1aze;		/* Mapped at index 0x2015, subindex 0x01 */
extern UNS16 Vin_usVin_Warning_Low_2015h2aze;		/* Mapped at index 0x2015, subindex 0x02 */
extern UNS16 Vin_usVin_Warning_High_2015h3aze;		/* Mapped at index 0x2015, subindex 0x03 */
extern INTEGER8 Temperature_Parameters_scTempC_Board_Reading_2018h1aze;		/* Mapped at index 0x2018, subindex 0x01 */
extern INTEGER8 Temperature_Parameters_scTempC_Board_Warning_2018h2aze;		/* Mapped at index 0x2018, subindex 0x02 */
extern INTEGER8 Temperature_Parameters_scTempC_Board_Error_2018h3aze;		/* Mapped at index 0x2018, subindex 0x03 */
extern INTEGER8 Temperature_Error_2019h3_scTempC_Bridge_Reading_2019h1aze;		/* Mapped at index 0x2019, subindex 0x01 */
extern INTEGER8 Temperature_Error_2019h3_scTempC_Bridge_Warning_2019h2aze;		/* Mapped at index 0x2019, subindex 0x02 */
extern INTEGER8 Temperature_Error_2019h3_scTempC_Bridge_Error_2019h3aze;		/* Mapped at index 0x2019, subindex 0x03 */
extern UNS8 LimitReached_bLimitReachedFlag_2020h1aze;		/* Mapped at index 0x2020, subindex 0x01 */
extern UNS8 LimitReached_bLimitReachedMask_2020h2aze;		/* Mapped at index 0x2020, subindex 0x02 */
extern INTEGER32 ActPositionSoftLimit_lAct_Position_Neg_SoftLimit_user_2022h1aze;		/* Mapped at index 0x2022, subindex 0x01 */
extern INTEGER32 ActPositionSoftLimit_lAct_Position_Pos_SoftLimit_user_2022h2aze;		/* Mapped at index 0x2022, subindex 0x02 */
extern INTEGER8 scBridgeSign_2030haze;		/* Mapped at index 0x2030, subindex 0x00*/
extern UNS8 ucUnitOptions_2031haze;		/* Mapped at index 0x2031, subindex 0x00*/
extern UNS8 CaptureIn_Parameters_ucCaptureInCtrl_2033h1aze;		/* Mapped at index 0x2033, subindex 0x01 */
extern UNS8 CaptureIn_Parameters_ucCaptureInFlag_2033h2aze;		/* Mapped at index 0x2033, subindex 0x02 */
extern UNS8 CaptureIn_Parameters_ucCaptureInFilter_2033h3aze;		/* Mapped at index 0x2033, subindex 0x03 */
extern INTEGER32 CaptureIn_Parameters_lCaptureInPositn_user_2033h4aze;		/* Mapped at index 0x2033, subindex 0x04 */
extern UNS16 Bridge_To_Brake_Timers_usBridgeOn_To_BrakeOff_TimeMs_2034h1aze;		/* Mapped at index 0x2034, subindex 0x01 */
extern UNS16 BrakeSignal_To_Bridge_Timer_usBrakeOnSignal_To_BridgeOff_TimeMs_2035h1aze;		/* Mapped at index 0x2035, subindex 0x01 */
extern UNS16 BrakeSignal_To_Bridge_Timer_usBrakeOffSettleAllowTimeMs_2035h2aze;		/* Mapped at index 0x2035, subindex 0x02 */
extern UNS16 CURRDLYaze;		/* Mapped at index 0x2036, subindex 0x00*/
extern UNS16 usBridgeOn_To_Encoder_SettleTimeMs_2037haze;		/* Mapped at index 0x2037, subindex 0x00*/
extern UNS16 TripOutConfig_2038h_usTripOutCtrl_2038h1aze;		/* Mapped at index 0x2038, subindex 0x01 */
extern INTEGER32 TripOutConfig_2038h_lTripOutPositnFirst_user_2038h2aze;		/* Mapped at index 0x2038, subindex 0x02 */
extern INTEGER32 TripOutConfig_2038h_lTripOutPositnModulo_user_2038h3aze;		/* Mapped at index 0x2038, subindex 0x03 */
extern UNS8 TripOutConfig_2038h_ucTripOutPulseWidth_2038h4aze;		/* Mapped at index 0x2038, subindex 0x04 */
extern UNS8 ucHomingConfig_2098haze;		/* Mapped at index 0x2098, subindex 0x00*/
extern INTEGER32 lIndexOffsetPosition_usteps_2099haze;		/* Mapped at index 0x2099, subindex 0x00*/
extern UNS8 ucRunCurrentPcnt_2204haze;		/* Mapped at index 0x2204, subindex 0x00*/
extern UNS8 ucHoldCurrentPcnt_2205haze;		/* Mapped at index 0x2205, subindex 0x00*/
extern INTEGER32 lPositn_present_point_tgt_user_2211haze;		/* Mapped at index 0x2211, subindex 0x00*/
extern INTEGER32 lPositn_final_point_tgt_user_2212haze;		/* Mapped at index 0x2212, subindex 0x00*/
extern UNS8 PositionMaintain_ucPositionMaintain_Reaction_Code_2221haze;		/* Mapped at index 0x2221, subindex 0x01 */
extern UNS8 ucGeneral_purpose_variable_2401haze;		/* Mapped at index 0x2401, subindex 0x00*/
extern UNS32 Num_Of_Gen_Pur_Var_ulGeneral_purpose_variable_2402h1aze;		/* Mapped at index 0x2402, subindex 0x01 */
extern UNS32 Num_Of_Gen_Pur_Var_ulGeneral_purpose_variable_2402h2aze;		/* Mapped at index 0x2402, subindex 0x02 */
extern INTEGER32 Num_Of_Gen_Pur_Var_lGeneral_purpose_variable_2402h3aze;		/* Mapped at index 0x2402, subindex 0x03 */
extern INTEGER32 Num_Of_Gen_Pur_Var_lGeneral_purpose_variable_2402h4aze;		/* Mapped at index 0x2402, subindex 0x04 */
extern UNS8 ucSemOptions_2504haze;		/* Mapped at index 0x2504, subindex 0x00*/
extern UNS8 ucHybridEn_2701haze;		/* Mapped at index 0x2701, subindex 0x00*/
extern UNS8 ucHybridModes_2702haze;		/* Mapped at index 0x2702, subindex 0x00*/
extern UNS32 ulMakeUp_velocity_2703haze;		/* Mapped at index 0x2703, subindex 0x00*/
extern UNS8 ucTorque_max_velocity_2704haze;		/* Mapped at index 0x2704, subindex 0x00*/
extern UNS16 ASLRTIMEaze;		/* Mapped at index 0x2710, subindex 0x00*/
extern INTEGER16 sLocked_Rotor_opcode_2711haze;		/* Mapped at index 0x2711, subindex 0x00*/
extern INTEGER16 sFollowingError_opcode_2712haze;		/* Mapped at index 0x2712, subindex 0x00*/
extern UNS8 HybridControlByteaze;		/* Mapped at index 0x2740, subindex 0x00*/
extern UNS8 HybridStatusByteaze;		/* Mapped at index 0x2741, subindex 0x00*/
extern UNS8 ucHybrid_StatByte_2742haze;		/* Mapped at index 0x2742, subindex 0x00*/
extern UNS8 ucHybrid_StatByte_Filter_2743haze;		/* Mapped at index 0x2743, subindex 0x00*/
extern UNS32 OptionsSetting_5001h_ulUnitOptions_5001h1aze;		/* Mapped at index 0x5001, subindex 0x01 */
extern UNS8 OptionsSetting_5001h_Number_Of_Writes_Leftaze;		/* Mapped at index 0x5001, subindex 0x02 */
extern UNS8 AsciiSerialNumber_5002h_Full_Serial_Numberaze[10];		/* Mapped at index 0x5002, subindex 0x01 */
extern UNS8 AsciiSerialNumber_5002h_Number_Of_Writes_Leftaze;		/* Mapped at index 0x5002, subindex 0x02 */
extern UNS8 PartNumber_5003h_Full_Part_Numberaze[10];		/* Mapped at index 0x5003, subindex 0x01 */
extern UNS8 PartNumber_5003h_Number_Of_Writes_Leftaze;		/* Mapped at index 0x5003, subindex 0x02 */
extern UNS32 MotorConfigSetting_5004h_ulMotorConfig_5004h1aze;		/* Mapped at index 0x5004, subindex 0x01 */
extern UNS8 MotorConfigSetting_5004h_Number_Of_Writes_Leftaze;		/* Mapped at index 0x5004, subindex 0x02 */
extern UNS8 MotherBd_AsciiSerialNumber_5005h_MotherBd_Serial_Numberaze[10];		/* Mapped at index 0x5005, subindex 0x01 */
extern UNS8 MotherBd_AsciiSerialNumber_5005h_ucWrCountDown_5005h2aze;		/* Mapped at index 0x5005, subindex 0x02 */
extern UNS16 usUnlockCode_5007haze;		/* Mapped at index 0x5007, subindex 0x00*/
extern UNS16 Boot_Version_Boot_Version_Prefixaze;		/* Mapped at index 0x5008, subindex 0x01 */
extern UNS16 Boot_Version_Boot_Version_Suffixaze;		/* Mapped at index 0x5008, subindex 0x02 */
extern UNS16 NVM_FormatVersionaze;		/* Mapped at index 0x5009, subindex 0x00*/
extern UNS8 ucTestOption_5010haze;		/* Mapped at index 0x5010, subindex 0x00*/
extern UNS32 ulBurnInCycleMs_x5011haze;		/* Mapped at index 0x5011, subindex 0x00*/
extern INTEGER32 lBurnInMove_5012haze;		/* Mapped at index 0x5012, subindex 0x00*/
extern UNS32 ulFeedConst_To_EncoderRatio_And_GearRatio_5090haze;		/* Mapped at index 0x5090, subindex 0x00*/
extern UNS8 ucMps160_Spi_5160haze;		/* Mapped at index 0x5160, subindex 0x00*/
extern INTEGER16 sTimken_Map_Quad0_At0_x100_5161haze;		/* Mapped at index 0x5161, subindex 0x00*/
extern UNS8 MpsValidData_ucMpsValidData_5162h1aze;		/* Mapped at index 0x5162, subindex 0x01 */
extern UNS8 MpsValidData_ucMpsValidData_5162h2aze;		/* Mapped at index 0x5162, subindex 0x02 */
extern INTEGER16 MapSpinAvg_ssMapAvg_Neg_5163h1aze;		/* Mapped at index 0x5163, subindex 0x01 */
extern INTEGER16 MapSpinAvg_ssMapAvg_Pos_5163h2aze;		/* Mapped at index 0x5163, subindex 0x02 */
extern INTEGER32 VELCVEL_Snapshotaze;		/* Mapped at index 0x5214, subindex 0x00*/
extern INTEGER32 Signed_VELCVEL_Snapshotaze;		/* Mapped at index 0x5215, subindex 0x00*/
extern INTEGER32 lVelocity_dmand_val_inc_5216haze;		/* Mapped at index 0x5216, subindex 0x00*/
extern INTEGER8 Velocity_contol_regaze;		/* Mapped at index 0x521A, subindex 0x00*/
extern INTEGER32 IDXPOS_Snapshotaze;		/* Mapped at index 0x522C, subindex 0x00*/
extern INTEGER32 lIDXPOS_BridgeSign_intern_522Dhaze;		/* Mapped at index 0x522D, subindex 0x00*/
extern INTEGER32 lIDXENC_BridgeSign_intern_5231haze;		/* Mapped at index 0x5231, subindex 0x00*/
extern INTEGER8 Motor_State_ucState3000aze;		/* Mapped at index 0x5246, subindex 0x00*/
extern UNS16 Enable_Upgrade_Codeaze;		/* Mapped at index 0x5300, subindex 0x00*/
extern UNS32 ReadWriteMemory_5303h_ulMemoryAddress_5303h1aze;		/* Mapped at index 0x5303, subindex 0x01 */
extern UNS8 ReadWriteMemory_5303h_szSharedRAM8_gaze;		/* Mapped at index 0x5303, subindex 0x02 */
extern UNS16 ReadWriteMemory_5303h_szSharedRAM16_gaze;		/* Mapped at index 0x5303, subindex 0x03 */
extern UNS32 ReadWriteMemory_5303h_szSharedRAM32_gaze;		/* Mapped at index 0x5303, subindex 0x04 */
extern INTEGER16 sAbort_connection_opcode_6007haze;		/* Mapped at index 0x6007, subindex 0x00*/
extern UNS16 usErrorCode_603Fhaze;		/* Mapped at index 0x603F, subindex 0x00*/
extern UNS16 Controlwordaze;		/* Mapped at index 0x6040, subindex 0x00*/
extern UNS16 Statuswordaze;		/* Mapped at index 0x6041, subindex 0x00*/
extern INTEGER16 sQuick_stop_opcode_605Ahaze;		/* Mapped at index 0x605A, subindex 0x00*/
extern INTEGER16 sShutdown_opcode_605Bhaze;		/* Mapped at index 0x605B, subindex 0x00*/
extern INTEGER16 sDisable_operation_opcode_605Chaze;		/* Mapped at index 0x605C, subindex 0x00*/
extern INTEGER16 sHalt_opcode_605Dhaze;		/* Mapped at index 0x605D, subindex 0x00*/
extern INTEGER16 sFault_reaction_opcode_605Ehaze;		/* Mapped at index 0x605E, subindex 0x00*/
extern INTEGER8 Modes_of_operationaze;		/* Mapped at index 0x6060, subindex 0x00*/
extern INTEGER8 Modes_of_operation_displayaze;		/* Mapped at index 0x6061, subindex 0x00*/
extern INTEGER32 Position_demannd_valueaze;		/* Mapped at index 0x6062, subindex 0x00*/
extern INTEGER32 Position_actual_value_incaze;		/* Mapped at index 0x6063, subindex 0x00*/
extern INTEGER32 Position_actual_valueaze;		/* Mapped at index 0x6064, subindex 0x00*/
extern UNS32 Maximal_following_erroraze;		/* Mapped at index 0x6065, subindex 0x00*/
extern UNS16 usFollow_error_timeout_ms_6066h0aze;		/* Mapped at index 0x6066, subindex 0x00*/
extern UNS32 Position_windowaze;		/* Mapped at index 0x6067, subindex 0x00*/
extern UNS16 Position_window_timeaze;		/* Mapped at index 0x6068, subindex 0x00*/
extern INTEGER32 Velocity_actual_valueaze;		/* Mapped at index 0x606C, subindex 0x00*/
extern INTEGER16 Target_torque_x1000aze;		/* Mapped at index 0x6071, subindex 0x00*/
extern INTEGER32 Target_positionaze;		/* Mapped at index 0x607A, subindex 0x00*/
extern INTEGER32 Home_offsetaze;		/* Mapped at index 0x607C, subindex 0x00*/
extern UNS8 Polarityaze;		/* Mapped at index 0x607E, subindex 0x00*/
extern UNS32 Profile_velocityaze;		/* Mapped at index 0x6081, subindex 0x00*/
extern UNS32 End_velocityaze;		/* Mapped at index 0x6082, subindex 0x00*/
extern UNS32 Profile_accelerationaze;		/* Mapped at index 0x6083, subindex 0x00*/
extern UNS32 Profile_decelerationaze;		/* Mapped at index 0x6084, subindex 0x00*/
extern UNS32 Quick_stop_decelerationaze;		/* Mapped at index 0x6085, subindex 0x00*/
extern INTEGER16 Motion_profile_typeaze;		/* Mapped at index 0x6086, subindex 0x00*/
extern UNS32 ulTorque_slope_x1000_pSecond_6087haze;		/* Mapped at index 0x6087, subindex 0x00*/
extern UNS32 Factor_Group_Encoder_Resolution_608Fh_ulEncoder_resolution_numerator_encinc_608Fh1aze;		/* Mapped at index 0x608F, subindex 0x01 */
extern UNS32 Factor_Group_Encoder_Resolution_608Fh_ulEncoder_resolution_denominator_motorrevs_608Fh2aze;		/* Mapped at index 0x608F, subindex 0x02 */
extern UNS32 Factor_Group_Feed_and_DriveShaft_ulFeed_numerator_user_6092h1aze;		/* Mapped at index 0x6092, subindex 0x01 */
extern UNS32 Factor_Group_Feed_and_DriveShaft_ulDriveShaft_revolutions_denominator_revs_6092h2aze;		/* Mapped at index 0x6092, subindex 0x02 */
extern INTEGER8 Homing_methodaze;		/* Mapped at index 0x6098, subindex 0x00*/
extern UNS32 Homing_speeds_Speed_for_switch_searchaze;		/* Mapped at index 0x6099, subindex 0x01 */
extern UNS32 Homing_speeds_Speed_for_zero_searchaze;		/* Mapped at index 0x6099, subindex 0x02 */
extern UNS8 Sync_lInterpolationPeriod_60C2h_ucInterpolationPeriod_60C2h1aze;		/* Mapped at index 0x60C2, subindex 0x01 */
extern INTEGER8 Sync_lInterpolationPeriod_60C2h_scInterpolationFactor_60C2h2aze;		/* Mapped at index 0x60C2, subindex 0x02 */
extern INTEGER32 lMaxSlipWindow_user_60F8haze;		/* Mapped at index 0x60F8, subindex 0x00*/
extern UNS32 Digital_inputsaze;		/* Mapped at index 0x60FD, subindex 0x00*/
extern UNS32 number_of_entries_Digital_outputsaze;		/* Mapped at index 0x60FE, subindex 0x01 */
extern INTEGER32 Target_velocityaze;		/* Mapped at index 0x60FF, subindex 0x00*/
extern UNS16 Motor_typeaze;		/* Mapped at index 0x6402, subindex 0x00*/
extern UNS32 Supported_drive_modesaze;		/* Mapped at index 0x6502, subindex 0x00*/

#endif // MOTEUR_TRANSLATION_2_H