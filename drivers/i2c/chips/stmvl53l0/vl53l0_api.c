/*******************************************************************************
 Copyright ?2015, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "inc/vl53l0_api.h"
#include "inc/vl53l0_tuning.h"
#include <linux/delay.h>
#include <linux/kernel.h>


#ifndef ABS
#define ABS(x)              (((x) > 0) ? (x) : (-(x)))
#endif


#define VL53L0_SETPARAMETERFIELD(Dev, field, value) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        CurrentParameters.field = value; \
        CurrentParameters = PALDevDataSet(Dev, CurrentParameters, CurrentParameters); }
#define VL53L0_GETPARAMETERFIELD(Dev, field, variable) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        variable = CurrentParameters.field; }
#define VL53L0_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        CurrentParameters.field[index] = value; \
        CurrentParameters = PALDevDataSet(Dev, CurrentParameters, CurrentParameters); }
#define VL53L0_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
    if(Status==VL53L0_ERROR_NONE){ \
        CurrentParameters = PALDevDataGet(Dev, CurrentParameters); \
        variable = CurrentParameters.field[index]; }
#define VL53L0_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
	if(Status==VL53L0_ERROR_NONE){ \
		DeviceSpecificParameters = PALDevDataGet(Dev, DeviceSpecificParameters); \
		DeviceSpecificParameters.field = value; \
		DeviceSpecificParameters = PALDevDataSet(Dev, DeviceSpecificParameters, DeviceSpecificParameters); }
#define VL53L0_GETDEVICESPECIFICPARAMETER(Dev, field) \
		PALDevDataGet(Dev, DeviceSpecificParameters).field

#define VL53L0_FIXPOINT1616TOFIXPOINT97(Value) (uint16_t)((Value>>9)&0xFFFF)
#define VL53L0_FIXPOINT97TOFIXPOINT1616(Value) (FixPoint1616_t)(Value<<9)

#define VL53L0_FIXPOINT1616TOFIXPOINT412(Value) (uint16_t)((Value>>4)&0xFFFF)
#define VL53L0_FIXPOINT412TOFIXPOINT1616(Value) (FixPoint1616_t)(Value<<4)

#define VL53L0_FIXPOINT1616TOFIXPOINT313(Value) (uint16_t)((Value>>3)&0xFFFF)
#define VL53L0_FIXPOINT313TOFIXPOINT1616(Value) (FixPoint1616_t)(Value<<3)

#define VL53L0_FIXPOINT1616TOFIXPOINT08(Value) (uint8_t)((Value>>8)&0x00FF)
#define VL53L0_FIXPOINT08TOFIXPOINT1616(Value) (FixPoint1616_t)(Value<<8)

#define VL53L0_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + (uint16_t)lsb)

VL53L0_Error VL53L0_get_vcsel_pulse_period(VL53L0_DEV Dev, uint8_t* pVCSELPulsePeriod, uint8_t RangeIndex);
uint8_t VL53L0_encode_vcsel_period(uint8_t vcsel_period_pclks);
uint8_t VL53L0_decode_vcsel_period(uint8_t vcsel_period_reg);
uint32_t VL53L0_calc_ranging_wait_us(VL53L0_DEV Dev, uint16_t timeout_overall_periods, uint8_t vcsel_period);
VL53L0_Error VL53L0_check_part_used(VL53L0_DEV Dev, uint8_t* ModuleId, uint8_t* Revision, VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo);
VL53L0_Error VL53L0_get_info_from_device(VL53L0_DEV Dev, uint8_t* ModuleId, uint8_t* Revision, VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo);
VL53L0_Error VL53L0_device_read_strobe(VL53L0_DEV Dev);
VL53L0_Error VL53L0_get_pal_range_status(VL53L0_DEV Dev,
                                         uint8_t DeviceRangeStatus,
                                         FixPoint1616_t SignalRate,
                                         FixPoint1616_t CrosstalkCompensation,
                                         uint16_t EffectiveSpadRtnCount,
                                         VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
                                         uint8_t* pPalRangeStatus);
VL53L0_Error VL53L0_measurement_poll_for_completion(VL53L0_DEV Dev);
VL53L0_Error VL53L0_load_tuning_settings(VL53L0_DEV Dev,  uint8_t* pTuningSettingBuffer);

uint32_t VL53L0_calc_macro_period_ps(VL53L0_DEV Dev, uint8_t vcsel_period);
uint32_t VL53L0_decode_timeout(uint16_t encoded_timeout);


VL53L0_Error VL53L0_SetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev, int32_t OffsetCalibrationDataMicroMeter) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int16_t cMaxOffset = 511;
    int16_t cMinOffset = -512;
    int16_t cOffsetRange = 1024;
    int16_t OffsetCalibrationDataMilliMeter = (OffsetCalibrationDataMicroMeter + 500)/1000;

    if(OffsetCalibrationDataMilliMeter > cMaxOffset)
    {
        OffsetCalibrationDataMilliMeter = cMaxOffset;
    }
    else if(OffsetCalibrationDataMilliMeter < cMinOffset)
    {
        OffsetCalibrationDataMilliMeter = cMinOffset;
    }

    if(OffsetCalibrationDataMilliMeter < 0)
    {
        
        OffsetCalibrationDataMilliMeter += cOffsetRange;
    }

    Status = VL53L0_WrWord(Dev, VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)(OffsetCalibrationDataMilliMeter << 2));

    return Status;
}

VL53L0_Error VL53L0_GetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev, int32_t * pOffsetCalibrationDataMicroMeter) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t RangeOffsetRegister;
    int16_t cMaxOffset = 511;
    int16_t cOffsetRange = 1024;


    Status = VL53L0_RdWord(Dev, VL53L0_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, &RangeOffsetRegister);

    if (Status == VL53L0_ERROR_NONE) {

        RangeOffsetRegister = (RangeOffsetRegister + 0x02) >> 2;

        
        if(RangeOffsetRegister > cMaxOffset)
        {
            *pOffsetCalibrationDataMicroMeter = (int16_t)(RangeOffsetRegister - cOffsetRange) * 1000;
        }
        else
        {
            *pOffsetCalibrationDataMicroMeter = (int16_t)RangeOffsetRegister * 1000;
        }
    }

    return Status;
}

VL53L0_Error VL53L0_DataInit(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    int32_t OffsetCalibrationData;
    int i;

#ifdef USE_I2C_2V8
    Status = VL53L0_UpdateByte(Dev, VL53L0_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0xFE, 0x01);
#endif

    VL53L0_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz, 748421);

    
    Status = VL53L0_GetDeviceParameters(Dev, &CurrentParameters);
    if (Status == VL53L0_ERROR_NONE) {
        
        CurrentParameters.DeviceMode = VL53L0_DEVICEMODE_SINGLE_RANGING;
        CurrentParameters.HistogramMode = VL53L0_HISTOGRAMMODE_DISABLED;
        PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
    }

    
    PALDevDataSet(Dev, SigmaEstRefArray, 100);
    PALDevDataSet(Dev, SigmaEstEffPulseWidth, 900);
    PALDevDataSet(Dev, SigmaEstEffAmbWidth, 500);
    PALDevDataSet(Dev, targetRefRate, 0x0A00); 

    
    PALDevDataSet(Dev, UseInternalTuningSettings, 1);

    

    for (i = 0; i < VL53L0_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
    if (Status == VL53L0_ERROR_NONE) {
            Status |= VL53L0_SetLimitCheckEnable(Dev, i, 1);
        } else {
            break;
    }
    }

    
    if (Status == VL53L0_ERROR_NONE)
    	Status = VL53L0_SetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, 0);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(32<<16));
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    			(FixPoint1616_t)(25 * 65536 / 100));
    			
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetLimitCheckValue(Dev,
        		VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
    			(FixPoint1616_t)(35 * 65536));
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetOffsetCalibrationDataMicroMeter(Dev,
        		&OffsetCalibrationData);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters,
        		OffsetCalibrationData)
        PALDevDataSet(Dev, Part2PartOffsetNVMMicroMeter,
        		OffsetCalibrationData);

        PALDevDataSet(Dev, SequenceConfig, 0xFF);

        PALDevDataSet(Dev, PalState, VL53L0_STATE_WAIT_STATICINIT);
    }

    return Status;
}

VL53L0_Error VL53L0_StaticInit(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t* pTuningSettingBuffer;
    uint16_t tempword;
    uint8_t  tempbyte;
    uint8_t  UseInternalTuningSettings;



    
    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0x88, 0x00);


    UseInternalTuningSettings = PALDevDataGet(Dev, UseInternalTuningSettings);

    if (UseInternalTuningSettings == 0) {
        pTuningSettingBuffer = PALDevDataGet(Dev, pTuningSettingsPointer);
    } else {
        pTuningSettingBuffer = DefaultTuningSettings;
    }

    if (Status == VL53L0_ERROR_NONE)
    	Status = VL53L0_load_tuning_settings(Dev, pTuningSettingBuffer);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, 0x80, 0x00);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetGpioConfig(Dev, 0, 0,
                VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
                VL53L0_INTERRUPTPOLARITY_LOW);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, 0xFF, 0x01);
        Status |= VL53L0_RdWord(Dev, 0x84, &tempword);
        Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz,
                VL53L0_FIXPOINT412TOFIXPOINT1616(tempword));
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetDeviceParameters(Dev, &CurrentParameters);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_RANGE_CONFIG, &tempbyte);
        PALDevDataSet(Dev, RangeFractionalEnable, tempbyte);
    }

    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, CurrentParameters, CurrentParameters);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev,
                VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
        if (Status == VL53L0_ERROR_NONE) {
            PALDevDataSet(Dev, SequenceConfig, tempbyte);
    	}
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_PerformRefCalibration(Dev);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }

    return Status;
}

VL53L0_Error VL53L0_GetDeviceParameters(VL53L0_DEV Dev, VL53L0_DeviceParameters_t* pDeviceParameters) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;


    Status = VL53L0_GetDeviceMode(Dev, &(pDeviceParameters->DeviceMode));

    if(Status==VL53L0_ERROR_NONE){
       Status = VL53L0_GetHistogramMode(Dev,
               &(pDeviceParameters->HistogramMode));
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetInterMeasurementPeriodMilliSeconds(Dev,
                &(pDeviceParameters->InterMeasurementPeriodMilliSeconds));
    }
    if (Status == VL53L0_ERROR_NONE) {
        pDeviceParameters->XTalkCompensationEnable = 0;
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetXTalkCompensationRateMegaCps(Dev,
                &(pDeviceParameters->XTalkCompensationRateMegaCps));
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetOffsetCalibrationDataMicroMeter(Dev,
                &(pDeviceParameters->RangeOffsetMicroMeters));
    }

    if (Status == VL53L0_ERROR_NONE) {
        for (i = 0; i < VL53L0_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
            
            
            if (Status == VL53L0_ERROR_NONE) {
                Status |= VL53L0_GetLimitCheckValue(Dev, i,
                        &(pDeviceParameters->LimitChecksValue[i]));
            } else {
                break;
            }
            if (Status == VL53L0_ERROR_NONE) {
                Status |= VL53L0_GetLimitCheckEnable(Dev, i,
                        &(pDeviceParameters->LimitChecksEnable[i]));
            } else {
                break;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetWrapAroundCheckEnable(Dev,
                &(pDeviceParameters->WrapAroundCheckEnable));
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetMeasurementTimingBudgetMicroSeconds(Dev,
                &(pDeviceParameters->MeasurementTimingBudgetMicroSeconds));
    }

    return Status;
}

VL53L0_Error VL53L0_SetDeviceMode(VL53L0_DEV Dev, VL53L0_DeviceModes DeviceMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;


    switch (DeviceMode) {
    case VL53L0_DEVICEMODE_SINGLE_RANGING:
    case VL53L0_DEVICEMODE_CONTINUOUS_RANGING:
    case VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
    case VL53L0_DEVICEMODE_SINGLE_HISTOGRAM:
    case VL53L0_DEVICEMODE_GPIO_DRIVE:
    case VL53L0_DEVICEMODE_GPIO_OSC:
        
        VL53L0_SETPARAMETERFIELD(Dev, DeviceMode, DeviceMode);
        break;
    default:
        
        Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    return Status;
}

VL53L0_Error VL53L0_GetDeviceMode(VL53L0_DEV Dev, VL53L0_DeviceModes* pDeviceMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;

    VL53L0_GETPARAMETERFIELD(Dev, DeviceMode, *pDeviceMode);

    return Status;
}

VL53L0_Error VL53L0_GetHistogramMode(VL53L0_DEV Dev, VL53L0_HistogramModes* pHistogramMode) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;

    VL53L0_GETPARAMETERFIELD(Dev, HistogramMode, *pHistogramMode);

    return Status;
}

VL53L0_Error VL53L0_GetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev, uint32_t* pMeasurementTimingBudgetMicroSeconds) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t CurrentVCSELPulsePeriod;
    uint8_t CurrentVCSELPulsePeriodPClk;
    uint16_t encodedTimeOut;
    uint32_t FinalRangeTimeoutMicroSeconds;
    uint32_t PreRangeTimeoutMicroSeconds = 8000;
    uint32_t DccAndTccTimeoutMicroSeconds = 6000;
    uint32_t AdditionalOverheadsMicroSeconds = 5360;
    uint32_t TotalAdditionalTimingMicroSeconds = PreRangeTimeoutMicroSeconds +
    		DccAndTccTimeoutMicroSeconds + AdditionalOverheadsMicroSeconds;


    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_get_vcsel_pulse_period(Dev, &CurrentVCSELPulsePeriodPClk, 1);
        CurrentVCSELPulsePeriod =
        		VL53L0_encode_vcsel_period(CurrentVCSELPulsePeriodPClk);

        
        Status = VL53L0_RdWord(Dev,
        		VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        		&encodedTimeOut);
        if (Status == VL53L0_ERROR_NONE) {
            FinalRangeTimeoutMicroSeconds = VL53L0_calc_ranging_wait_us(Dev,
            		encodedTimeOut, CurrentVCSELPulsePeriod);

            *pMeasurementTimingBudgetMicroSeconds =
            		FinalRangeTimeoutMicroSeconds +
            		TotalAdditionalTimingMicroSeconds;
            VL53L0_SETPARAMETERFIELD(Dev, MeasurementTimingBudgetMicroSeconds,
            		*pMeasurementTimingBudgetMicroSeconds);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetInterMeasurementPeriodMilliSeconds(VL53L0_DEV Dev, uint32_t* pInterMeasurementPeriodMilliSeconds) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint16_t osc_calibrate_val;
    uint32_t IMPeriodMilliSeconds;

    Status = VL53L0_RdWord(Dev, VL53L0_REG_OSC_CALIBRATE_VAL, &osc_calibrate_val);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdDWord(Dev, VL53L0_REG_SYSTEM_INTERMEASUREMENT_PERIOD, &IMPeriodMilliSeconds);
    }

    if (Status == VL53L0_ERROR_NONE) {
        if (osc_calibrate_val !=0) {
            *pInterMeasurementPeriodMilliSeconds = IMPeriodMilliSeconds / osc_calibrate_val;
        }
        VL53L0_SETPARAMETERFIELD(Dev, InterMeasurementPeriodMilliSeconds, *pInterMeasurementPeriodMilliSeconds);
    }

    return Status;
}

VL53L0_Error VL53L0_SetXTalkCompensationEnable(VL53L0_DEV Dev, uint8_t XTalkCompensationEnable) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t TempFix1616;


    if (XTalkCompensationEnable == 0) {
        TempFix1616 = 0;
    } else {
        VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
        		TempFix1616);
    }

    
    Status = VL53L0_WrWord(Dev, VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, VL53L0_FIXPOINT1616TOFIXPOINT313(TempFix1616));

    if (Status == VL53L0_ERROR_NONE) {
        if (XTalkCompensationEnable == 0) {
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 0);
        } else {
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 1);
    }
    }

    return Status;
}

VL53L0_Error VL53L0_GetXTalkCompensationEnable(VL53L0_DEV Dev, uint8_t* pXTalkCompensationEnable) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;

    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);
    *pXTalkCompensationEnable = Temp8;

    return Status;
}

VL53L0_Error VL53L0_SetXTalkCompensationRateMegaCps(VL53L0_DEV Dev, FixPoint1616_t XTalkCompensationRateMegaCps) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;

    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationEnable, Temp8);

    if (Temp8 == 0) { 
        VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    } else {
        
        Status = VL53L0_WrWord(Dev, VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, VL53L0_FIXPOINT1616TOFIXPOINT313(XTalkCompensationRateMegaCps));
    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps, XTalkCompensationRateMegaCps);
    }
    }

    return Status;
}

VL53L0_Error VL53L0_GetXTalkCompensationRateMegaCps(VL53L0_DEV Dev, FixPoint1616_t* pXTalkCompensationRateMegaCps) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t Value;
    FixPoint1616_t TempFix1616 = 0;
    VL53L0_DeviceParameters_t CurrentParameters;


    Status = VL53L0_RdWord(Dev,
    		VL53L0_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS,
    		(uint16_t*) &Value);
    if (Status == VL53L0_ERROR_NONE) {
        if (Value == 0) { 
            VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            		TempFix1616);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 0);
        } else {
            TempFix1616 = VL53L0_FIXPOINT313TOFIXPOINT1616(Value);
            *pXTalkCompensationRateMegaCps = TempFix1616;
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
            		TempFix1616);
            VL53L0_SETPARAMETERFIELD(Dev, XTalkCompensationEnable, 1);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_SetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId,
		uint8_t LimitCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t TempFix1616 = 0;
    uint8_t LimitCheckEnableInt;


    if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    } else {
        if (LimitCheckEnable == 0) {
            TempFix1616 = 0;
            LimitCheckEnableInt = 0;
        } else {
            VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
            		LimitCheckId, TempFix1616);
            
            LimitCheckEnableInt = 1;
        }

        switch (LimitCheckId) {

		case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckEnableInt);

			break;

		case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

			Status = VL53L0_WrWord(Dev,
					VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
					VL53L0_FIXPOINT1616TOFIXPOINT97(TempFix1616));

			break;

		case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckEnableInt);

			break;

		default:
			Status = VL53L0_ERROR_INVALID_PARAMS;

        }

    }

    if (Status == VL53L0_ERROR_NONE) {
        if (LimitCheckEnable == 0) {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
            		LimitCheckId, 0);
        } else {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
            		LimitCheckId, 1);
    }
    }

    return Status;
}

VL53L0_Error VL53L0_GetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId, uint8_t *pLimitCheckEnable)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8 = 0;


    if (LimitCheckId >= VL53L0_CHECKENABLE_NUMBER_OF_CHECKS) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, Temp8);
    *pLimitCheckEnable = Temp8;

    return Status;
}

VL53L0_Error VL53L0_SetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t Temp8;


    VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable, LimitCheckId, Temp8);

    if (Temp8 == 0) { 
        VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId,
        		LimitCheckValue);
    } else {

        switch (LimitCheckId) {

		case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, LimitCheckValue);
			break;

		case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

			Status = VL53L0_WrWord(Dev,
					VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
					VL53L0_FIXPOINT1616TOFIXPOINT97(LimitCheckValue));

			break;

		case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:

			
			VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, LimitCheckValue);

			break;

		default:
			Status = VL53L0_ERROR_INVALID_PARAMS;

    }

    if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue, LimitCheckId,
            		LimitCheckValue);
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    uint8_t EnableZeroValue = 0;
    uint16_t Temp16;
    FixPoint1616_t TempFix1616;


    switch (LimitCheckId) {

	case VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE:
		
		VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, TempFix1616);
		EnableZeroValue = 0;
		break;

	case VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
		Status = VL53L0_RdWord(Dev,
				VL53L0_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
				&Temp16);
		if (Status == VL53L0_ERROR_NONE) {
			TempFix1616 = VL53L0_FIXPOINT97TOFIXPOINT1616(Temp16);
		}

		EnableZeroValue = 1;
		break;

	case VL53L0_CHECKENABLE_SIGNAL_REF_CLIP:
		
		VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
				VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, TempFix1616);
		EnableZeroValue = 0;
		break;

	default:
		Status = VL53L0_ERROR_INVALID_PARAMS;

    }

    if (Status == VL53L0_ERROR_NONE) {

        if (EnableZeroValue == 1) {

            if (TempFix1616 == 0) { 
                VL53L0_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                		LimitCheckId, TempFix1616);
                *pLimitCheckValue = TempFix1616;
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                		LimitCheckId, 0);
            } else {
                *pLimitCheckValue = TempFix1616;
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                		LimitCheckId, TempFix1616);
                VL53L0_SETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                		LimitCheckId, 1);
            }
        } else {
            *pLimitCheckValue = TempFix1616;
        }
    }

    return Status;

}

VL53L0_Error VL53L0_GetWrapAroundCheckEnable(VL53L0_DEV Dev, uint8_t* pWrapAroundCheckEnable) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t data;
    VL53L0_DeviceParameters_t CurrentParameters;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, &data);
    if (Status == VL53L0_ERROR_NONE) {
        PALDevDataSet(Dev, SequenceConfig, data);
        if (data & (0x01 << 7)) {
            *pWrapAroundCheckEnable = 0x01;
        } else {
            *pWrapAroundCheckEnable = 0x00;
        }
    }
    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SETPARAMETERFIELD(Dev, WrapAroundCheckEnable, *pWrapAroundCheckEnable);
    }

    return Status;
}

VL53L0_Error VL53L0_PerformSingleMeasurement(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceModes DeviceMode;


    
    Status = VL53L0_GetDeviceMode(Dev, &DeviceMode);

    if (Status     == VL53L0_ERROR_NONE &&
        DeviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING)
    {
        Status = VL53L0_StartMeasurement(Dev);
    }

    if (Status == VL53L0_ERROR_NONE) {
        msleep(30);
        Status = VL53L0_measurement_poll_for_completion(Dev);
    }

    
    if (Status     == VL53L0_ERROR_NONE &&
        DeviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING)
    {
        PALDevDataSet(Dev, PalState, VL53L0_STATE_IDLE);
    }

    return Status;
}

VL53L0_Error VL53L0_measurement_poll_for_completion(VL53L0_DEV Dev){
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t NewDataReady=0;
    uint32_t LoopNb;


    LoopNb = 0;
    do {
        Status = VL53L0_GetMeasurementDataReady(Dev, &NewDataReady);
        msleep(1);
        LoopNb++;
    } while((NewDataReady == 0 ) &&
            (Status == VL53L0_ERROR_NONE) &&
            (LoopNb < VL53L0_DEFAULT_MAX_LOOP));


    return Status;
}

VL53L0_Error VL53L0_perform_single_ref_calibration(VL53L0_DEV Dev)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte = 0;
    uint32_t LoopNb;


    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START,
        		VL53L0_REG_SYSRANGE_MODE_START_STOP);
    }

    if (Status == VL53L0_ERROR_NONE) {
        
        LoopNb = 0;
        do {
            if (LoopNb > 0)
                Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSRANGE_START, &Byte);
            LoopNb = LoopNb + 1;
        } while (((Byte & VL53L0_REG_SYSRANGE_MODE_START_STOP) ==
        		VL53L0_REG_SYSRANGE_MODE_START_STOP) &&
                  (Status == VL53L0_ERROR_NONE) &&
                  (LoopNb < VL53L0_DEFAULT_MAX_LOOP));
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_measurement_poll_for_completion(Dev);
    }
    if(Status==VL53L0_ERROR_NONE){
        Status = VL53L0_ClearInterruptMask(Dev, 0);
    }

    return Status;
}


VL53L0_Error VL53L0_PerformRefCalibration(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SequenceConfig = 0;



    SequenceConfig = PALDevDataGet(Dev, SequenceConfig);

    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG, 0x03);

    
    PALDevDataSet(Dev, SequenceConfig, 0x01);

    if(Status==VL53L0_ERROR_NONE)
    	Status = VL53L0_perform_single_ref_calibration(Dev);

    
    PALDevDataSet(Dev, SequenceConfig, 0x02);

    if(Status==VL53L0_ERROR_NONE)
    	Status = VL53L0_perform_single_ref_calibration(Dev);

    if(Status==VL53L0_ERROR_NONE){
        
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_SEQUENCE_CONFIG,
        		SequenceConfig);
		if(Status==VL53L0_ERROR_NONE){
			PALDevDataSet(Dev, SequenceConfig, SequenceConfig);
		}
    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_PerformXTalkCalibration(VL53L0_DEV Dev, FixPoint1616_t XTalkCalDistance, FixPoint1616_t* pXTalkCompensationRateMegaCps) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t sum_ranging = 0;
    uint16_t sum_spads =0;
    FixPoint1616_t sum_signalRate = 0;
    FixPoint1616_t total_count = 0;
    uint8_t xtalk_meas = 0;
    VL53L0_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t xTalkStoredMeanSignalRate;
    FixPoint1616_t xTalkStoredMeanRange;
    FixPoint1616_t xTalkStoredMeanRtnSpads;
    uint32_t signalXTalkTotalPerSpad;
    uint32_t xTalkStoredMeanRtnSpadsAsInt;
    uint32_t xTalkCalDistanceAsInt;
    FixPoint1616_t XTalkCompensationRateMegaCps;

    if (XTalkCalDistance<=0) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_SetXTalkCompensationEnable(Dev, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        sum_ranging = 0;
        sum_spads =0;
        sum_signalRate = 0;
        total_count = 0;
        for(xtalk_meas=0;xtalk_meas<50;xtalk_meas++)
        {
            Status = VL53L0_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);

            if (Status != VL53L0_ERROR_NONE) {
                break;
            }

            
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging + RangingMeasurementData.RangeMilliMeter;
                sum_signalRate = sum_signalRate + RangingMeasurementData.SignalRateRtnMegaCps;
                sum_spads = sum_spads + RangingMeasurementData.EffectiveSpadRtnCount/16;
                total_count = total_count + 1;
            }
        }

        if (total_count == 0) {
            
            Status = VL53L0_ERROR_RANGE_ERROR;
        }
    }


    if (Status == VL53L0_ERROR_NONE) {
        
        xTalkStoredMeanSignalRate = sum_signalRate / total_count;
        xTalkStoredMeanRange = (FixPoint1616_t)((uint32_t)(sum_ranging<<16) / total_count);
        xTalkStoredMeanRtnSpads = (FixPoint1616_t)((uint32_t)(sum_spads<<16) / total_count);

        
        
        
        
        
        
        xTalkStoredMeanRtnSpadsAsInt = (xTalkStoredMeanRtnSpads + 0x8000) >> 16;

        
        
        xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

        if(xTalkStoredMeanRtnSpadsAsInt == 0 || xTalkCalDistanceAsInt == 0 || xTalkStoredMeanRange >= XTalkCalDistance)
        {
            XTalkCompensationRateMegaCps = 0;
        }
        else
        {
            
            
            xTalkCalDistanceAsInt = (XTalkCalDistance + 0x8000) >> 16;

            
            
            
            signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate)/xTalkStoredMeanRtnSpadsAsInt;

            
            
            signalXTalkTotalPerSpad *= ((1<<16) - (xTalkStoredMeanRange/xTalkCalDistanceAsInt));

            
            XTalkCompensationRateMegaCps = (signalXTalkTotalPerSpad + 0x8000) >> 16;
        }

        *pXTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;

        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_SetXTalkCompensationEnable(Dev, 1);
        }

        
        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_SetXTalkCompensationRateMegaCps(Dev, XTalkCompensationRateMegaCps);
        }

    }

    return Status;
}

VL53L0_API VL53L0_Error VL53L0_PerformOffsetCalibration(VL53L0_DEV Dev, uint16_t CalDistanceMilliMeter, int32_t* pOffsetMicroMeter) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t sum_ranging = 0;
    FixPoint1616_t total_count = 0;
    VL53L0_RangingMeasurementData_t RangingMeasurementData;
    FixPoint1616_t StoredMeanRange;
    uint32_t StoredMeanRangeAsInt;
    VL53L0_DeviceParameters_t CurrentParameters;
    int meas = 0;

    if (CalDistanceMilliMeter<=0) {
        Status = VL53L0_ERROR_INVALID_PARAMS;
    }

    if (Status == VL53L0_ERROR_NONE) {
        VL53L0_SetOffsetCalibrationDataMicroMeter(Dev, 0);
    }

    
    if (Status == VL53L0_ERROR_NONE) {
        sum_ranging = 0;
        total_count = 0;
        for(meas=0;meas<50;meas++)
        {
            Status = VL53L0_PerformSingleRangingMeasurement(Dev, &RangingMeasurementData);

            if (Status != VL53L0_ERROR_NONE) {
                break;
            }

            
            if (RangingMeasurementData.RangeStatus == 0) {
                sum_ranging = sum_ranging + RangingMeasurementData.RangeMilliMeter;
                total_count = total_count + 1;
            }
        }

        if (total_count == 0) {
            
            Status = VL53L0_ERROR_RANGE_ERROR;
        }
    }


    if (Status == VL53L0_ERROR_NONE) {
        
        StoredMeanRange = (FixPoint1616_t)((uint32_t)(sum_ranging<<16) / total_count);

        StoredMeanRangeAsInt = (StoredMeanRange + 0x8000) >> 16;

        *pOffsetMicroMeter = (CalDistanceMilliMeter - StoredMeanRangeAsInt) * 1000;

        
        if (Status == VL53L0_ERROR_NONE) {
            VL53L0_SETPARAMETERFIELD(Dev, RangeOffsetMicroMeters, *pOffsetMicroMeter)
            Status = VL53L0_SetOffsetCalibrationDataMicroMeter(Dev, *pOffsetMicroMeter);
        }

    }

    return Status;
}

VL53L0_Error VL53L0_StartMeasurement(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceModes DeviceMode;
    uint8_t Byte = 0;
    uint32_t LoopNb;

    
    VL53L0_GetDeviceMode(Dev, &DeviceMode);

    switch (DeviceMode)
    {
    case VL53L0_DEVICEMODE_SINGLE_RANGING:
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START, VL53L0_REG_SYSRANGE_MODE_SINGLESHOT | VL53L0_REG_SYSRANGE_MODE_START_STOP);
        break;
    case VL53L0_DEVICEMODE_CONTINUOUS_RANGING:
        
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START, VL53L0_REG_SYSRANGE_MODE_BACKTOBACK | VL53L0_REG_SYSRANGE_MODE_START_STOP);
        if (Status == VL53L0_ERROR_NONE) {
            
            PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
        }
        break;
    case VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
        
        Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSRANGE_START, VL53L0_REG_SYSRANGE_MODE_TIMED | VL53L0_REG_SYSRANGE_MODE_START_STOP);
        if (Status == VL53L0_ERROR_NONE) {
            
            PALDevDataSet(Dev, PalState, VL53L0_STATE_RUNNING);
        }
        break;
    default:
        
        Status = VL53L0_ERROR_MODE_NOT_SUPPORTED;
    }

    Byte = VL53L0_REG_SYSRANGE_MODE_START_STOP;
    if (Status == VL53L0_ERROR_NONE) {
        
        LoopNb = 0;
        do {
            if (LoopNb > 0)
                Status = VL53L0_RdByte(Dev, VL53L0_REG_SYSRANGE_START, &Byte);
            LoopNb = LoopNb + 1;
        } while (((Byte & VL53L0_REG_SYSRANGE_MODE_START_STOP) == VL53L0_REG_SYSRANGE_MODE_START_STOP) &&
                  (Status == VL53L0_ERROR_NONE) &&
                  (LoopNb < VL53L0_DEFAULT_MAX_LOOP));

        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
            Status = VL53L0_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetMeasurementDataReady(VL53L0_DEV Dev, uint8_t *pMeasurementDataReady) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t SysRangeStatusRegister;
    uint8_t InterruptConfig;
    uint32_t InterruptMask;

    InterruptConfig = VL53L0_GETDEVICESPECIFICPARAMETER(Dev, Pin0GpioFunctionality);

    if (InterruptConfig == VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
        Status = VL53L0_GetInterruptMaskStatus(Dev, &InterruptMask);
        if (Status == VL53L0_ERROR_NONE) {
            if (InterruptMask == VL53L0_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
                *pMeasurementDataReady = 1;
            } else {
                *pMeasurementDataReady = 0;
            }
        }
    } else {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_RANGE_STATUS, &SysRangeStatusRegister);
        if (Status == VL53L0_ERROR_NONE) {
            if (SysRangeStatusRegister & 0x01) {
                *pMeasurementDataReady = 1;
            } else {
                *pMeasurementDataReady = 0;
            }
        }
    }

    return Status;
}

VL53L0_Error VL53L0_GetRangingMeasurementData(VL53L0_DEV Dev, VL53L0_RangingMeasurementData_t* pRangingMeasurementData) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t DeviceRangeStatus;
    uint8_t RangeFractionalEnable;
    uint8_t PalRangeStatus;
    uint16_t AmbientRate;
    FixPoint1616_t SignalRate;
    uint16_t CrosstalkCompensation;
    uint16_t EffectiveSpadRtnCount;
    uint16_t tmpuint16;
    uint8_t localBuffer[14];
    uint32_t localDWordData;
    uint16_t localWordData;
    VL53L0_RangingMeasurementData_t LastRangeDataBuffer;

    
    
    
    Status = VL53L0_RdDWord(Dev, 0x14, &localDWordData);
    if (Status == VL53L0_ERROR_NONE) {
        localBuffer[0] = (localDWordData >> 24) & 0xFF;
        localBuffer[1] = (localDWordData >> 16) & 0xFF;
        localBuffer[2] = (localDWordData >> 8)  & 0xFF;
        localBuffer[3] = (localDWordData)       & 0xFF;
        Status = VL53L0_RdDWord(Dev, 0x18, &localDWordData);
        if (Status == VL53L0_ERROR_NONE) {
            localBuffer[4] = (localDWordData >> 24) & 0xFF;
            localBuffer[5] = (localDWordData >> 16) & 0xFF;
            localBuffer[6] = (localDWordData >> 8)  & 0xFF;
            localBuffer[7] = (localDWordData)       & 0xFF;
            Status = VL53L0_RdDWord(Dev, 0x1C, &localDWordData);
            if (Status == VL53L0_ERROR_NONE) {
                localBuffer[8]  = (localDWordData >> 24) & 0xFF;
                localBuffer[9]  = (localDWordData >> 16) & 0xFF;
                localBuffer[10] = (localDWordData >> 8)  & 0xFF;
                localBuffer[11] = (localDWordData)       & 0xFF;
                Status = VL53L0_RdWord(Dev, 0x20, &localWordData);
                if (Status == VL53L0_ERROR_NONE) {
                    localBuffer[12] = (localWordData >> 8)   & 0xFF;
                    localBuffer[13] = (localWordData)        & 0xFF;
                }
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE) {

        pRangingMeasurementData->ZoneId = 0; 
        pRangingMeasurementData->TimeStamp = 0; 

        tmpuint16 = VL53L0_MAKEUINT16(localBuffer[11], localBuffer[10]);
        
        
        RangeFractionalEnable = PALDevDataGet(Dev, RangeFractionalEnable);

        if (RangeFractionalEnable) {
            pRangingMeasurementData->RangeMilliMeter = (uint16_t)((tmpuint16)>>2);
            pRangingMeasurementData->RangeFractionalPart = (uint8_t)((tmpuint16 & 0x03)<<6);
        } else {
            pRangingMeasurementData->RangeMilliMeter = tmpuint16;
            pRangingMeasurementData->RangeFractionalPart = 0;
        }


        pRangingMeasurementData->RangeDMaxMilliMeter = 0;
        pRangingMeasurementData->MeasurementTimeUsec = 0;

        SignalRate = VL53L0_FIXPOINT97TOFIXPOINT1616(VL53L0_MAKEUINT16(localBuffer[7], localBuffer[6]));
        pRangingMeasurementData->SignalRateRtnMegaCps = SignalRate; 

        AmbientRate = VL53L0_MAKEUINT16(localBuffer[9], localBuffer[8]);
        pRangingMeasurementData->AmbientRateRtnMegaCps = VL53L0_FIXPOINT97TOFIXPOINT1616(AmbientRate);

        EffectiveSpadRtnCount = VL53L0_MAKEUINT16(localBuffer[3], localBuffer[2]);
        pRangingMeasurementData->EffectiveSpadRtnCount = EffectiveSpadRtnCount; 

        DeviceRangeStatus = localBuffer[0];

        
        CrosstalkCompensation = VL53L0_MAKEUINT16(localBuffer[13], localBuffer[12]);

        Status = VL53L0_get_pal_range_status(Dev, DeviceRangeStatus, SignalRate, CrosstalkCompensation, EffectiveSpadRtnCount, pRangingMeasurementData, &PalRangeStatus);

        if (Status == VL53L0_ERROR_NONE) {
            pRangingMeasurementData->RangeStatus = PalRangeStatus;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        
        LastRangeDataBuffer = PALDevDataGet(Dev, LastRangeMeasure);

        LastRangeDataBuffer.RangeMilliMeter = pRangingMeasurementData->RangeMilliMeter;
        LastRangeDataBuffer.RangeFractionalPart = pRangingMeasurementData->RangeFractionalPart;
        LastRangeDataBuffer.RangeDMaxMilliMeter = pRangingMeasurementData->RangeDMaxMilliMeter;
        LastRangeDataBuffer.MeasurementTimeUsec = pRangingMeasurementData->MeasurementTimeUsec;
        LastRangeDataBuffer.SignalRateRtnMegaCps = pRangingMeasurementData->SignalRateRtnMegaCps;
        LastRangeDataBuffer.AmbientRateRtnMegaCps = pRangingMeasurementData->AmbientRateRtnMegaCps;
        LastRangeDataBuffer.EffectiveSpadRtnCount = pRangingMeasurementData->EffectiveSpadRtnCount;
        LastRangeDataBuffer.RangeStatus = pRangingMeasurementData->RangeStatus;

        PALDevDataSet(Dev, LastRangeMeasure, LastRangeDataBuffer);
    }

    return Status;
}

VL53L0_Error VL53L0_PerformSingleRangingMeasurement(VL53L0_DEV Dev, VL53L0_RangingMeasurementData_t* pRangingMeasurementData) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;

    
    
    Status = VL53L0_SetDeviceMode(Dev, VL53L0_DEVICEMODE_SINGLE_RANGING);

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_PerformSingleMeasurement(Dev);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_GetRangingMeasurementData(Dev, pRangingMeasurementData);
    }

    if(Status==VL53L0_ERROR_NONE){
        Status = VL53L0_ClearInterruptMask(Dev, 0);
    }

    return Status;
}

VL53L0_Error VL53L0_SetGpioConfig(VL53L0_DEV Dev, uint8_t Pin, VL53L0_DeviceModes DeviceMode, VL53L0_GpioFunctionality Functionality, VL53L0_InterruptPolarity Polarity) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceSpecificParameters_t DeviceSpecificParameters;
    uint8_t data;

    if (Pin != 0) {
        Status = VL53L0_ERROR_GPIO_NOT_EXISTING;
    } else if (DeviceMode == VL53L0_DEVICEMODE_GPIO_DRIVE) {
        if (Polarity == VL53L0_INTERRUPTPOLARITY_LOW) {
            data = 0x10;
        } else {
            data = 1;
        }
        Status = VL53L0_WrByte(Dev,
        		VL53L0_REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

    } else if (DeviceMode == VL53L0_DEVICEMODE_GPIO_OSC) {

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0x80, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x85, 0x02);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x04);
    	Status |= VL53L0_WrByte(Dev, 0xcd, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0xcc, 0x11);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x07);
    	Status |= VL53L0_WrByte(Dev, 0xbe, 0x00);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x06);
    	Status |= VL53L0_WrByte(Dev, 0xcc, 0x09);

    	Status |= VL53L0_WrByte(Dev, 0xff, 0x00);
    	Status |= VL53L0_WrByte(Dev, 0xff, 0x01);
    	Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

    } else {

		if (Status == VL53L0_ERROR_NONE) {
			switch(Functionality){
				case VL53L0_GPIOFUNCTIONALITY_OFF:
					data = 0x00;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
					data = 0x01;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
					data = 0x02;
					break;
				case VL53L0_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
					data = 0x03;
					break;
				case VL53L0_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
					data = 0x04;
					break;
				default:
					Status = VL53L0_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
			}
		}

		if(Status==VL53L0_ERROR_NONE)
			Status = VL53L0_WrByte(Dev,
					VL53L0_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);


		if(Status==VL53L0_ERROR_NONE){
			if (Polarity == VL53L0_INTERRUPTPOLARITY_LOW) {
				data = 0;
			} else {
				data = (uint8_t)(1<<4);
			}
			Status = VL53L0_UpdateByte(Dev,
					VL53L0_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
		}

		if(Status==VL53L0_ERROR_NONE)
			VL53L0_SETDEVICESPECIFICPARAMETER(Dev,
					Pin0GpioFunctionality, Functionality);


		if(Status==VL53L0_ERROR_NONE)
			Status = VL53L0_ClearInterruptMask(Dev, 0);

    }

    return Status;
}

VL53L0_Error VL53L0_ClearInterruptMask(VL53L0_DEV Dev, uint32_t InterruptMask) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t LoopCount;
    uint8_t Byte;

    
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    LoopCount = 0;
    do {
        VL53L0_RdByte(Dev, VL53L0_REG_RESULT_INTERRUPT_STATUS, &Byte);
        LoopCount++;
    } while (((Byte & 0x07) != 0x00) && (LoopCount < 8));
    Status = VL53L0_WrByte(Dev, VL53L0_REG_SYSTEM_INTERRUPT_CLEAR, 0x00); 

    return Status;
}

VL53L0_Error VL53L0_GetInterruptMaskStatus(VL53L0_DEV Dev, uint32_t *pInterruptMaskStatus) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t Byte;

    Status = VL53L0_RdByte(Dev, VL53L0_REG_RESULT_INTERRUPT_STATUS, &Byte);
    *pInterruptMaskStatus = Byte & 0x07;

    if (Byte & 0x18){
        Status = VL53L0_ERROR_RANGE_ERROR; 
    }

    return Status;
}

VL53L0_Error VL53L0_get_vcsel_pulse_period(VL53L0_DEV Dev, uint8_t* pVCSELPulsePeriod, uint8_t RangeIndex) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t vcsel_period_reg;


    switch (RangeIndex)
    {
        case 0:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
            break;
        case 1:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
            break;
        case 2:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
            break;
        default:
            Status = VL53L0_RdByte(Dev,
            		VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
            		&vcsel_period_reg);
    }

    if (Status == VL53L0_ERROR_NONE) {
        *pVCSELPulsePeriod = VL53L0_decode_vcsel_period(vcsel_period_reg);
    }

    return Status;
}

uint32_t VL53L0_calc_ranging_wait_us(VL53L0_DEV Dev, uint16_t timeout_overall_periods, uint8_t vcsel_period) {
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t timeout_period_mclks = 0;
    uint32_t actual_timeout_period_us = 0;

    macro_period_ps = VL53L0_calc_macro_period_ps(Dev, vcsel_period);
    macro_period_ns = macro_period_ps / 1000;

    timeout_period_mclks = VL53L0_decode_timeout(timeout_overall_periods);
    actual_timeout_period_us = ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;

    return actual_timeout_period_us;
}

uint32_t VL53L0_calc_macro_period_ps(VL53L0_DEV Dev, uint8_t vcsel_period) {
    uint32_t PLL_multiplier;
    uint64_t PLL_period_ps;
    uint8_t vcsel_period_pclks;
    uint32_t macro_period_vclks;
    uint32_t macro_period_ps;


    PLL_multiplier = 65536 / 64; 
    PLL_period_ps = (1000 * 1000 * PLL_multiplier) / VL53L0_GETDEVICESPECIFICPARAMETER(Dev, OscFrequencyMHz);

    vcsel_period_pclks = VL53L0_decode_vcsel_period(vcsel_period);

    macro_period_vclks = 2304;
    macro_period_ps = (uint32_t)(macro_period_vclks * vcsel_period_pclks * PLL_period_ps);

    return macro_period_ps;
}

uint8_t VL53L0_decode_vcsel_period(uint8_t vcsel_period_reg) {


    uint8_t vcsel_period_pclks = 0;

    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

    return vcsel_period_pclks;
}

uint8_t VL53L0_encode_vcsel_period(uint8_t vcsel_period_pclks) {


    uint8_t vcsel_period_reg = 0;

    vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

    return vcsel_period_reg;
}

uint32_t VL53L0_decode_timeout(uint16_t encoded_timeout) {

    uint32_t timeout_mclks = 0;

    timeout_mclks = ((uint32_t) (encoded_timeout & 0x00FF) << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

    return timeout_mclks;

}

VL53L0_Error VL53L0_check_part_used(VL53L0_DEV Dev, uint8_t* ModuleId, uint8_t* Revision, VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t ModuleIdInt;
    uint8_t RevisionInt;


    Status = VL53L0_get_info_from_device(Dev, &ModuleIdInt, &RevisionInt, pVL53L0_DeviceInfo);

    if (Status == VL53L0_ERROR_NONE) {
        *ModuleId = ModuleIdInt;

        if (ModuleIdInt == 0) {
            *Revision = 0;
        } else {
            *Revision = RevisionInt;
        }
    }

    return Status;
}

VL53L0_Error VL53L0_get_info_from_device(VL53L0_DEV Dev, uint8_t* ModuleId, uint8_t* Revision, VL53L0_DeviceInfo_t* pVL53L0_DeviceInfo) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t byte;
    uint32_t TmpDWord;


    Status |= VL53L0_WrByte(Dev, 0x80, 0x01);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0_WrByte(Dev, 0x00, 0x00);

    Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
    Status |= VL53L0_RdByte(Dev, 0x83, &byte);
    Status |= VL53L0_WrByte(Dev, 0x83, byte|4);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x07);
    Status |= VL53L0_WrByte(Dev, 0x81, 0x01);

    Status |= VL53L0_PollingDelay(Dev);

    Status |= VL53L0_WrByte(Dev, 0x80, 0x01);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x02); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdByte(Dev, 0x90, ModuleId);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x7B); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdByte(Dev, 0x90, Revision);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x77); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

    pVL53L0_DeviceInfo->ProductId[0] = (char)((TmpDWord >> 25) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[1] = (char)((TmpDWord >> 18) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[2] = (char)((TmpDWord >> 11) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[3] = (char)((TmpDWord >> 4) & 0x07f);

    byte = (uint8_t)((TmpDWord & 0x00f) << 3);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x78); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

    pVL53L0_DeviceInfo->ProductId[4] = (char)(byte + ((TmpDWord >> 29) & 0x07f));
    pVL53L0_DeviceInfo->ProductId[5] = (char)((TmpDWord >> 22) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[6] = (char)((TmpDWord >> 15) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[7] = (char)((TmpDWord >> 8) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[8] = (char)((TmpDWord >> 1) & 0x07f);

    byte = (uint8_t)((TmpDWord & 0x001) << 6);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x79); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

    pVL53L0_DeviceInfo->ProductId[9] = (char)(byte + ((TmpDWord >> 26) & 0x07f));
    pVL53L0_DeviceInfo->ProductId[10] = (char)((TmpDWord >> 19) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[11] = (char)((TmpDWord >> 12) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[12] = (char)((TmpDWord >> 5) & 0x07f);

    byte = (uint8_t)((TmpDWord & 0x01f) << 2);

    Status |= VL53L0_WrByte(Dev, 0x94, 0x80); 

    Status |= VL53L0_device_read_strobe(Dev);

    Status |= VL53L0_RdDWord(Dev, 0x90, &TmpDWord);

    pVL53L0_DeviceInfo->ProductId[13] = (char)(byte + ((TmpDWord >> 30) & 0x07f));
    pVL53L0_DeviceInfo->ProductId[14] = (char)((TmpDWord >> 23) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[15] = (char)((TmpDWord >> 16) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[16] = (char)((TmpDWord >> 9) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[17] = (char)((TmpDWord >> 2) & 0x07f);
    pVL53L0_DeviceInfo->ProductId[18] = '\0';

    Status |= VL53L0_WrByte(Dev, 0x81, 0x00);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x06);
    Status |= VL53L0_RdByte(Dev, 0x83, &byte);
    Status |= VL53L0_WrByte(Dev, 0x83, byte&0xfb);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0_WrByte(Dev, 0x00, 0x01);

    Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);
    Status |= VL53L0_WrByte(Dev, 0x80, 0x00);

    return Status;
}

VL53L0_Error VL53L0_device_read_strobe(VL53L0_DEV Dev) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t strobe;
    uint32_t LoopNb;

    Status |= VL53L0_WrByte(Dev, 0x83, 0x00);

    
    
    if (Status == VL53L0_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status =VL53L0_RdByte(Dev, 0x83, &strobe);
            if ((strobe != 0x00) || Status != VL53L0_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
        } while (LoopNb < VL53L0_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0_DEFAULT_MAX_LOOP) {
            Status = VL53L0_ERROR_TIME_OUT;
        }
    }

    Status |= VL53L0_WrByte(Dev, 0x83, 0x01);

    return Status;

}

uint32_t VL53L0_isqrt(uint32_t num)
{


    uint32_t  res = 0;
    uint32_t  bit = 1 << 30; 

    
    while (bit > num)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if (num >= res + bit)
        {
            num -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

uint32_t VL53L0_quadrature_sum(uint32_t a,
                                uint32_t b)
{
    uint32_t  res = 0;

    if( a > 65535 || b > 65535)
    {
        res = 65535;
    }
    else
    {
        res = VL53L0_isqrt(a*a + b*b);
    }

    return res;
}

VL53L0_Error VL53L0_get_jmp_vcsel_ambient_rate(VL53L0_DEV Dev, uint32_t *pAmbient_rate_kcps, uint32_t *pVcsel_rate_kcps, uint32_t *pSignalTotalEventsRtn)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint16_t encodedTimeOut;

    uint32_t    total_periods_elapsed_rtn__macrop  = 0;
    uint32_t    result_core__total_periods_elapsed_rtn  = 0;
    uint32_t    rngb1_config__timeout__macrop = 0;
    uint32_t    rngb2_config__timeout__macrop = 0;
    uint32_t    result_core__ambient_window_events_rtn = 0;
    uint32_t     result_core__signal_total_events_rtn = 0;
    uint8_t     last_woi_period;
    uint8_t     rnga_config__vcsel_period;
    uint8_t     rngb1_config__vcsel_period;
    uint8_t     rngb2_config__vcsel_period;
    uint8_t     global_config__vcsel_width;

    uint32_t    ambient_duration_us = 0;
    uint32_t    vcsel_duration_us = 0;

    uint32_t    pll_period_us  = 0;


    
    Status = VL53L0_WrByte(Dev, 0xFF, 0x01);
    Status |= VL53L0_RdDWord(Dev, 0xC8, &result_core__total_periods_elapsed_rtn);
    Status |= VL53L0_RdDWord(Dev, 0xF0, &pll_period_us);
    Status |= VL53L0_RdDWord(Dev, 0xbc, &result_core__ambient_window_events_rtn);
    Status |= VL53L0_RdDWord(Dev, 0xc0, &result_core__signal_total_events_rtn);
    Status |= VL53L0_WrByte(Dev, 0xFF, 0x00);


    if (Status == VL53L0_ERROR_NONE) {
        result_core__total_periods_elapsed_rtn = (int32_t)(result_core__total_periods_elapsed_rtn & 0x00ffffff);
        pll_period_us = (int32_t)(pll_period_us & 0x3ffff);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdWord(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &encodedTimeOut);
        if (Status == VL53L0_ERROR_NONE) {
            rngb1_config__timeout__macrop = VL53L0_decode_timeout(encodedTimeOut) - 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
        		&rnga_config__vcsel_period);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
        		&rngb1_config__vcsel_period);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, VL53L0_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
        		&rngb2_config__vcsel_period);
    }
    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdByte(Dev, 0x32, &global_config__vcsel_width);
    }

    if (Status == VL53L0_ERROR_NONE) {
        Status = VL53L0_RdWord(Dev,
        		VL53L0_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        		&encodedTimeOut);
        if (Status == VL53L0_ERROR_NONE) {
            rngb2_config__timeout__macrop =
            		VL53L0_decode_timeout(encodedTimeOut) - 1;
        }
    }

    if (Status == VL53L0_ERROR_NONE)
    {
        total_periods_elapsed_rtn__macrop =
        		result_core__total_periods_elapsed_rtn + 1;

        if (result_core__total_periods_elapsed_rtn ==
        		rngb1_config__timeout__macrop)
        {
           last_woi_period = rngb1_config__vcsel_period;
        }
        else if (result_core__total_periods_elapsed_rtn ==
        		rngb2_config__timeout__macrop)
        {
           last_woi_period = rngb2_config__vcsel_period;
        }
        else
        {
           last_woi_period = rnga_config__vcsel_period;

        }
        
        ambient_duration_us = last_woi_period *
        		total_periods_elapsed_rtn__macrop * pll_period_us;
        ambient_duration_us = ambient_duration_us / 1000;

        if (ambient_duration_us != 0) {
            *pAmbient_rate_kcps = ((1 << 15) *
            		result_core__ambient_window_events_rtn) /
            				ambient_duration_us;
        } else {
            Status = VL53L0_ERROR_DIVISION_BY_ZERO;
        }

        if (Status == VL53L0_ERROR_NONE)
        {

            
            vcsel_duration_us = (10*global_config__vcsel_width + 4) *
            		total_periods_elapsed_rtn__macrop * pll_period_us ;
            vcsel_duration_us = vcsel_duration_us / 10000 ;


            if (vcsel_duration_us != 0) {
                *pVcsel_rate_kcps = ((1 << 13) *
                		result_core__signal_total_events_rtn) /
                				vcsel_duration_us;
                *pSignalTotalEventsRtn = result_core__signal_total_events_rtn;
            } else {
                Status = VL53L0_ERROR_DIVISION_BY_ZERO;
            }

        }
    }

    return Status;

}

VL53L0_Error VL53L0_calc_sigma_estimate(VL53L0_DEV Dev, VL53L0_RangingMeasurementData_t *pRangingMeasurementData, FixPoint1616_t* pSigmaEstimate)
{
	
    const uint32_t cPulseEffectiveWidth_centi_ns   = 800;
    
    const uint32_t cAmbientEffectiveWidth_centi_ns = 600;
    const FixPoint1616_t cSigmaEstRef              = 0x00000042;
    const uint32_t cVcselPulseWidth_ps             = 4700; 
    const FixPoint1616_t cSigmaEstMax              = 0x028F87AE;
    
    const FixPoint1616_t cTOF_per_mm_ps            = 0x0006999A;
    const uint32_t c16BitRoundingParam             = 0x00008000;
    const FixPoint1616_t cMaxXTalk_kcps            = 0x00320000;

    uint32_t signalTotalEventsRtn;
    FixPoint1616_t sigmaEstimateP1;
    FixPoint1616_t sigmaEstimateP2;
    FixPoint1616_t sigmaEstimateP3;
    FixPoint1616_t deltaT_ps;
    FixPoint1616_t pwMult;
    FixPoint1616_t sigmaEstRtn;
    FixPoint1616_t sigmaEstimate;
    FixPoint1616_t xTalkCorrection;
    uint32_t signalTotalEventsRtnRawVal;
    FixPoint1616_t ambientRate_kcps;
    FixPoint1616_t vcselRate_kcps;
    FixPoint1616_t xTalkCompRate_mcps;
    uint32_t xTalkCompRate_kcps;
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DeviceParameters_t CurrentParameters;
    FixPoint1616_t diff1_mcps;
    FixPoint1616_t diff2_mcps;
    FixPoint1616_t sqr1;
    FixPoint1616_t sqr2;
    FixPoint1616_t sqrSum;
    FixPoint1616_t sqrtResult_centi_ns;
    FixPoint1616_t sqrtResult;



    VL53L0_GETPARAMETERFIELD(Dev, XTalkCompensationRateMegaCps,
    		xTalkCompRate_mcps);


    xTalkCompRate_kcps = xTalkCompRate_mcps * 1000;
    if(xTalkCompRate_kcps > cMaxXTalk_kcps)
    {
        xTalkCompRate_kcps = cMaxXTalk_kcps;
    }

    Status =  VL53L0_get_jmp_vcsel_ambient_rate(Dev,
                                                &ambientRate_kcps,
                                                &vcselRate_kcps,
                                                &signalTotalEventsRtnRawVal);
    if (Status == VL53L0_ERROR_NONE) {
        if (vcselRate_kcps == 0) {
            *pSigmaEstimate = 0;
            PALDevDataSet(Dev, SigmaEstimate, 0);
        } else {
            signalTotalEventsRtn = signalTotalEventsRtnRawVal;
            if(signalTotalEventsRtn < 1)
            {
                signalTotalEventsRtn = 1;
            }


            sigmaEstimateP1 = cPulseEffectiveWidth_centi_ns;

            
            sigmaEstimateP2 = (ambientRate_kcps << 16)/vcselRate_kcps;
            sigmaEstimateP2 *= cAmbientEffectiveWidth_centi_ns;

            sigmaEstimateP3 = 2 * VL53L0_isqrt(signalTotalEventsRtn * 12);

            
            deltaT_ps = pRangingMeasurementData->RangeMilliMeter * cTOF_per_mm_ps;

            diff1_mcps = (((vcselRate_kcps << 16) - xTalkCompRate_kcps) + 500)/1000;

            
            diff2_mcps = (((vcselRate_kcps << 16) + xTalkCompRate_kcps) + 500)/1000;

            
            diff1_mcps <<= 12;

            
            xTalkCorrection  = ABS(diff1_mcps/diff2_mcps);

            
            xTalkCorrection <<= 4;

            
            pwMult = deltaT_ps/cVcselPulseWidth_ps; 

            pwMult *= ((1 << 16) - xTalkCorrection);

            
            pwMult =  (pwMult + c16BitRoundingParam) >> 16;

            
            pwMult += (1 << 16);

            pwMult >>= 1;
            
            pwMult = pwMult * pwMult;

            
            pwMult >>= 14;

            
            sqr1 = pwMult * sigmaEstimateP1;

            
            sqr1 = (sqr1 + 0x800) >> 12;

            
            sqr1 *= sqr1;

            sqr2 = sigmaEstimateP2;

            
            sqr2 = (sqr2 + 0x800) >> 12;

            
            sqr2 *= sqr2;

            
            sqrSum = sqr1 + sqr2;

            
            sqrtResult_centi_ns = VL53L0_isqrt(sqrSum);

            
            sqrtResult_centi_ns <<= 12;

            sigmaEstRtn      = ((sqrtResult_centi_ns+50)/100 * VL53L0_SPEED_OF_LIGHT_IN_AIR);
            sigmaEstRtn      /= (sigmaEstimateP3);
            sigmaEstRtn      += 5000; 
            sigmaEstRtn      /= 10000;

            
            sqr1 = sigmaEstRtn * sigmaEstRtn;
            
            sqr2 = cSigmaEstRef * cSigmaEstRef;

            
            sqrtResult = VL53L0_isqrt((sqr1 + sqr2) << 12);
            sqrtResult = (sqrtResult + 0x20) >> 6;

            sigmaEstimate    = 1000 * sqrtResult;

            if((vcselRate_kcps < 1) || (signalTotalEventsRtn < 1) || (sigmaEstimate > cSigmaEstMax))
            {
                sigmaEstimate = cSigmaEstMax;
            }

            *pSigmaEstimate = (uint32_t)(sigmaEstimate);
            PALDevDataSet(Dev, SigmaEstimate, *pSigmaEstimate);
        }
    }

    return Status;

}

VL53L0_Error VL53L0_get_pal_range_status(VL53L0_DEV Dev,
                                         uint8_t DeviceRangeStatus,
                                         FixPoint1616_t SignalRate,
                                         FixPoint1616_t CrosstalkCompensation,
                                         uint16_t EffectiveSpadRtnCount,
                                         VL53L0_RangingMeasurementData_t *pRangingMeasurementData,
                                         uint8_t* pPalRangeStatus) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    uint8_t tmpByte;
    uint8_t SigmaLimitCheckEnable = 0;
    uint8_t SignalRefClipLimitCheckEnable = 0;
    FixPoint1616_t SigmaEstimate;
    FixPoint1616_t SigmaLimitValue;
    FixPoint1616_t SignalRefClipValue;
    uint8_t DeviceRangeStatusInternal = 0;
    uint16_t tmpWord = 0;
    FixPoint1616_t LastSignalRefMcps;



    DeviceRangeStatusInternal = ((DeviceRangeStatus & 0x78) >> 3);

    if (DeviceRangeStatusInternal == 11) {
        tmpByte = 0;
    } else if (DeviceRangeStatusInternal == 0) {
        tmpByte = 11;
    } else {
        tmpByte = DeviceRangeStatusInternal;
    }

    
    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x01);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_RdWord(Dev, VL53L0_REG_RESULT_PEAK_SIGNAL_RATE_REF,
        		&tmpWord);

    LastSignalRefMcps = VL53L0_FIXPOINT97TOFIXPOINT1616(tmpWord);

    if (Status == VL53L0_ERROR_NONE)
        Status = VL53L0_WrByte(Dev, 0xFF, 0x00);

    PALDevDataSet(Dev, LastSignalRefMcps, LastSignalRefMcps);

    if (Status == VL53L0_ERROR_NONE)
    	Status =  VL53L0_GetLimitCheckEnable(Dev,
    		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitCheckEnable);

    if ((SigmaLimitCheckEnable != 0) && (Status == VL53L0_ERROR_NONE)) {
        Status = VL53L0_calc_sigma_estimate(Dev, pRangingMeasurementData,
        		&SigmaEstimate);

        if (Status == VL53L0_ERROR_NONE) {
            Status = VL53L0_GetLimitCheckValue(Dev,
            		VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, &SigmaLimitValue);

            if ((SigmaLimitValue > 0) && (SigmaEstimate > SigmaLimitValue)) {
                
                tmpByte += 16;
            }
        }
    }

    if (Status == VL53L0_ERROR_NONE)
    	Status =  VL53L0_GetLimitCheckEnable(Dev,
    			VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
    			&SignalRefClipLimitCheckEnable);

    if ((SignalRefClipLimitCheckEnable != 0) && (Status == VL53L0_ERROR_NONE)) {

		Status = VL53L0_GetLimitCheckValue(Dev,
				VL53L0_CHECKENABLE_SIGNAL_REF_CLIP, &SignalRefClipValue);

		if ((SignalRefClipValue > 0) &&
				(LastSignalRefMcps > SignalRefClipValue)) {
			
			tmpByte += 32;
		}
    }

    if (Status == VL53L0_ERROR_NONE) {
        *pPalRangeStatus = tmpByte;
    }


    return Status;

}

VL53L0_Error VL53L0_load_tuning_settings(VL53L0_DEV Dev, uint8_t* pTuningSettingBuffer) {
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    int i;
    int Index;
    uint8_t msb;
    uint8_t lsb;
    uint8_t SelectParam;
    uint8_t NumberOfWrites;
    uint8_t Address;
    uint8_t localBuffer[4]; 
    uint16_t Temp16;


    Index = 0;

    while ((*(pTuningSettingBuffer+Index) != 0) &&
    		(Status == VL53L0_ERROR_NONE)) {
        NumberOfWrites = *(pTuningSettingBuffer+Index);
        Index++;
        if (NumberOfWrites == 0xFF) {
            
            SelectParam = *(pTuningSettingBuffer+Index);
            Index++;
            switch (SelectParam) {
                case 0: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstRefArray, Temp16);
                    break;
                case 1: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffPulseWidth, Temp16);
                    break;
                case 2: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, SigmaEstEffAmbWidth, Temp16);
                    break;
                case 3: 
                    msb = *(pTuningSettingBuffer+Index);
                    Index++;
                    lsb = *(pTuningSettingBuffer+Index);
                    Index++;
                    Temp16 = VL53L0_MAKEUINT16(lsb, msb);
                    PALDevDataSet(Dev, targetRefRate, Temp16);
                    break;
                default: 
                    Status = VL53L0_ERROR_INVALID_PARAMS;
            }

        } else if (NumberOfWrites <= 4) {
            Address = *(pTuningSettingBuffer+Index);
            Index++;

            for (i=0; i<NumberOfWrites; i++) {
                localBuffer[i] = *(pTuningSettingBuffer+Index);
                Index++;
            }


            if (NumberOfWrites == 1) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
            } else if (NumberOfWrites == 2) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
            } else if (NumberOfWrites == 3) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
                Status = VL53L0_WrByte(Dev, Address+2, localBuffer[2]);
           } else if (NumberOfWrites == 4) {
                Status = VL53L0_WrByte(Dev, Address, localBuffer[0]);
                Status = VL53L0_WrByte(Dev, Address+1, localBuffer[1]);
                Status = VL53L0_WrByte(Dev, Address+2, localBuffer[2]);
                Status = VL53L0_WrByte(Dev, Address+3, localBuffer[3]);
           }

        } else {
            Status = VL53L0_ERROR_INVALID_PARAMS;
        }
    }


    return Status;
}

