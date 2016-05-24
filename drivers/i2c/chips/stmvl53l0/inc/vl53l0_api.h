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



#ifndef _VL53L0_API_H_
#define _VL53L0_API_H_

#include "vl53l0_def.h"
#include "vl53l0_platform.h"


#ifdef __cplusplus
extern "C" {
#endif


#ifdef _MSC_VER
#   ifdef VL53L0_API_EXPORTS
#       define VL53L0_API  __declspec(dllexport)
#   else
#       define VL53L0_API
#   endif
#else
#   define VL53L0_API
#endif



VL53L0_API VL53L0_Error VL53L0_SetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev, int32_t OffsetCalibrationDataMicroMeter);

VL53L0_API VL53L0_Error VL53L0_GetOffsetCalibrationDataMicroMeter(VL53L0_DEV Dev, int32_t * pOffsetCalibrationDataMicroMeter);

VL53L0_API VL53L0_Error VL53L0_DataInit(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_StaticInit(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_GetDeviceParameters(VL53L0_DEV Dev, VL53L0_DeviceParameters_t* pDeviceParameters);

VL53L0_API VL53L0_Error VL53L0_SetDeviceMode(VL53L0_DEV Dev, VL53L0_DeviceModes DeviceMode);

VL53L0_API VL53L0_Error VL53L0_GetDeviceMode(VL53L0_DEV Dev, VL53L0_DeviceModes* pDeviceMode);

VL53L0_API VL53L0_Error VL53L0_GetHistogramMode(VL53L0_DEV Dev, VL53L0_HistogramModes* pHistogramMode);

VL53L0_API VL53L0_Error VL53L0_GetMeasurementTimingBudgetMicroSeconds(VL53L0_DEV Dev, uint32_t*  pMeasurementTimingBudgetMicroSeconds);

VL53L0_API VL53L0_Error VL53L0_GetInterMeasurementPeriodMilliSeconds(VL53L0_DEV Dev, uint32_t* pInterMeasurementPeriodMilliSeconds);

VL53L0_API VL53L0_Error VL53L0_SetXTalkCompensationEnable(VL53L0_DEV Dev, uint8_t XTalkCompensationEnable);

VL53L0_API VL53L0_Error VL53L0_GetXTalkCompensationEnable(VL53L0_DEV Dev, uint8_t* pXTalkCompensationEnable);

VL53L0_API VL53L0_Error VL53L0_SetXTalkCompensationRateMegaCps(VL53L0_DEV Dev, FixPoint1616_t XTalkCompensationRateMegaCps);

VL53L0_API VL53L0_Error VL53L0_GetXTalkCompensationRateMegaCps(VL53L0_DEV Dev, FixPoint1616_t* pXTalkCompensationRateMegaCps);

VL53L0_API VL53L0_Error VL53L0_SetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId, uint8_t LimitCheckEnable);

VL53L0_API VL53L0_Error VL53L0_GetLimitCheckEnable(VL53L0_DEV Dev, uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);

VL53L0_API VL53L0_Error VL53L0_SetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);

VL53L0_API VL53L0_Error VL53L0_GetLimitCheckValue(VL53L0_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);

VL53L0_API VL53L0_Error VL53L0_GetWrapAroundCheckEnable(VL53L0_DEV Dev, uint8_t* pWrapAroundCheckEnable);

VL53L0_API VL53L0_Error VL53L0_PerformSingleMeasurement(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_PerformRefCalibration(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_PerformXTalkCalibration(VL53L0_DEV Dev, FixPoint1616_t XTalkCalDistance, FixPoint1616_t* pXTalkCompensationRateMegaCps);

VL53L0_API VL53L0_Error VL53L0_PerformOffsetCalibration(VL53L0_DEV Dev, uint16_t CalDistanceMilliMeter, int32_t* pOffsetMicroMeter);

VL53L0_API VL53L0_Error VL53L0_StartMeasurement(VL53L0_DEV Dev);

VL53L0_API VL53L0_Error VL53L0_GetMeasurementDataReady(VL53L0_DEV Dev, uint8_t *pMeasurementDataReady);

VL53L0_API VL53L0_Error VL53L0_GetRangingMeasurementData(VL53L0_DEV Dev, VL53L0_RangingMeasurementData_t *pRangingMeasurementData);

VL53L0_API VL53L0_Error VL53L0_PerformSingleRangingMeasurement(VL53L0_DEV Dev, VL53L0_RangingMeasurementData_t *pRangingMeasurementData);

VL53L0_API VL53L0_Error VL53L0_SetGpioConfig(VL53L0_DEV Dev, uint8_t Pin, VL53L0_DeviceModes DeviceMode, VL53L0_GpioFunctionality Functionality, VL53L0_InterruptPolarity Polarity);

VL53L0_API VL53L0_Error VL53L0_ClearInterruptMask(VL53L0_DEV Dev, uint32_t InterruptMask);

VL53L0_API VL53L0_Error VL53L0_GetInterruptMaskStatus(VL53L0_DEV Dev, uint32_t *pInterruptMaskStatus);



#ifdef __cplusplus
}
#endif

#endif 

