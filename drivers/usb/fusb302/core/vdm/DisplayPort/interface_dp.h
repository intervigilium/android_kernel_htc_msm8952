#ifndef _FSC_INTERFACE_DP_H
#define _FSC_INTERFACE_DP_H

#include "../../platform.h"
#include "dp_types.h"

VOID requestDpStatus(VOID);
VOID requestDpConfig(DisplayPortConfig_t in);
VOID configDp(BOOL enabled, UINT32 status);
VOID configAutoDpModeEntry(BOOL enabled, UINT32 mask, UINT32 value);
VOID WriteDpControls(UINT8* data);
VOID ReadDpControls(UINT8* data);
VOID ReadDpStatus(UINT8* data);

VOID informStatus(DisplayPortStatus_t stat);
VOID updateStatusData(VOID);
VOID informConfigResult (BOOL success);
BOOL DpReconfigure(DisplayPortConfig_t config);

extern BOOL DpEnabled;
extern BOOL DpAutoModeEntryEnabled;
extern DisplayPortCaps_t DpModeEntryMask;
extern DisplayPortCaps_t DpModeEntryValue;

extern DisplayPortCaps_t DpCaps;
extern DisplayPortStatus_t DpStatus;
extern DisplayPortConfig_t DpConfig;

extern DisplayPortStatus_t DpPpStatus;
extern DisplayPortConfig_t DpPpRequestedConfig;
extern DisplayPortConfig_t DpPpConfig;
extern UINT32 DpModeEntered;

#endif 
