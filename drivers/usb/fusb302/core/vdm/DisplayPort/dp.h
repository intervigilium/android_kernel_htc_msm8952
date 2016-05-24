#ifndef __DISPLAYPORT_DP_H__
#define __DISPLAYPORT_DP_H__

#include "../../platform.h"
#include "../../PD_Types.h"
#include "../vdm_types.h"
#include "dp_types.h"

VOID initializeDp(VOID);

VOID resetDp(VOID);

BOOL processDpCommand(UINT32* arr_in);

VOID sendStatusData(doDataObject_t svdmh_in);

VOID replyToConfig(doDataObject_t svdmh_in, BOOL success);

BOOL dpEvaluateModeEntry(UINT32 mode_in);


#endif
