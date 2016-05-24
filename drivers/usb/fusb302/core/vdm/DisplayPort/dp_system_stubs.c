#include "../../platform.h"
#include "interface_dp.h"

VOID informStatus(DisplayPortStatus_t stat)
{
	
	
    DpPpStatus.word = stat.word;
}

VOID informConfigResult (BOOL success)
{
	
	
    if (success == TRUE) DpPpConfig.word = DpPpRequestedConfig.word;
}

VOID updateStatusData(VOID)
{
	
	
}

BOOL DpReconfigure(DisplayPortConfig_t config)
{
    
    
    
    DpConfig.word = config.word;
    
    return TRUE;
}
