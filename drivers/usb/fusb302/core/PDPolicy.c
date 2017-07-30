/****************************************************************************
 * FileName:        PDPolicy.c
 * Processor:       PIC32MX250F128B
 * Compiler:        MPLAB XC32
 * Company:         Fairchild Semiconductor
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the “Company”)
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include "PD_Types.h"
#include "Log.h"
#include "PDPolicy.h"
#include "PDProtocol.h"
#include "TypeC.h"
#include "fusb30X.h"
#include "vdm/vdm_callbacks.h"
#include "vdm/vdm_callbacks_defs.h"
#include "vdm/vdm.h"
#include "vdm/vdm_types.h"
#include "vdm/bitfield_translators.h"
#include "vdm/DisplayPort/dp_types.h"
#include "vdm/DisplayPort/dp.h"
#include "vdm/DisplayPort/interface_dp.h"


extern volatile UINT16          Timer_S;                                        
extern volatile UINT16          Timer_tms;                                      
extern BOOL                     g_Idle;                                         

extern int fusb_debug_level;

static UINT8                    USBPDBufStart;                                  
static UINT8                    USBPDBufEnd;                                    
static BOOL                     USBPDBufOverflow;                               
StateLog                        PDStateLog;                                     
PolicyState_t                   LastPolicyState;

BOOL                            USBPDTxFlag;                                    
BOOL                            IsHardReset;                                    
BOOL                            IsVCONNSource;                                  
sopMainHeader_t                 PDTransmitHeader;                               
sopMainHeader_t                 CapsHeaderSink;                                 
sopMainHeader_t                 CapsHeaderSource;                               
sopMainHeader_t                 CapsHeaderReceived;                             
doDataObject_t                  PDTransmitObjects[7];                           
doDataObject_t                  CapsSink[7];                                    
doDataObject_t                  CapsSource[7];                                  
doDataObject_t                  CapsReceived[7];                                
doDataObject_t                  USBPDContract;                                  
doDataObject_t                  SinkRequest;                                    
UINT32                          SinkRequestMaxVoltage;                          
UINT32                          SinkRequestMaxPower;                            
UINT32                          SinkRequestOpPower;                             
BOOL                            SinkGotoMinCompatible;                          
BOOL                            SinkUSBSuspendOperation;                        
BOOL                            SinkUSBCommCapable;                             
BOOL                            SourceCapsUpdated;                              

       PolicyState_t            PolicyState;                                    
       PolicyState_t            LastPolicyState;                                
       UINT8                    PolicySubIndex;                                 
       BOOL                     PolicyIsSource;                                 
       BOOL                     PolicyIsDFP;                                    
       BOOL                     PolicyHasContract;                              
       UINT32                   VbusTransitionTime;                             
static UINT8                    CollisionCounter;                               
static UINT8                    HardResetCounter;                               
static UINT8                    CapsCounter;                                    
static UINT32                   PolicyStateTimer;                               
       UINT32                   NoResponseTimer;                                
static UINT32                   SwapSourceStartTimer;                           
       sopMainHeader_t          PolicyRxHeader;                                 
       sopMainHeader_t          PolicyTxHeader;                                 
       doDataObject_t           PolicyRxDataObj[7];                             
       doDataObject_t           PolicyTxDataObj[7];                             
static BOOL                     isPartnerDRP;                                   
static BOOL                     isContractValid;                                

extern VdmManager               vdmm;
VdmDiscoveryState_t             AutoVdmState;
extern BOOL                     mode_entered;
extern UINT32                   DpModeEntered;

extern SvidInfo                 core_svid_info;
extern INT32                    AutoDpModeEntryObjPos;

extern UINT8                    manualRetries;                                  
extern UINT8                    nTries;                                       
UINT16                          auto_mode_disc_tracker;

extern BOOL                     ProtocolCheckRxBeforeTx;

UINT32 vdm_msg_length;
doDataObject_t vdm_msg_obj[7];
PolicyState_t vdm_next_ps;
BOOL sendingVdmData;

UINT32 VdmTimer;
BOOL   VdmTimerStarted;

void PolicyTickAt100us( void )
{
    if( !USBPDActive )
        return;

    if (PolicyStateTimer)                                                       
        PolicyStateTimer--;                                                     
    if ((NoResponseTimer < T_TIMER_DISABLE) && (NoResponseTimer > 0))        
        NoResponseTimer--;                                                      
    if (VdmTimer)
        VdmTimer--;
    if (SwapSourceStartTimer)
        SwapSourceStartTimer--;
}

void InitializePDPolicyVariables(void)
{
    SwapSourceStartTimer = 0;
    USBPDBufStart = 0;                                                          
    USBPDBufEnd = 0;                                                            
    USBPDBufOverflow = FALSE;                                                   
    SinkRequestMaxVoltage = 240;                                                
    SinkRequestMaxPower = 1000;                                                 
    SinkRequestOpPower = 1000;                                                  
    SinkGotoMinCompatible = FALSE;                                              
    SinkUSBSuspendOperation = FALSE;                                            
    SinkUSBCommCapable = FALSE;                                                 
    SourceCapsUpdated = FALSE;                                                  
    VbusTransitionTime = 20 * TICK_SCALE_TO_MS;                                 
    CapsHeaderSource.NumDataObjects = 2;                                        
    CapsHeaderSource.PortDataRole = 0;                                          
    CapsHeaderSource.PortPowerRole = 1;                                         
    CapsHeaderSource.SpecRevision = 1;                                          
    CapsSource[0].FPDOSupply.Voltage = 100;                                     
    CapsSource[0].FPDOSupply.MaxCurrent = 50;                                   
    CapsSource[0].FPDOSupply.PeakCurrent = 0;                                   
    CapsSource[0].FPDOSupply.DataRoleSwap = TRUE;                               
    CapsSource[0].FPDOSupply.USBCommCapable = FALSE;                            
    CapsSource[0].FPDOSupply.ExternallyPowered = TRUE;                          
    CapsSource[0].FPDOSupply.USBSuspendSupport = FALSE;                         
    CapsSource[0].FPDOSupply.DualRolePower = TRUE;                              
    CapsSource[0].FPDOSupply.SupplyType = 0;                                    

    CapsSource[1].FPDOSupply.Voltage = 240;                                     
    CapsSource[1].FPDOSupply.MaxCurrent = 50;                                   
    CapsSource[1].FPDOSupply.PeakCurrent = 0;                                   
    CapsSource[1].FPDOSupply.DataRoleSwap = 0;                                  
    CapsSource[1].FPDOSupply.USBCommCapable = 0;                                
    CapsSource[1].FPDOSupply.ExternallyPowered = 0;                             
    CapsSource[1].FPDOSupply.USBSuspendSupport = 0;                             
    CapsSource[1].FPDOSupply.DualRolePower = 0;                                 
    CapsSource[1].FPDOSupply.SupplyType = 0;                                    

    CapsHeaderSink.NumDataObjects = 2;                                          
    CapsHeaderSink.PortDataRole = 0;                                            
    CapsHeaderSink.PortPowerRole = 0;                                           
    CapsHeaderSink.SpecRevision = 1;                                            
    CapsSink[0].FPDOSink.Voltage = 100;                                         
    CapsSink[0].FPDOSink.OperationalCurrent = 10;                               
    CapsSink[0].FPDOSink.DataRoleSwap = 0;                                      
    CapsSink[0].FPDOSink.USBCommCapable = 0;                                    
    CapsSink[0].FPDOSink.ExternallyPowered = 0;                                 
    CapsSink[0].FPDOSink.HigherCapability = FALSE;                              
    CapsSink[0].FPDOSink.DualRolePower = 0;                                     

    CapsSink[1].FPDOSink.Voltage = 240;                                         
    CapsSink[1].FPDOSink.OperationalCurrent = 10;                               
    CapsSink[1].FPDOSink.DataRoleSwap = 0;                                      
    CapsSink[1].FPDOSink.USBCommCapable = 0;                                    
    CapsSink[1].FPDOSink.ExternallyPowered = 0;                                 
    CapsSink[1].FPDOSink.HigherCapability = 0;                                  
    CapsSink[1].FPDOSink.DualRolePower = 0;                                     

    InitializeVdmManager();                                                
    vdmInitDpm();
    AutoVdmState = AUTO_VDM_INIT;
    auto_mode_disc_tracker = 0;
    AutoDpModeEntryObjPos = -1;

    ProtocolCheckRxBeforeTx = FALSE;
    isContractValid = FALSE;

    InitializeStateLog(&PDStateLog);
}


void USBPDEnable(BOOL DeviceUpdate, BOOL TypeCDFP)
{
    UINT8 data[5];
    IsHardReset = FALSE;
    isPartnerDRP = FALSE;

	if (fusb_debug_level >= 0)
		printk("FUSB  [%s] - USBPDEnable(%d,%d), PDflag -> USBPDEnabled:%d\n",
			__func__, DeviceUpdate, TypeCDFP, USBPDEnabled);

    if (USBPDEnabled == TRUE)
    {
        if (blnCCPinIsCC1) {                                                    
            Registers.Switches.TXCC1 = 1;                                    
            Registers.Switches.MEAS_CC1 = 1;

            Registers.Switches.TXCC2 = 0;                                   
            Registers.Switches.MEAS_CC2 = 0;
        }
        else if (blnCCPinIsCC2) {                                               
            Registers.Switches.TXCC2 = 1;                                    
            Registers.Switches.MEAS_CC2 = 1;

            Registers.Switches.TXCC1 = 0;                                   
            Registers.Switches.MEAS_CC1 = 0;
        }
        if (blnCCPinIsCC1 || blnCCPinIsCC2)                                     
        {
            USBPDActive = TRUE;                                                 
            ResetProtocolLayer(FALSE);                                          
            NoResponseTimer = T_TIMER_DISABLE;                                  
            PolicyIsSource = TypeCDFP;                                          
            PolicyIsDFP = TypeCDFP;                                             
            if (PolicyIsSource)                                                 
            {
                PolicyState = peSourceStartup;                                  
                PolicySubIndex = 0;
                LastPolicyState = peDisabled;
                Registers.Switches.POWERROLE = 1;                               
                Registers.Switches.DATAROLE = 1;                                
                IsVCONNSource = TRUE;
            }
            else                                                                
            {
                PolicyState = peSinkStartup;                                    
                PolicySubIndex = 0;
                LastPolicyState = peDisabled;
                Registers.Switches.POWERROLE = 0;                               
                Registers.Switches.DATAROLE = 0;                                
                IsVCONNSource = FALSE;
            }
            Registers.Switches.AUTO_CRC = 1;
            Registers.Power.PWR |= 0x8;                                         
            Registers.Control.AUTO_PRE = 0;                                     
            if(manualRetries)
            {
                Registers.Control.N_RETRIES = 0;                                    
            }
            else
            {
                Registers.Control.N_RETRIES = 3;                                    
            }
            Registers.Control.AUTO_RETRY = 1;                                   
            Registers.Slice.SDAC = SDAC_DEFAULT;                                
            data[0] = Registers.Slice.byte;                                     
            data[1] = Registers.Control.byte[0] | 0x40;                         
            data[2] = Registers.Control.byte[1] | 0x04;                         
            data[3] = Registers.Control.byte[2];
            data[4] = Registers.Control.byte[3];
            DeviceWrite(regSlice, 1, &data[0]);                                 
            DeviceWrite(regControl0, 4, &data[1]);
            if (DeviceUpdate)
            {
                DeviceWrite(regPower, 1, &Registers.Power.byte);                
                DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);      
            }
            StoreUSBPDToken(TRUE, pdtAttach);                                   
        }

#ifdef FSC_INTERRUPT_TRIGGERED
        g_Idle = FALSE;                                                         
        Registers.Mask.byte = 0x00;
        DeviceWrite(regMask, 1, &Registers.Mask.byte);
        Registers.MaskAdv.byte[0] = 0x00;
        DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
        Registers.MaskAdv.M_GCRCSENT = 0;
        DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
        platform_enable_timer(TRUE);
#endif
    }
}

void USBPDDisable(BOOL DeviceUpdate)
{
    IsHardReset = FALSE;
    if (USBPDActive == TRUE)                                                    
        StoreUSBPDToken(TRUE, pdtDetach);                                       
    USBPDActive = FALSE;                                                        
    ProtocolState = PRLDisabled;                                                
    PolicyState = peDisabled;                                                   
    PDTxStatus = txIdle;                                                        
    PolicyIsSource = FALSE;                                                     
    PolicyHasContract = FALSE;                                                  
    SourceCapsUpdated = TRUE;                                                   
    if (DeviceUpdate)
    {
        Registers.Switches.TXCC1 = 0;                                           
        Registers.Switches.TXCC2 = 0;
        Registers.Switches.AUTO_CRC = 0;                                        
        Registers.Power.PWR &= 0x7;                                             

        DeviceWrite(regPower, 1, &Registers.Power.byte);                       
        DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);             
    }
    ProtocolFlushRxFIFO();
    ProtocolFlushTxFIFO();
}

void EnableUSBPD(void)
{
    BOOL enabled = blnSMEnabled;                                                
    if (USBPDEnabled)                                                           
        return;                                                                 
    else
    {
        DisableTypeCStateMachine();                                             
        USBPDEnabled = TRUE;                                                    
        if (enabled)                                                            
            EnableTypeCStateMachine();                                          
    }
}

void DisableUSBPD(void)
{
    BOOL enabled = blnSMEnabled;                                                
    if (!USBPDEnabled)                                                          
        return;                                                                 
    else
    {
        DisableTypeCStateMachine();                                             
        USBPDEnabled = FALSE;                                                   
        if (enabled)                                                            
            EnableTypeCStateMachine();                                          
    }
}


void USBPDPolicyEngine(void)
{
    if (LastPolicyState != PolicyState)                                    
    {
        WriteStateLog(&PDStateLog, PolicyState, Timer_tms, Timer_S);
    }

    LastPolicyState = PolicyState;

    switch (PolicyState)
    {
        case peDisabled:
            break;
        case peErrorRecovery:
            PolicyErrorRecovery();
            break;
        
        case peSourceSendHardReset:
            PolicySourceSendHardReset();
            break;
        case peSourceSendSoftReset:
            PolicySourceSendSoftReset();
            break;
        case peSourceSoftReset:
            PolicySourceSoftReset();
            break;
        case peSourceStartup:
            PolicySourceStartup();
            break;
        case peSourceDiscovery:
            PolicySourceDiscovery();
            break;
        case peSourceSendCaps:
            PolicySourceSendCaps();
            break;
        case peSourceDisabled:
            PolicySourceDisabled();
            break;
        case peSourceTransitionDefault:
            PolicySourceTransitionDefault();
            break;
        case peSourceNegotiateCap:
            PolicySourceNegotiateCap();
            break;
        case peSourceCapabilityResponse:
            PolicySourceCapabilityResponse();
            break;
        case peSourceTransitionSupply:
            PolicySourceTransitionSupply();
            break;
        case peSourceReady:
            PolicySourceReady();
            break;
        case peSourceGiveSourceCaps:
            PolicySourceGiveSourceCap();
            break;
        case peSourceGetSinkCaps:
            PolicySourceGetSinkCap();
            break;
        case peSourceSendPing:
            PolicySourceSendPing();
            break;
        case peSourceGotoMin:
            PolicySourceGotoMin();
            break;
        case peSourceGiveSinkCaps:
            PolicySourceGiveSinkCap();
            break;
        case peSourceGetSourceCaps:
            PolicySourceGetSourceCap();
            break;
        case peSourceSendDRSwap:
            PolicySourceSendDRSwap();
            break;
        case peSourceEvaluateDRSwap:
            PolicySourceEvaluateDRSwap();
            break;
        case peSourceSendVCONNSwap:
            PolicySourceSendVCONNSwap();
            break;
        case peSourceSendPRSwap:
            PolicySourceSendPRSwap();
            break;
        case peSourceEvaluatePRSwap:
            PolicySourceEvaluatePRSwap();
            break;
        case peSourceWaitNewCapabilities:
            PolicySourceWaitNewCapabilities();
            break;
        
        case peSinkStartup:
            PolicySinkStartup();
            break;
        case peSinkSendHardReset:
            PolicySinkSendHardReset();
            break;
        case peSinkSoftReset:
            PolicySinkSoftReset();
            break;
        case peSinkSendSoftReset:
            PolicySinkSendSoftReset();
            break;
        case peSinkTransitionDefault:
            PolicySinkTransitionDefault();
            break;
        case peSinkDiscovery:
            PolicySinkDiscovery();
            break;
        case peSinkWaitCaps:
            PolicySinkWaitCaps();
            break;
        case peSinkEvaluateCaps:
            PolicySinkEvaluateCaps();
            break;
        case peSinkSelectCapability:
            PolicySinkSelectCapability();
            break;
        case peSinkTransitionSink:
            PolicySinkTransitionSink();
            break;
        case peSinkReady:
            PolicySinkReady();
            break;
        case peSinkGiveSinkCap:
            PolicySinkGiveSinkCap();
            break;
        case peSinkGetSourceCap:
            PolicySinkGetSourceCap();
            break;
        case peSinkGetSinkCap:
            PolicySinkGetSinkCap();
            break;
        case peSinkGiveSourceCap:
            PolicySinkGiveSourceCap();
            break;
        case peSinkSendDRSwap:
            PolicySinkSendDRSwap();
            break;
        case peSinkEvaluateDRSwap:
            PolicySinkEvaluateDRSwap();
            break;
        case peSinkEvaluateVCONNSwap:
            PolicySinkEvaluateVCONNSwap();
            break;
        case peSinkSendPRSwap:
            PolicySinkSendPRSwap();
            break;
        case peSinkEvaluatePRSwap:
            PolicySinkEvaluatePRSwap();
            break;
        case peGiveVdm:
            PolicyGiveVdm();
            break;

        
        case PE_BIST_Receive_Mode:      
            policyBISTReceiveMode();
            break;
        case PE_BIST_Frame_Received:    
            policyBISTFrameReceived();
            break;

        
        case PE_BIST_Carrier_Mode_2:     
            policyBISTCarrierMode2();
            break;

        default:
            if ((PolicyState >= FIRST_VDM_STATE) && (PolicyState <= LAST_VDM_STATE) ) {
                
                PolicyVdm();
            } else {
                
                PolicyInvalidState();
            }
            break;
    }

    USBPDTxFlag = FALSE;                                                        
}


void PolicyErrorRecovery(void)
{
    SetStateErrorRecovery();
}

void PolicySourceSendHardReset(void)
{
    PolicySendHardReset(peSourceTransitionDefault, tPSHardReset);
}

void PolicySourceSoftReset(void)
{
    PolicySendCommand(CMTAccept, peSourceSendCaps, 0);
}

void PolicySourceSendSoftReset(void)
{
    if(!manualRetries)
    {
        if(Registers.Control.N_RETRIES == 0)
        {
            Registers.Control.N_RETRIES = 3;                                    
            DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
        }
    }
    else
    {
        if(nTries !=4)
        {
            nTries = 4;                                                       
        }
    }

    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTSoftReset, peSourceSendSoftReset, 1) == STAT_SUCCESS) 
                PolicyStateTimer = tSenderResponse;                             
            break;
        default:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTAccept))  
                {
                    PolicyState = peSourceSendCaps;                             
                }
                else                                                            
                    PolicyState = peSourceSendHardReset;                        
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceSendHardReset;                            
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySourceStartup(void)
{
    INT32 i;
    if(!manualRetries)
    {
        if(Registers.Control.N_RETRIES != 0)
        {
            Registers.Control.N_RETRIES = 0;                                    
            DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
        }
    }
    else
    {
        if(nTries != 1)
        {
            nTries = 1;                                                       
        }
    }

    switch (PolicySubIndex)
    {
        case 0:
            isPartnerDRP = FALSE;
            PolicyIsSource = TRUE;                                                      
            ResetProtocolLayer(TRUE);                                                   
            PRSwapTimer = 0;                                                            
            CapsCounter = 0;                                                            
            CollisionCounter = 0;                                                       
            PolicyStateTimer = tVSafe5V;                                        
            PolicySubIndex++;
            break;
        case 1:
            if ((PolicyHasContract || (PolicyStateTimer == 0)) && (SwapSourceStartTimer == 0))
            {
                PolicySubIndex++;
            }
            break;
        case 2:
            PolicyStateTimer = 0;                                                       
            PolicyState = peSourceSendCaps;                                             
            PolicySubIndex = 0;                                                         

            AutoVdmState = AUTO_VDM_INIT;
            auto_mode_disc_tracker = 0;
            mode_entered = FALSE;
            AutoDpModeEntryObjPos = -1;

            resetDp();

            core_svid_info.num_svids = 0;
            for (i = 0; i < MAX_NUM_SVIDS; i++) {
                core_svid_info.svids[i] = 0;
            }
            break;
        default:
            PolicySubIndex = 0;
            break;
    }
}

void PolicySourceDiscovery(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            PolicyStateTimer = tTypeCSendSourceCap;                             
            PolicySubIndex++;                                                   
            break;
        default:
            if ((HardResetCounter > nHardResetCount) && (NoResponseTimer == 0) && (PolicyHasContract == TRUE))
            {                                                                   
                PolicyState = peErrorRecovery;                                  
                PolicySubIndex = 0;
            }
            else if ((HardResetCounter > nHardResetCount) && (NoResponseTimer == 0) && (PolicyHasContract == FALSE))
            {                                                                   
                    PolicyState = peSourceDisabled;                             
                    PolicySubIndex = 0;                                             
            }
            if (PolicyStateTimer == 0)                                          
            {
                if (CapsCounter > nCapsCount)                                   
                    PolicyState = peSourceDisabled;                             
                else                                                            
                    PolicyState = peSourceSendCaps;                             
                PolicySubIndex = 0;                                             
            }
            break;
    }
}

void PolicySourceSendCaps(void)
{
    if ((HardResetCounter > nHardResetCount) && (NoResponseTimer == 0))         
    {
        if (PolicyHasContract)                                                  
            PolicyState = peErrorRecovery;                                      
        else                                                                    
            PolicyState = peSourceDisabled;                                     
    }
    else                                                                        
    {
        switch (PolicySubIndex)
        {
            case 0:
                if (PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSourceSendCaps, 1, SOP_TYPE_SOP) == STAT_SUCCESS)
                {
                    HardResetCounter = 0;                                       
                    CapsCounter = 0;                                            
                    NoResponseTimer = T_TIMER_DISABLE;                          
                    PolicyStateTimer = tSenderResponse;                         
                }
                break;
            default:
                if(!manualRetries)
                {
                    if(Registers.Control.N_RETRIES == 0)
                    {
                        Registers.Control.N_RETRIES = 3;                                    
                        DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
                    }
                }
                else
                {
                    if(nTries != 4)
                    {
                        nTries = 4;                                                       
                    }
                }
                if (ProtocolMsgRx)                                              
                {
                    ProtocolMsgRx = FALSE;                                      
                    if ((PolicyRxHeader.NumDataObjects == 1) && (PolicyRxHeader.MessageType == DMTRequest)) 
                        PolicyState = peSourceNegotiateCap;                     
                    else                                                        
                        PolicyState = peSourceSendSoftReset;                    
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                }
                else if (!PolicyStateTimer)                                     
                {
                    ProtocolMsgRx = FALSE;                                      
                    PolicyState = peSourceSendHardReset;                        
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                }
                break;
        }
    }
}

void PolicySourceDisabled(void)
{
    USBPDContract.object = 0;                                                   
    
#ifdef FSC_INTERRUPT_TRIGGERED
    g_Idle = TRUE;                                                              
    Registers.Mask.byte = 0xFF;
    Registers.Mask.M_COMP_CHNG = 0;
    DeviceWrite(regMask, 1, &Registers.Mask.byte);
    Registers.MaskAdv.byte[0] = 0xFF;
    Registers.MaskAdv.M_HARDRST = 0;
    DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
    Registers.MaskAdv.M_GCRCSENT = 0;
    DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    platform_enable_timer(FALSE);
#endif

}

void PolicySourceTransitionDefault(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            platform_set_vbus_5v_enable(FALSE);                                           
            platform_set_vbus_lvl1_enable(FALSE);                                          
            USBPDContract.object = 0;                                           
            if(!PolicyIsDFP)                                                    
            {
                PolicyIsDFP = TRUE;;                                            
                Registers.Switches.DATAROLE = PolicyIsDFP;                      
                DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);      
            }
            if(IsVCONNSource)                                                   
            {
                Registers.Switches.VCONN_CC1 = 0;
                Registers.Switches.VCONN_CC2 = 0;
                DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
            }
            PolicySubIndex++;
            
            break;
        case 1:
            if(VbusVSafe0V())                                                   
            {
                PolicyStateTimer = tSrcRecover;
                PolicySubIndex++;
            }
            break;
        case 2:
            if(PolicyStateTimer == 0)                                           
            {
                PolicySubIndex++;
            }
            break;
        default:
                platform_set_vbus_5v_enable(TRUE);                              
                if(blnCCPinIsCC1)
                {
                    Registers.Switches.VCONN_CC2 = 1;
                }
                else
                {
                    Registers.Switches.VCONN_CC1 = 1;
                }
                DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
                IsVCONNSource = TRUE;
                NoResponseTimer = tNoResponse;                                  
                PolicyState = peSourceStartup;                                  
                PolicySubIndex = 0;
            break;
    }
}

void PolicySourceNegotiateCap(void)
{
    
    BOOL reqAccept = FALSE;                                                     
    UINT8 objPosition;                                                          
    objPosition = PolicyRxDataObj[0].FVRDO.ObjectPosition;                      
    if ((objPosition > 0) && (objPosition <= CapsHeaderSource.NumDataObjects))  
    {
        if ((PolicyRxDataObj[0].FVRDO.OpCurrent <= CapsSource[objPosition-1].FPDOSupply.MaxCurrent)) 
            reqAccept = TRUE;                                                   
    }
    if (reqAccept)                                                              
    {
        PolicyState = peSourceTransitionSupply;                                 

    }
    else                                                                        
        PolicyState = peSourceCapabilityResponse;                               
}

void PolicySourceTransitionSupply(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            PolicySendCommand(CMTAccept, peSourceTransitionSupply, 1);          
            break;
        case 1:
            PolicyStateTimer = tSrcTransition;                                  
            PolicySubIndex++;                                                   
            break;
        case 2:
            if (!PolicyStateTimer)                                              
                PolicySubIndex++;                                               
            break;
        case 3:
            PolicyHasContract = TRUE;                                           
            USBPDContract.object = PolicyRxDataObj[0].object;                   
            if (platform_get_vbus_lvl1_enable())                                         
            {
                PolicySubIndex = 5;                                         
            }
            else                                                            
            {
                platform_set_vbus_5v_enable( TRUE );                                    
                platform_set_vbus_lvl1_enable( TRUE );                                   
                PolicyStateTimer = VbusTransitionTime;                      
                PolicySubIndex++;                                           
            }

            if (platform_get_vbus_lvl1_enable())
            {
                platform_set_vbus_5v_enable( TRUE );                                    
                platform_set_vbus_lvl1_enable( FALSE );                                  
                PolicyStateTimer = VbusTransitionTime;                      
                PolicySubIndex++;                                           
            }
            else
            {
                PolicySubIndex = 5;                                         
            }

            break;
        case 4:
            
            if (PolicyStateTimer == 0)
                PolicySubIndex++;                                               
            break;
        default:
            PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                     
            break;
    }
}

void PolicySourceCapabilityResponse(void)
{
    if (PolicyHasContract)                                                      
    {
        if(isContractValid)
        {
            PolicySendCommand(CMTReject, peSourceReady, 0);                     
        }
        else
        {
            PolicySendCommand(CMTReject, peSourceSendHardReset, 0);                     
        }
    }
    else                                                                        
    {
        PolicySendCommand(CMTReject, peSourceWaitNewCapabilities, 0);           
    }
}

void PolicySourceReady(void)
{
    if (ProtocolMsgRx)                                                          
    {
        ProtocolMsgRx = FALSE;                                                  
        if (PolicyRxHeader.NumDataObjects == 0)                                 
        {
            switch (PolicyRxHeader.MessageType)                                 
            {
                case CMTGetSourceCap:
                    PolicyState = peSourceGiveSourceCaps;                       
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTGetSinkCap:                                             
                    PolicyState = peSourceGiveSinkCaps;                         
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTDR_Swap:                                                
                    PolicyState = peSourceEvaluateDRSwap;                       
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTPR_Swap:
                    PolicyState = peSourceEvaluatePRSwap;                       
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTSoftReset:
                    PolicyState = peSourceSoftReset;                            
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                default:                                                        
                    break;
            }
        }
        else                                                                    
        {
            switch (PolicyRxHeader.MessageType)
            {
                case DMTRequest:
                    PolicyState = peSourceNegotiateCap;                         
                    break;
                case DMTVenderDefined:
                    convertAndProcessVdmMessage(ProtocolMsgRxSop);
                    break;
                case DMTBIST:
                    processDMTBIST();
                    break;
                default:                                                        
                    break;
            }
            PolicySubIndex = 0;                                                 
            PDTxStatus = txIdle;                                                
        }
    }
    else if (USBPDTxFlag)                                                       
    {
        if (PDTransmitHeader.NumDataObjects == 0)
        {
            switch (PDTransmitHeader.MessageType)                               
            {
                case CMTGetSinkCap:
                    PolicyState = peSourceGetSinkCaps;                          
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTGetSourceCap:
                    PolicyState = peSourceGetSourceCaps;                        
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTPing:
                    PolicyState = peSourceSendPing;                             
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTGotoMin:
                    PolicyState = peSourceGotoMin;                              
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTPR_Swap:
                    if (PortType == USBTypeC_DRP)                               
                    {
                        PolicyState = peSourceSendPRSwap;                       
                        PolicySubIndex = 0;                                     
                        PDTxStatus = txIdle;                                    
                    }
                    break;
                case CMTDR_Swap:
                    if (PortType == USBTypeC_DRP)                               
                    {
                        PolicyState = peSourceSendDRSwap;                       
                        PolicySubIndex = 0;                                     
                        PDTxStatus = txIdle;                                    
                    }
                    break;
                case CMTVCONN_Swap:
                    PolicyState = peSourceSendVCONNSwap;                        
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTSoftReset:
                    PolicyState = peSourceSendSoftReset;                        
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                default:                                                        
                    break;
            }
        }
        else
        {
            switch (PDTransmitHeader.MessageType)
            {
                case DMTSourceCapabilities:
                    PolicyState = peSourceSendCaps;
                    PolicySubIndex = 0;
                    PDTxStatus = txIdle;
                    break;
                case DMTVenderDefined:
                    PolicySubIndex = 0;
                    doVdmCommand();
                    break;
                default:
                    break;
            }
        }
    }
    else if(PolicyIsDFP && (AutoVdmState != AUTO_VDM_DONE) && (GetUSBPDBufferNumBytes() == 0))
    {
        autoVdmDiscovery();
    }
    else
    {
#ifdef FSC_INTERRUPT_TRIGGERED
        g_Idle = TRUE;                                                          
        Registers.Mask.byte = 0xFF;
        Registers.Mask.M_COMP_CHNG = 0;
        DeviceWrite(regMask, 1, &Registers.Mask.byte);
        Registers.MaskAdv.byte[0] = 0xFF;
        Registers.MaskAdv.M_HARDRST = 0;
        DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
        Registers.MaskAdv.M_GCRCSENT = 0;
        DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
        platform_enable_timer(FALSE);
#endif
    }

}

void PolicySourceGiveSourceCap(void)
{
    PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSourceReady, 0, SOP_TYPE_SOP);
}

void PolicySourceGetSourceCap(void)
{
    PolicySendCommand(CMTGetSourceCap, peSourceReady, 0);
}

void PolicySourceGetSinkCap(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTGetSinkCap, peSourceGetSinkCaps, 1) == STAT_SUCCESS) 
                PolicyStateTimer = tSenderResponse;                             
            break;
        default:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if ((PolicyRxHeader.NumDataObjects > 0) && (PolicyRxHeader.MessageType == DMTSinkCapabilities))
                {
                    UpdateCapabilitiesRx(FALSE);
                    PolicyState = peSourceReady;                                
                }
                else                                                            
                {
                    PolicyState = peSourceSendHardReset;                        
                }
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceSendHardReset;                            
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySourceGiveSinkCap(void)
{
    if (PortType == USBTypeC_DRP)
        PolicySendData(DMTSinkCapabilities, CapsHeaderSink.NumDataObjects, &CapsSink[0], peSourceReady, 0, SOP_TYPE_SOP);
    else
        PolicySendCommand(CMTReject, peSourceReady, 0);                         
}

void PolicySourceSendPing(void)
{
    PolicySendCommand(CMTPing, peSourceReady, 0);
}

void PolicySourceGotoMin(void)
{
    if (ProtocolMsgRx)
    {
        ProtocolMsgRx = FALSE;                                                  
        if (PolicyRxHeader.NumDataObjects == 0)                                 
        {
            switch(PolicyRxHeader.MessageType)                                  
            {
                case CMTSoftReset:
                    PolicyState = peSourceSoftReset;                            
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                default:                                                        
                    break;
            }
        }
    }
    else
    {
        switch (PolicySubIndex)
        {
            case 0:
                PolicySendCommand(CMTGotoMin, peSourceGotoMin, 1);                  
                break;
            case 1:
                PolicyStateTimer = tSrcTransition;                                  
                PolicySubIndex++;                                                   
                break;
            case 2:
                if (!PolicyStateTimer)                                              
                    PolicySubIndex++;                                               
                break;
            case 3:
                
                PolicySubIndex++;                                                   
                break;
            case 4:
                
                PolicySubIndex++;                                                   
                break;
            default:
                PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                     
                break;
        }
    }
}

void PolicySourceSendDRSwap(void)
{
    UINT8 Status;
    switch (PolicySubIndex)
    {
        case 0:
            Status = PolicySendCommandNoReset(CMTDR_Swap, peSourceSendDRSwap, 1);   
            if (Status == STAT_SUCCESS)                                         
                PolicyStateTimer = tSenderResponse;                             
            else if (Status == STAT_ERROR)                                      
                PolicyState = peErrorRecovery;                                  
            break;
        default:
            if (ProtocolMsgRx)
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch(PolicyRxHeader.MessageType)                          
                    {
                        case CMTAccept:
                            PolicyIsDFP = !PolicyIsDFP;                         
                            Registers.Switches.DATAROLE = PolicyIsDFP;          
                            DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]); 
                            PolicyState = peSourceReady;                        
                            break;
                        case CMTSoftReset:
                            PolicyState = peSourceSoftReset;                    
                            break;
                        default:                                                
                            PolicyState = peSourceReady;                        
                            break;
                    }
                }
                else                                                            
                {
                    PolicyState = peSourceReady;                                
                }
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (PolicyStateTimer == 0)                                     
            {
                PolicyState = peSourceReady;                                    
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySourceEvaluateDRSwap(void)
{
    UINT8 Status;
    if (PortType != USBTypeC_DRP)                                               
    {
        PolicySendCommand(CMTReject, peSourceReady, 0);                         
    }
    else                                                                        
    {
        Status = PolicySendCommandNoReset(CMTAccept, peSourceReady, 0);         
        if (Status == STAT_SUCCESS)                                             
        {
            PolicyIsDFP = !PolicyIsDFP;                                         
            Registers.Switches.DATAROLE = PolicyIsDFP;                          
            DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);         
        }
        else if (Status == STAT_ERROR)                                          
        {
            PolicyState = peErrorRecovery;                                      
            PolicySubIndex = 0;                                                 
            PDTxStatus = txIdle;                                                
        }
    }
}

void PolicySourceSendVCONNSwap(void)
{
    switch(PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTVCONN_Swap, peSourceSendVCONNSwap, 1) == STAT_SUCCESS) 
                PolicyStateTimer = tSenderResponse;                             
            break;
        case 1:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTAccept:                                         
                            PolicySubIndex++;                                   
                            break;
                        case CMTWait:                                           
                        case CMTReject:
                            PolicyState = peSourceReady;                        
                            PolicySubIndex = 0;                                 
                            PDTxStatus = txIdle;                                
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceReady;                                    
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        case 2:
            if (IsVCONNSource)                                                  
            {
                PolicyStateTimer = tVCONNSourceOn;                              
                PolicySubIndex++;                                               
            }
            else                                                                
            {
                if (blnCCPinIsCC1)                                              
                    Registers.Switches.VCONN_CC2 = 1;                           
                else                                                            
                    Registers.Switches.VCONN_CC1 = 1;                           
                DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      
                IsVCONNSource = TRUE;
                PolicyStateTimer = VbusTransitionTime;                          
                PolicySubIndex = 4;                                             
            }
            break;
        case 3:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            Registers.Switches.VCONN_CC1 = 0;                   
                            Registers.Switches.VCONN_CC2 = 0;                   
                            DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]); 
                            IsVCONNSource = FALSE;
                            PolicyState = peSourceReady;                        
                            PolicySubIndex = 0;                                 
                            PDTxStatus = txIdle;                                
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceSendHardReset;                            
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        default:
            if (!PolicyStateTimer)
            {
                PolicySendCommand(CMTPS_RDY, peSourceReady, 0);                 
            }
            break;
    }
}

void PolicySourceSendPRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: 
            if (PolicySendCommand(CMTPR_Swap, peSourceSendPRSwap, 1) == STAT_SUCCESS) 
                PolicyStateTimer = tSenderResponse;                             
            break;
        case 1:  
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTAccept:                                         
                            PRSwapTimer = tPRSwapBailout;                       
                            PolicyStateTimer = tSrcTransition;                  
                            PolicySubIndex++;                                   
                            break;
                        case CMTWait:                                           
                        case CMTReject:
                            PolicyState = peSourceReady;                        
                            PolicySubIndex = 0;                                 
                            PDTxStatus = txIdle;                                
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceReady;                                    
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        case 2: 
            if (!PolicyStateTimer)
            {
                RoleSwapToAttachedSink();                                       
                PolicyStateTimer = 0;                                           
                PolicySubIndex++;                                               
            }
            break;
        case 3: 
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceSendPRSwap, 4);
                if (Status == STAT_SUCCESS)                                     
                    PolicyStateTimer = tPSSourceOn;                          
                else if (Status == STAT_ERROR)
                    PolicyState = peErrorRecovery;                              
            }
            break;
        default: 
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            PolicyState = peSinkStartup;                        
                            PolicySubIndex = 0;                                 
                            PolicyStateTimer = 0;                               
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peErrorRecovery;                                  
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySourceEvaluatePRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: 
            if (PortType != USBTypeC_DRP)                                       
            {
                PolicySendCommand(CMTReject, peSourceReady, 0);                 
            }
            else
            {
                if (PolicySendCommand(CMTAccept, peSourceEvaluatePRSwap, 1) == STAT_SUCCESS) 
                {
                    PRSwapTimer = tPRSwapBailout;                               
                    RoleSwapToAttachedSink();                                   
                    PolicyStateTimer = 0;                                       
                }
            }
            break;
        case 1: 
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceEvaluatePRSwap, 2);    
                if (Status == STAT_SUCCESS)                                     
                    PolicyStateTimer = tPSSourceOn;                          
                else if (Status == STAT_ERROR)
                    PolicyState = peErrorRecovery;                              
            }
            break;
        default: 
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            PolicyState = peSinkStartup;                        
                            PolicySubIndex = 0;                                 
                            PolicyStateTimer = 0;                               
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peErrorRecovery;                                  
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySourceWaitNewCapabilities(void)                                      
{
#ifdef FSC_INTERRUPT_TRIGGERED
    g_Idle = TRUE;                                                              
    Registers.Mask.byte = 0xFF;
    Registers.Mask.M_COMP_CHNG = 0;
    DeviceWrite(regMask, 1, &Registers.Mask.byte);
    Registers.MaskAdv.byte[0] = 0xFF;
    Registers.MaskAdv.M_HARDRST = 0;
    DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
    Registers.MaskAdv.M_GCRCSENT = 0;
    DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    platform_enable_timer(FALSE);
#endif
    switch(PolicySubIndex)
    {
        case 0:
            
            break;
        default:
            
            PolicyState = peSourceSendCaps;
            PolicySubIndex = 0;
            break;
    }
}


void PolicySinkSendHardReset(void)
{
    IsHardReset = TRUE;
    PolicySendHardReset(peSinkTransitionDefault, 0);
}

void PolicySinkSoftReset(void)
{
    if (PolicySendCommand(CMTAccept, peSinkWaitCaps, 0) == STAT_SUCCESS)
        PolicyStateTimer = tSinkWaitCap;
}

void PolicySinkSendSoftReset(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendCommand(CMTSoftReset, peSinkSendSoftReset, 1) == STAT_SUCCESS)    
                PolicyStateTimer = tSenderResponse;                             
            break;
        default:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTAccept))  
                {
                    PolicyState = peSinkWaitCaps;                               
                    PolicyStateTimer = tSinkWaitCap;                            
                }
                else                                                            
                    PolicyState = peSinkSendHardReset;                          
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSinkSendHardReset;                              
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySinkTransitionDefault(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            IsHardReset = TRUE;
            USBPDContract.object = 0;                                                   
            NoResponseTimer = tNoResponse;                                              
            if(PolicyIsDFP)                                                     
            {
                PolicyIsDFP = FALSE;                                            
                Registers.Switches.DATAROLE = PolicyIsDFP;                      
                DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);      
            }
            if(IsVCONNSource)                                                   
            {
                Registers.Switches.VCONN_CC1 = 0;
                Registers.Switches.VCONN_CC2 = 0;
                DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
                IsVCONNSource = FALSE;
            }
            PolicySubIndex++;
            break;
        case 1:
            if(VbusVSafe0V())
            {
                PolicySubIndex++;
            }
            else if (NoResponseTimer == 0)                                      
            {
                if(PolicyHasContract)
                {
                    PolicyState = peErrorRecovery;
                    PolicySubIndex = 0;
                }
                else
                {
                    PolicyState = peSinkStartup;
                    PolicySubIndex = 0;
                }
            }
            break;
        default:
            PolicyState = peSinkStartup;                                                
            PolicySubIndex = 0;                                                         
            PDTxStatus = txIdle;                                                        
            break;
    }



}

void PolicySinkStartup(void)
{
    INT32 i;
    if(!manualRetries)
    {
        if(Registers.Control.N_RETRIES == 0)
        {
            Registers.Control.N_RETRIES = 3;                                    
            DeviceWrite(regControl3, 1, &Registers.Control.byte[3]);
        }
    }
    else
    {
        if(nTries != 4)
        {
            nTries = 4;                                                         
        }
    }
    isPartnerDRP = FALSE;
    PolicyIsSource = FALSE;                                                     
    ResetProtocolLayer(TRUE);                                                   
    CapsCounter = 0;                                                            
    CollisionCounter = 0;                                                       
    PolicyStateTimer = 0;                                                       
    PolicyState = peSinkDiscovery;                                              
    PolicySubIndex = 0;                                                         

    AutoVdmState = AUTO_VDM_INIT;
    auto_mode_disc_tracker = 0;
    mode_entered = FALSE;
    AutoDpModeEntryObjPos = -1;
    resetDp();

    core_svid_info.num_svids = 0;
    for (i = 0; i < MAX_NUM_SVIDS; i++) {
        core_svid_info.svids[i] = 0;
    }
}

void PolicySinkDiscovery(void)
{
    if (!VbusUnder5V())
    {
        IsHardReset = FALSE;
        PRSwapTimer = 0;                                                        
        PolicyState = peSinkWaitCaps;
        PolicySubIndex = 0;
        PolicyStateTimer = tTypeCSinkWaitCap;
    }
    else if (NoResponseTimer == 0)
    {
        PRSwapTimer = 0;                                                        
        PolicyState = peErrorRecovery;
        PolicySubIndex = 0;
    }
}

void PolicySinkWaitCaps(void)
{
    if (ProtocolMsgRx)                                                          
    {
        ProtocolMsgRx = FALSE;                                                  
        if ((PolicyRxHeader.NumDataObjects > 0) && (PolicyRxHeader.MessageType == DMTSourceCapabilities)) 
        {
            UpdateCapabilitiesRx(TRUE);                                         
            PolicyState = peSinkEvaluateCaps;                                   
        }
        else if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
            PolicyState = peSinkSoftReset;                                      
        }
        PolicySubIndex = 0;                                                     
    }
    else if ((PolicyHasContract == TRUE) && (NoResponseTimer == 0) && (HardResetCounter > nHardResetCount))
    {
        PolicyState = peErrorRecovery;
        PolicySubIndex = 0;
    }
    else if ((PolicyStateTimer == 0) && (HardResetCounter <= nHardResetCount))
    {
        PolicyState = peSinkSendHardReset;
        PolicySubIndex = 0;
    }
    else if ((PolicyHasContract == FALSE) && (NoResponseTimer == 0) && (HardResetCounter > nHardResetCount))
    {
#ifdef FSC_INTERRUPT_TRIGGERED
        g_Idle = TRUE;                                                          
        Registers.Mask.byte = 0xFF;
        Registers.Mask.M_VBUSOK = 0;
        DeviceWrite(regMask, 1, &Registers.Mask.byte);
        Registers.MaskAdv.byte[0] = 0xFF;
        Registers.MaskAdv.M_HARDRST = 0;
        DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
        Registers.MaskAdv.M_GCRCSENT = 0;
        DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
        platform_enable_timer(FALSE);
#endif
        }

}

void PolicySinkEvaluateCaps(void)
{
    
    
    
    
    INT32 i, reqPos;
    UINT32 objVoltage = 0;
    UINT32 objCurrent, objPower, MaxPower, SelVoltage, ReqCurrent;
    NoResponseTimer = T_TIMER_DISABLE;                                          
    HardResetCounter = 0;                                                       
    SelVoltage = 0;
    MaxPower = 0;
    reqPos = 0;                                                                 
    for (i=0; i<CapsHeaderReceived.NumDataObjects; i++)                         
    {
        switch (CapsReceived[i].PDO.SupplyType)
        {
            case pdoTypeFixed:
                objVoltage = CapsReceived[i].FPDOSupply.Voltage;                
                if (objVoltage > SinkRequestMaxVoltage)                         
                    objPower = 0;                                               
                else                                                            
                {
                    objCurrent = CapsReceived[i].FPDOSupply.MaxCurrent;
                    objPower = objVoltage * objCurrent;                         
                }
                break;
            case pdoTypeVariable:
                objVoltage = CapsReceived[i].VPDO.MaxVoltage;                   
                if (objVoltage > SinkRequestMaxVoltage)                         
                    objPower = 0;                                               
                else                                                            
                {
                    objVoltage = CapsReceived[i].VPDO.MinVoltage;               
                    objCurrent = CapsReceived[i].VPDO.MaxCurrent;               
                    objPower = objVoltage * objCurrent;                         
                }
                break;
            case pdoTypeBattery:                                                
            default:                                                            
                objPower = 0;                                                   
                break;
        }
        if (objPower > MaxPower)                                                
        {
            MaxPower = objPower;                                                
            SelVoltage = objVoltage;                                            
            reqPos = i + 1;                                                     
            if ((CapsReceived[i].PDO.SupplyType == pdoTypeFixed) && (CapsReceived[i].FPDOSupply.DualRolePower == 1))                        
            {
                isPartnerDRP = TRUE;
            }
            else
            {
                isPartnerDRP = FALSE;
            }
        }
    }
    if ((reqPos > 0) && (SelVoltage > 0))
    {
        SinkRequest.FVRDO.ObjectPosition = reqPos & 0x07;                       
        SinkRequest.FVRDO.GiveBack = SinkGotoMinCompatible;                     
        SinkRequest.FVRDO.NoUSBSuspend = SinkUSBSuspendOperation;               
        SinkRequest.FVRDO.USBCommCapable = SinkUSBCommCapable;                  
        ReqCurrent = SinkRequestOpPower / SelVoltage;                           
        SinkRequest.FVRDO.OpCurrent = (ReqCurrent & 0x3FF);                     
        ReqCurrent = SinkRequestMaxPower / SelVoltage;                          
        SinkRequest.FVRDO.MinMaxCurrent = (ReqCurrent & 0x3FF);                 
        if (SinkGotoMinCompatible)                                              
            SinkRequest.FVRDO.CapabilityMismatch = FALSE;                       
        else                                                                    
        {
            if (MaxPower < SinkRequestMaxPower)                                 
                SinkRequest.FVRDO.CapabilityMismatch = TRUE;                    
            else                                                                
                SinkRequest.FVRDO.CapabilityMismatch = FALSE;                   
        }
        PolicyState = peSinkSelectCapability;                                   
        PolicySubIndex = 0;                                                     
        PolicyStateTimer = tSenderResponse;                                     
    }
    else
    {
        
        PolicyState = peSinkWaitCaps;                                           
        PolicyStateTimer = tTypeCSinkWaitCap;                                        
    }
}

void PolicySinkSelectCapability(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            if (PolicySendData(DMTRequest, 1, &SinkRequest, peSinkSelectCapability, 1, SOP_TYPE_SOP) == STAT_SUCCESS)
            {
                NoResponseTimer = tSenderResponse;                                   
            }
            break;
       case 1:
            if (ProtocolMsgRx)
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch(PolicyRxHeader.MessageType)                          
                    {
                        case CMTAccept:
                            PolicyHasContract = TRUE;                           
                            USBPDContract.object = SinkRequest.object;          
                            PolicyStateTimer = tPSTransition;                   
                            PolicyState = peSinkTransitionSink;                 
                            break;
                        case CMTWait:
                        case CMTReject:
                            if(PolicyHasContract)
                            {
                                PolicyState = peSinkReady;                      
                            }
                            else
                            {
                                PolicyState = peSinkWaitCaps;                   
                                HardResetCounter = nHardResetCount + 1;         
                            }
                            break;
                        case CMTSoftReset:
                            PolicyState = peSinkSoftReset;                      
                            break;
                        default:
                            PolicyState = peSinkSendSoftReset;                  
                            break;
                    }
                }
                else                                                            
                {
                    switch (PolicyRxHeader.MessageType)
                    {
                        case DMTSourceCapabilities:                             
                            UpdateCapabilitiesRx(TRUE);                         
                            PolicyState = peSinkEvaluateCaps;                   
                            break;
                        default:
                            PolicyState = peSinkSendSoftReset;                  
                            break;
                    }
                }
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (PolicyStateTimer == 0)                                     
            {
                PolicyState = peSinkSendHardReset;                              
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySinkTransitionSink(void)
{
    if (ProtocolMsgRx)
    {
        ProtocolMsgRx = FALSE;                                                  
        if (PolicyRxHeader.NumDataObjects == 0)                                 
        {
            switch(PolicyRxHeader.MessageType)                                  
            {
                case CMTPS_RDY:
                    PolicyState = peSinkReady;                                  
                    break;
                case CMTSoftReset:
                    PolicyState = peSinkSoftReset;                              
                    break;
                default:
                    PolicyState = peSinkSendSoftReset;                          
                    break;
            }
        }
        else                                                                    
        {
            switch (PolicyRxHeader.MessageType)                                 
            {
                case DMTSourceCapabilities:                                     
                    UpdateCapabilitiesRx(TRUE);                                 
                    PolicyState = peSinkEvaluateCaps;                           
                    break;
                default:                                                        
                    PolicyState = peSinkSendSoftReset;                          
                    break;
            }
        }
        PolicySubIndex = 0;                                                     
        PDTxStatus = txIdle;                                                    
    }
    else if (PolicyStateTimer == 0)                                             
    {
        PolicyState = peSinkSendHardReset;                                      
        PolicySubIndex = 0;                                                     
        PDTxStatus = txIdle;                                                    
    }
}

void PolicySinkReady(void)
{
    if (ProtocolMsgRx)                                                          
    {
        ProtocolMsgRx = FALSE;                                                  
        if (PolicyRxHeader.NumDataObjects == 0)                                 
        {
            switch (PolicyRxHeader.MessageType)                                 
            {
                case CMTGotoMin:
                    PolicyState = peSinkTransitionSink;                         
                    PolicyStateTimer = tPSTransition;                           
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTGetSinkCap:
                    PolicyState = peSinkGiveSinkCap;                            
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTGetSourceCap:
                    PolicyState = peSinkGiveSourceCap;                          
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTDR_Swap:                                                
                    PolicyState = peSinkEvaluateDRSwap;                         
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTPR_Swap:
                    PolicyState = peSinkEvaluatePRSwap;                         
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTVCONN_Swap:                                             
                    PolicyState = peSinkEvaluateVCONNSwap;                      
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTSoftReset:
                    PolicyState = peSinkSoftReset;                              
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                default:                                                        
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
            }
        }
        else
        {
            switch (PolicyRxHeader.MessageType)
            {
                case DMTSourceCapabilities:
                    UpdateCapabilitiesRx(TRUE);                                 
                    PolicyState = peSinkEvaluateCaps;                           
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case DMTVenderDefined:
                    convertAndProcessVdmMessage(ProtocolMsgRxSop);
                    break;
                case DMTBIST:
                    processDMTBIST();
                    break;
                default:                                                        
                    PolicySubIndex = 0;                                                     
                    PDTxStatus = txIdle;                                                    
                    break;
            }
        }
    }
    else if (USBPDTxFlag)                                                       
    {
        if (PDTransmitHeader.NumDataObjects == 0)
        {
            switch (PDTransmitHeader.MessageType)
            {
                case CMTGetSourceCap:
                    PolicyState = peSinkGetSourceCap;                           
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                   break;
                case CMTGetSinkCap:
                    PolicyState = peSinkGetSinkCap;                             
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                case CMTDR_Swap:
                    if (PortType == USBTypeC_DRP)                               
                    {
                        PolicyState = peSinkSendDRSwap;                         
                        PolicySubIndex = 0;                                     
                        PDTxStatus = txIdle;                                    
                    }
                    break;
                case CMTPR_Swap:
                    if (PortType == USBTypeC_DRP)                               
                    {
                        PolicyState = peSinkSendPRSwap;                         
                        PolicySubIndex = 0;                                     
                        PDTxStatus = txIdle;                                    
                    }
                    break;
                case CMTSoftReset:
                    PolicyState = peSinkSendSoftReset;                          
                    PolicySubIndex = 0;                                         
                    PDTxStatus = txIdle;                                        
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch (PDTransmitHeader.MessageType)
            {
                case DMTRequest:
                    SinkRequest.object = PDTransmitObjects[0].object;           
                    PolicyState = peSinkSelectCapability;                       
                    PolicySubIndex = 0;                                         
                    PolicyStateTimer = tSenderResponse;                         
                    break;
                case DMTVenderDefined:
                    doVdmCommand();
                    break;
                default:
                    break;
            }
        }
    }
    else if(PolicyIsDFP && (AutoVdmState != AUTO_VDM_DONE) && (GetUSBPDBufferNumBytes() == 0))
    {
        autoVdmDiscovery();
    }
    else
    {
#ifdef FSC_INTERRUPT_TRIGGERED
        g_Idle = TRUE;                                                          
        Registers.Mask.byte = 0xFF;
        Registers.Mask.M_VBUSOK = 0;
        DeviceWrite(regMask, 1, &Registers.Mask.byte);
        Registers.MaskAdv.byte[0] = 0xFF;
        Registers.MaskAdv.M_HARDRST = 0;
        DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
        Registers.MaskAdv.M_GCRCSENT = 0;
        DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
        platform_enable_timer(FALSE);
#endif
    }

}

void PolicySinkGiveSinkCap(void)
{
    PolicySendData(DMTSinkCapabilities, CapsHeaderSink.NumDataObjects, &CapsSink[0], peSinkReady, 0, SOP_TYPE_SOP);
}

void PolicySinkGetSinkCap(void)
{
    PolicySendCommand(CMTGetSinkCap, peSinkReady, 0);
}

void PolicySinkGiveSourceCap(void)
{
    if (PortType == USBTypeC_DRP)
        PolicySendData(DMTSourceCapabilities, CapsHeaderSource.NumDataObjects, &CapsSource[0], peSinkReady, 0, SOP_TYPE_SOP);
    else
        PolicySendCommand(CMTReject, peSinkReady, 0);                           
}

void PolicySinkGetSourceCap(void)
{
    PolicySendCommand(CMTGetSourceCap, peSinkReady, 0);
}

void PolicySinkSendDRSwap(void)
{
    UINT8 Status;
    switch (PolicySubIndex)
    {
        case 0:
            Status = PolicySendCommandNoReset(CMTDR_Swap, peSinkSendDRSwap, 1); 
            if (Status == STAT_SUCCESS)                                         
                PolicyStateTimer = tSenderResponse;                             
            else if (Status == STAT_ERROR)                                      
                PolicyState = peErrorRecovery;                                  
            break;
        default:
            if (ProtocolMsgRx)
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch(PolicyRxHeader.MessageType)                          
                    {
                        case CMTAccept:
                            PolicyIsDFP = !PolicyIsDFP;                         
                            Registers.Switches.DATAROLE = PolicyIsDFP;          
                            DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]); 
                            PolicyState = peSinkReady;                          
                            break;
                        case CMTSoftReset:
                            PolicyState = peSinkSoftReset;                      
                            break;
                        default:                                                
                            PolicyState = peSinkReady;                          
                            break;
                    }
                }
                else                                                            
                {
                    PolicyState = peSinkReady;                                  
                }
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            else if (PolicyStateTimer == 0)                                     
            {
                PolicyState = peSinkReady;                                      
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
}

void PolicySinkEvaluateDRSwap(void)
{
    UINT8 Status;
    if (PortType != USBTypeC_DRP)                                               
    {
        PolicySendCommand(CMTReject, peSinkReady, 0);                           
    }
    else                                                                        
    {
        Status = PolicySendCommandNoReset(CMTAccept, peSinkReady, 0);           
        if (Status == STAT_SUCCESS)                                             
        {
            PolicyIsDFP = !PolicyIsDFP;                                         
            Registers.Switches.DATAROLE = PolicyIsDFP;                          
            DeviceWrite(regSwitches1, 1, &Registers.Switches.byte[1]);         
        }
        else if (Status == STAT_ERROR)                                          
        {
            PolicyState = peErrorRecovery;                                      
            PolicySubIndex = 0;                                                 
            PDTxStatus = txIdle;                                                
        }
    }
}

void PolicySinkEvaluateVCONNSwap(void)
{
    switch(PolicySubIndex)
    {
        case 0:
            PolicySendCommand(CMTAccept, peSinkEvaluateVCONNSwap, 1);           
            break;
        case 1:
            if (IsVCONNSource)                                                  
            {
                PolicyStateTimer = tVCONNSourceOn;                              
                PolicySubIndex++;                                               
            }
            else                                                                
            {
                if (blnCCPinIsCC1)                                              
                {
                    Registers.Switches.VCONN_CC2 = 1;                           
                    Registers.Switches.PDWN2 = 0;                               
                }
                else                                                            
                {
                    Registers.Switches.VCONN_CC1 = 1;                           
                    Registers.Switches.PDWN1 = 0;                               
                }
                DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      
                IsVCONNSource = TRUE;
                PolicyStateTimer = VbusTransitionTime;                          
                PolicySubIndex = 3;                                             
            }
            break;
        case 2:
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            Registers.Switches.VCONN_CC1 = 0;                   
                            Registers.Switches.VCONN_CC2 = 0;                   
                            Registers.Switches.PDWN1 = 1;                       
                            Registers.Switches.PDWN2 = 1;                       
                            DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]); 
                            IsVCONNSource = FALSE;
                            PolicyState = peSinkReady;                          
                            PolicySubIndex = 0;                                 
                            PDTxStatus = txIdle;                                
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSourceSendHardReset;                            
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        default:
            if (!PolicyStateTimer)
            {
                PolicySendCommand(CMTPS_RDY, peSinkReady, 0);                       
            }
            break;
    }
}

void PolicySinkSendPRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: 
            if (PolicySendCommand(CMTPR_Swap, peSinkSendPRSwap, 1) == STAT_SUCCESS) 
                PolicyStateTimer = tSenderResponse;                             
            break;
        case 1:  
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTAccept:                                         
                            PRSwapTimer = tPRSwapBailout;                       
                            PolicyStateTimer = tPSSourceOff;                 
                            PolicySubIndex++;                                   
                            break;
                        case CMTWait:                                           
                        case CMTReject:
                            PolicyState = peSinkReady;                          
                            PolicySubIndex = 0;                                 
                            PDTxStatus = txIdle;                                
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peSinkReady;                                      
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        case 2:     
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            RoleSwapToAttachedSource();                         
                            PolicyStateTimer = tSourceOnDelay;                  
                            PolicySubIndex++;                                   
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peErrorRecovery;                                  
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        default: 
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup, 0);   
                if (Status == STAT_ERROR)
                    PolicyState = peErrorRecovery;                              
                SwapSourceStartTimer = tSwapSourceStart;
            }
            break;
    }
}

void PolicySinkEvaluatePRSwap(void)
{
    UINT8 Status;
    switch(PolicySubIndex)
    {
        case 0: 
            if ((PortType != USBTypeC_DRP) || (isPartnerDRP == FALSE))                                       
            {
                PolicySendCommand(CMTReject, peSinkReady, 0);                   
            }
            else
            {
                if (PolicySendCommand(CMTAccept, peSinkEvaluatePRSwap, 1) == STAT_SUCCESS) 
                {
                    PRSwapTimer = tPRSwapBailout;                               
                    PolicyStateTimer = tPSSourceOff;                         
                }
            }
            break;
        case 1: 
            if (ProtocolMsgRx)                                                  
            {
                ProtocolMsgRx = FALSE;                                          
                if (PolicyRxHeader.NumDataObjects == 0)                         
                {
                    switch (PolicyRxHeader.MessageType)                         
                    {
                        case CMTPS_RDY:                                         
                            RoleSwapToAttachedSource();                         
                            PolicyStateTimer = tSourceOnDelay;                  
                            PolicySubIndex++;                                   
                            break;
                        default:                                                
                            break;
                    }
                }
            }
            else if (!PolicyStateTimer)                                         
            {
                PolicyState = peErrorRecovery;                                  
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
        default:    
            if (!PolicyStateTimer)
            {
                Status = PolicySendCommandNoReset(CMTPS_RDY, peSourceStartup, 0);   
                if (Status == STAT_ERROR)
                    PolicyState = peErrorRecovery;                              
                SwapSourceStartTimer = tSwapSourceStart;
            }
            break;
    }
}

void PolicyGiveVdm(void) {
    if (ProtocolMsgRx)                                                          
    {
        sendVdmMessageFailed();                                                 
        PolicySubIndex = 0;                                                     
        PDTxStatus = txIdle;                                                    
    } else if (sendingVdmData) {
        UINT8 result = PolicySendData(DMTVenderDefined, vdm_msg_length, vdm_msg_obj, vdm_next_ps, 0, SOP_TYPE_SOP);
        if (result == STAT_SUCCESS) {
            startVdmTimer(PolicyState);
            sendingVdmData = FALSE;
        } else if (result == STAT_ERROR) {
            sendVdmMessageFailed();
            sendingVdmData = FALSE;
        }
    } else {
        sendVdmMessageFailed();
    }

    if (VdmTimerStarted && (VdmTimer == 0)) {
        vdmMessageTimeout();
    }
}

void PolicyVdm (void) {

    if (ProtocolMsgRx)                                                          
    {
        ProtocolMsgRx = FALSE;                                                  
        if (PolicyRxHeader.NumDataObjects != 0) {                               
            switch (PolicyRxHeader.MessageType)
            {
                case DMTVenderDefined:
                    convertAndProcessVdmMessage(ProtocolMsgRxSop);
                    break;
                default:                                                        
                    resetPolicyState();                                    
                    ProtocolMsgRx = TRUE;                                       
                    break;
            }
        } else {
            resetPolicyState();                                            
            ProtocolMsgRx = TRUE;                                               
        }
        PolicySubIndex = 0;                                                     
        PDTxStatus = txIdle;                                                    
    } else {
        if (sendingVdmData) {
            UINT8 result = PolicySendData(DMTVenderDefined, vdm_msg_length, vdm_msg_obj, vdm_next_ps, 0, SOP_TYPE_SOP);
            if (result == STAT_SUCCESS || result == STAT_ERROR)
            {
                sendingVdmData = FALSE;
            }
        }
    }

    if (VdmTimerStarted && (VdmTimer == 0)) {
        vdmMessageTimeout();
    }
}

void PolicyInvalidState (void) {
    
    if (PolicyIsSource) {
        PolicyState = peSourceSendHardReset;
    } else {
        PolicyState = peSinkSendHardReset;
    }
}


BOOL PolicySendHardReset(PolicyState_t nextState, UINT32 delay)
{
    BOOL Success = FALSE;
    switch (PolicySubIndex)
    {
        case 0:
            switch (PDTxStatus)
            {
                case txReset:
                case txWait:
                    
                    
                    break;
                case txSuccess:
                    PolicyStateTimer = delay;                                   
                    PolicySubIndex++;                                           
                    Success = TRUE;
                    break;
                default:                                                        
                    PDTxStatus = txReset;                                       
                    break;
            }
            break;
        default:
            if (PolicyStateTimer == 0)                                          
            {
                HardResetCounter++;                                             
                PolicyState = nextState;                                        
                PolicySubIndex = 0;                                             
                PDTxStatus = txIdle;                                            
            }
            break;
    }
    return Success;
}

UINT8 PolicySendCommand(UINT8 Command, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    switch (PDTxStatus)
    {
        case txIdle:
            PolicyTxHeader.word = 0;                                            
            PolicyTxHeader.NumDataObjects = 0;                                  
            PolicyTxHeader.MessageType = Command & 0x0F;                        
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         
            PDTxStatus = txSend;                                                
            break;
        case txSend:
        case txBusy:
        case txWait:
            
            
            break;
        case txSuccess:
            PolicyState = nextState;                                            
            PolicySubIndex = subIndex;
            PDTxStatus = txIdle;                                                
            Status = STAT_SUCCESS;
            break;
        case txError:                                                           
            if (PolicyState == peSourceSendSoftReset)                           
                PolicyState = peSourceSendHardReset;                            
            else if (PolicyState == peSinkSendSoftReset)                        
                PolicyState = peSinkSendHardReset;                              
            else if (PolicyIsSource)                                            
                PolicyState = peSourceSendSoftReset;                            
            else                                                                
                PolicyState = peSinkSendSoftReset;                              
            PolicySubIndex = 0;                                                 
            PDTxStatus = txIdle;                                                
            Status = STAT_ERROR;
            break;
        case txCollision:
            CollisionCounter++;                                                 
            if (CollisionCounter > nRetryCount)                                 
            {
                if (PolicyIsSource)
                    PolicyState = peSourceSendHardReset;                        
                else
                    PolicyState = peSinkSendHardReset;                          
                PolicySubIndex = 0;                                             
                PDTxStatus = txReset;                                           
                Status = STAT_ERROR;
            }
            else                                                                
                PDTxStatus = txIdle;                                            
            break;
        default:                                                                
            if (PolicyIsSource)
                PolicyState = peSourceSendHardReset;                            
            else
                PolicyState = peSinkSendHardReset;                              
            PolicySubIndex = 0;                                                 
            PDTxStatus = txReset;                                               
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

UINT8 PolicySendCommandNoReset(UINT8 Command, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    switch (PDTxStatus)
    {
        case txIdle:
            PolicyTxHeader.word = 0;                                            
            PolicyTxHeader.NumDataObjects = 0;                                  
            PolicyTxHeader.MessageType = Command & 0x0F;                        
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         
            PDTxStatus = txSend;                                                
            break;
        case txSend:
        case txBusy:
        case txWait:
            
            
            break;
        case txSuccess:
            PolicyState = nextState;                                            
            PolicySubIndex = subIndex;
            PDTxStatus = txIdle;                                                
            Status = STAT_SUCCESS;
            break;
        default:                                                                
            PolicyState = peErrorRecovery;                                      
            PolicySubIndex = 0;                                                 
            PDTxStatus = txReset;                                               
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

UINT8 PolicySendData(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex, SopType sop)
{
    UINT8 Status = STAT_BUSY;
    UINT32 i;
    switch (PDTxStatus)
    {
        case txIdle:
            if (NumDataObjects > 7)
                NumDataObjects = 7;
            PolicyTxHeader.word = 0x0000;                                       

            PolicyTxHeader.NumDataObjects = NumDataObjects;                     
            PolicyTxHeader.MessageType = MessageType & 0x0F;                    
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         
            for (i=0; i<NumDataObjects; i++)                                    
                PolicyTxDataObj[i].object = DataObjects[i].object;              
            if (PolicyState == peSourceSendCaps)                                
                CapsCounter++;                                                  
            PDTxStatus = txSend;                                                
            break;
        case txSend:
        case txBusy:
        case txWait:
        case txCollision:
            
            
            break;
        case txSuccess:
            PolicyState = nextState;                                            
            PolicySubIndex = subIndex;
            PDTxStatus = txIdle;                                                
            Status = STAT_SUCCESS;
            break;
        case txError:                                                           
            if (PolicyState == peSourceSendCaps)                                
                PolicyState = peSourceDiscovery;                                
            else if (PolicyIsSource)                                            
                PolicyState = peSourceSendSoftReset;                            
            else                                                                
                PolicyState = peSinkSendSoftReset;                              
            PolicySubIndex = 0;                                                 
            PDTxStatus = txIdle;                                                
            Status = STAT_ERROR;
            break;
        default:                                                                
            if (PolicyIsSource)
                PolicyState = peSourceSendHardReset;                            
            else
                PolicyState = peSinkSendHardReset;                              
            PolicySubIndex = 0;                                                 
            PDTxStatus = txReset;                                               
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

UINT8 PolicySendDataNoReset(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex)
{
    UINT8 Status = STAT_BUSY;
    UINT32 i;
    switch (PDTxStatus)
    {
        case txIdle:
            if (NumDataObjects > 7)
                NumDataObjects = 7;
            PolicyTxHeader.word = 0x0000;                                       
            PolicyTxHeader.NumDataObjects = NumDataObjects;                     
            PolicyTxHeader.MessageType = MessageType & 0x0F;                    
            PolicyTxHeader.PortDataRole = PolicyIsDFP;                          
            PolicyTxHeader.PortPowerRole = PolicyIsSource;                      
            PolicyTxHeader.SpecRevision = USBPDSPECREV;                         
            for (i=0; i<NumDataObjects; i++)                                    
                PolicyTxDataObj[i].object = DataObjects[i].object;              
            if (PolicyState == peSourceSendCaps)                                
                CapsCounter++;                                                  
            PDTxStatus = txSend;                                                
            break;
        case txSend:
        case txBusy:
        case txWait:
            
            
            break;
        case txSuccess:
            PolicyState = nextState;                                            
            PolicySubIndex = subIndex;
            PDTxStatus = txIdle;                                                
            Status = STAT_SUCCESS;
            break;
        default:                                                                
            PolicyState = peErrorRecovery;                                      
            PolicySubIndex = 0;                                                 
            PDTxStatus = txReset;                                               
            Status = STAT_ERROR;
            break;
    }
    return Status;
}

void UpdateCapabilitiesRx(BOOL IsSourceCaps)
{
    UINT32 i;
    SourceCapsUpdated = IsSourceCaps;                                           
    CapsHeaderReceived.word = PolicyRxHeader.word;                              
    for (i=0; i<CapsHeaderReceived.NumDataObjects; i++)                         
        CapsReceived[i].object = PolicyRxDataObj[i].object;                     
    for (i=CapsHeaderReceived.NumDataObjects; i<7; i++)                         
        CapsReceived[i].object = 0;                                             
}


void policyBISTReceiveMode(void)    
{
    
    
    
}

void policyBISTFrameReceived(void)  
{
    
    
    
}


void policyBISTCarrierMode2(void)
{
    switch (PolicySubIndex)
    {
        case 0:
            Registers.Control.BIST_MODE2 = 1;                                           
            DeviceWrite(regControl1, 1, &Registers.Control.byte[1]);
            Registers.Control.TX_START = 1;                                             
            DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    
            Registers.Control.TX_START = 0;                                             
            PolicyStateTimer = tBISTContMode;                                        
            PolicySubIndex = 1;
            break;
        case 1:
            if(PolicyStateTimer == 0)                                                   
            {
                Registers.Control.BIST_MODE2 = 0;                                           
                DeviceWrite(regControl1, 1, &Registers.Control.byte[1]);
                if (PolicyIsSource)                                                     
                {
                    PolicySourceSendHardReset();                                      
                    
                }
                else                                                                    
                {
                    PolicyState = peSinkTransitionDefault;                              
                }
            }
    }

}

void WriteSourceCapabilities(UINT8* abytData)
{
    UINT32 i, j;
    sopMainHeader_t Header;
    Header.byte[0] = *abytData++;                                               
    Header.byte[1] = *abytData++;                                               
    if ((Header.NumDataObjects > 0) && (Header.MessageType == DMTSourceCapabilities))   
    {
        CapsHeaderSource.word = Header.word;                                    
        for (i=0; i<CapsHeaderSource.NumDataObjects; i++)                       
        {
            for (j=0; j<4; j++)                                                 
                CapsSource[i].byte[j] = *abytData++;                            
        }
        if (PolicyIsSource)                                                     
        {
            PDTransmitHeader.word = CapsHeaderSource.word;                      
            USBPDTxFlag = TRUE;                                                 
            SourceCapsUpdated = TRUE;                                           
        }
    }
}

void ReadSourceCapabilities(UINT8* abytData)
{
    UINT32 i, j;
    *abytData++ = CapsHeaderSource.byte[0];
    *abytData++ = CapsHeaderSource.byte[1];
    for (i=0; i<CapsHeaderSource.NumDataObjects; i++)
    {
        for (j=0; j<4; j++)
            *abytData++ = CapsSource[i].byte[j];
    }
}

void WriteSinkCapabilities(UINT8* abytData)
{
    UINT32 i, j;
    sopMainHeader_t Header;
    Header.byte[0] = *abytData++;                                               
    Header.byte[1] = *abytData++;                                               
    if ((Header.NumDataObjects > 0) && (Header.MessageType == DMTSinkCapabilities))   
    {
        CapsHeaderSink.word = Header.word;                                      
        for (i=0; i<CapsHeaderSink.NumDataObjects; i++)                         
        {
            for (j=0; j<4; j++)                                                 
                CapsSink[i].byte[j] = *abytData++;                              
        }
        
    }
}

void ReadSinkCapabilities(UINT8* abytData)
{
    UINT32 i, j;
    *abytData++ = CapsHeaderSink.byte[0];
    *abytData++ = CapsHeaderSink.byte[1];
    for (i=0; i<CapsHeaderSink.NumDataObjects; i++)
    {
        for (j=0; j<4; j++)
            *abytData++ = CapsSink[i].byte[j];
    }
}

void WriteSinkRequestSettings(UINT8* abytData)
{
    UINT32 uintPower;
    SinkGotoMinCompatible = *abytData & 0x01 ? TRUE : FALSE;
    SinkUSBSuspendOperation = *abytData & 0x02 ? TRUE : FALSE;
    SinkUSBCommCapable = *abytData++ & 0x04 ? TRUE : FALSE;
    SinkRequestMaxVoltage = (UINT32) *abytData++;
    SinkRequestMaxVoltage |= ((UINT32) (*abytData++) << 8);                     
    uintPower = (UINT32) *abytData++;
    uintPower |= ((UINT32) (*abytData++) << 8);
    uintPower |= ((UINT32) (*abytData++) << 16);
    uintPower |= ((UINT32) (*abytData++) << 24);
    SinkRequestOpPower = uintPower;                                             
    uintPower = (UINT32) *abytData++;
    uintPower |= ((UINT32) (*abytData++) << 8);
    uintPower |= ((UINT32) (*abytData++) << 16);
    uintPower |= ((UINT32) (*abytData++) << 24);
    SinkRequestMaxPower = uintPower;                                            
    
}

void ReadSinkRequestSettings(UINT8* abytData)
{
    *abytData = SinkGotoMinCompatible ? 0x01 : 0;
    *abytData |= SinkUSBSuspendOperation ? 0x02 : 0;
    *abytData++ |= SinkUSBCommCapable ? 0x04 : 0;
    *abytData++ = (UINT8) (SinkRequestMaxVoltage & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxVoltage & 0xFF) >> 8);
    *abytData++ = (UINT8) (SinkRequestOpPower & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 8) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 16) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestOpPower >> 24) & 0xFF);
    *abytData++ = (UINT8) (SinkRequestMaxPower & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 8) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 16) & 0xFF);
    *abytData++ = (UINT8) ((SinkRequestMaxPower >> 24) & 0xFF);
}

void SendUSBPDHardReset(void)
{
    if (PolicyIsSource)                                                         
        PolicyState = peSourceSendHardReset;                                    
    else                                                                        
        PolicyState = peSinkSendHardReset;                                      
    PolicySubIndex = 0;
    PDTxStatus = txIdle;                                                        
}

void InitializeVdmManager(void)
{
	initializeVdm();

	
	vdmm.req_id_info 		= &vdmRequestIdentityInfo;
	vdmm.req_svid_info 		= &vdmRequestSvidInfo;
	vdmm.req_modes_info 	= &vdmRequestModesInfo;
	vdmm.enter_mode_result  = &vdmEnterModeResult;
	vdmm.exit_mode_result   = &vdmExitModeResult;
	vdmm.inform_id 			= &vdmInformIdentity;
	vdmm.inform_svids 		= &vdmInformSvids;
	vdmm.inform_modes 		= &vdmInformModes;
	vdmm.inform_attention   = &vdmInformAttention;
	vdmm.req_mode_entry		= &vdmModeEntryRequest;
	vdmm.req_mode_exit		= &vdmModeExitRequest;
}

void convertAndProcessVdmMessage(SopType sop)
{
    UINT32 i;
    
    
    UINT32 vdm_arr[7];
    for (i = 0; i < PolicyRxHeader.NumDataObjects; i++) {
        vdm_arr[i] = 0;
        vdm_arr[i] = PolicyRxDataObj[i].object;
    }

    processVdmMessage(sop, vdm_arr, PolicyRxHeader.NumDataObjects);
}

BOOL sendVdmMessage(SopType sop, UINT32* arr, UINT32 length, PolicyState_t next_ps) {
    UINT32 i;

    
    
    vdm_msg_length = length;
    vdm_next_ps = next_ps;
    for (i = 0; i < vdm_msg_length; i++) {
        vdm_msg_obj[i].object = arr[i];
    }
    sendingVdmData = TRUE;
    ProtocolCheckRxBeforeTx = TRUE;
    PolicyState = peGiveVdm;
    return 0;
}

void doVdmCommand(void)
{
    UINT32 command;
    UINT32 svid;
    UINT32 mode_index;
    SopType sop;


    command = PDTransmitObjects[0].byte[0] & 0x1F;
    svid = 0;
    svid |= (PDTransmitObjects[0].byte[3] << 8);
    svid |= (PDTransmitObjects[0].byte[2] << 0);

    mode_index = 0;
    mode_index = PDTransmitObjects[0].byte[1] & 0x7;

    
    sop = SOP_TYPE_SOP;

    if (svid == DP_SID) {
        if (command == DP_COMMAND_STATUS) {
            requestDpStatus();
        } else if (command == DP_COMMAND_CONFIG) {
            DisplayPortConfig_t temp;
            temp.word = PDTransmitObjects[1].object;
            requestDpConfig(temp);
        }
    }

    if (command == DISCOVER_IDENTITY) {
        requestDiscoverIdentity(sop);
    } else if (command == DISCOVER_SVIDS) {
        requestDiscoverSvids(sop);
    } else if (command == DISCOVER_MODES) {
        requestDiscoverModes(sop, svid);
    } else if (command == ENTER_MODE) {
        requestEnterMode(sop, svid, mode_index);
    } else if (command == EXIT_MODE) {
        requestExitMode(sop, svid, mode_index);
    }

}

SopType TokenToSopType(UINT8 data)
{
    SopType ret;
    
    if ((data & 0b11100000) == 0b11100000) {
        ret = SOP_TYPE_SOP;
    } else if ((data & 0b11100000) == 0b11000000) {
        ret = SOP_TYPE_SOP1;
    } else if ((data & 0b11100000) == 0b10100000) {
        ret = SOP_TYPE_SOP2;
    } else if ((data & 0b11100000) == 0b10000000) {
        ret = SOP_TYPE_SOP1_DEBUG;
    } else if ((data & 0b11100000) == 0b01100000) {
        ret = SOP_TYPE_SOP2_DEBUG;
    } else {
        ret = -1;
    }
    return ret;
}

void autoVdmDiscovery (void)
{
    
    if (GetUSBPDBufferNumBytes() != 0) return;

    if (!PolicyIsDFP) return; 

    if (PDTxStatus == txIdle) { 
        switch (AutoVdmState) {
            case AUTO_VDM_INIT:
            case AUTO_VDM_DISCOVER_ID_PP:
                requestDiscoverIdentity(SOP_TYPE_SOP);
                AutoVdmState = AUTO_VDM_DISCOVER_SVIDS_PP;
                break;
            case AUTO_VDM_DISCOVER_SVIDS_PP:
                requestDiscoverSvids(SOP_TYPE_SOP);
                AutoVdmState = AUTO_VDM_DISCOVER_MODES_PP;
                break;
            case AUTO_VDM_DISCOVER_MODES_PP:
                if ((auto_mode_disc_tracker == core_svid_info.num_svids)) {
                    AutoVdmState = AUTO_VDM_ENTER_DP_MODE_PP;
                    auto_mode_disc_tracker = 0;
                } else {
                    requestDiscoverModes(SOP_TYPE_SOP, core_svid_info.svids[auto_mode_disc_tracker]);
                    auto_mode_disc_tracker++;
                }
                break;
            case AUTO_VDM_ENTER_DP_MODE_PP:
                if (AutoDpModeEntryObjPos > 0) {
                    requestEnterMode(SOP_TYPE_SOP, DP_SID, AutoDpModeEntryObjPos);
                    AutoVdmState = AUTO_VDM_DP_GET_STATUS;
                } else {
                    AutoVdmState = AUTO_VDM_DONE;
                }
                break;
            case AUTO_VDM_DP_GET_STATUS:
                if (DpModeEntered) {
                    requestDpStatus();
                }
                AutoVdmState = AUTO_VDM_DONE;
                break;
            default:
                AutoVdmState = AUTO_VDM_DONE;
                break;
        }
    }
}

void resetLocalHardware(void)
{
    UINT8 data = 0x20;
    DeviceWrite(regReset, 1, &data);   

    DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);  
    DeviceRead(regSlice, 1, &Registers.Slice.byte);
    DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
    DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
    DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
    DeviceRead(regMask, 1, &Registers.Mask.byte);
    DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
    DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    DeviceRead(regStatus0a, 2, &Registers.Status.byte[0]);
    DeviceRead(regStatus0, 2, &Registers.Status.byte[4]);
}

void processDMTBIST(void)
{
    UINT8 bdo = PolicyRxDataObj[0].byte[3]>>4;
    switch (bdo)
    {
        case BDO_BIST_Carrier_Mode_2:
            PolicyState = PE_BIST_Carrier_Mode_2;
            PolicySubIndex = 0;
            ProtocolState = PRLIdle;
            break;
        case BDO_BIST_Test_Data:
            PolicyState = peDisabled;
            ProtocolState = PRLIdle;
            break;
        default:
            break;
    }
}

BOOL GetPDStateLog(UINT8 * data){   
    UINT32 i;
    UINT32 entries = PDStateLog.Count;
    UINT16 state_temp;
    UINT16 time_tms_temp;
    UINT16 time_s_temp;


    for(i=0; ((i<entries) && (i<12)); i++)
    {
        ReadStateLog(&PDStateLog, &state_temp, &time_tms_temp, &time_s_temp);

        data[i*5+1] = state_temp;
        data[i*5+2] = (time_tms_temp>>8);
        data[i*5+3] = (UINT8)time_tms_temp;
        data[i*5+4] = (time_s_temp)>>8;
        data[i*5+5] = (UINT8)time_s_temp;
    }

    data[0] = i;    


    return TRUE;
}

void ProcessReadPDStateLog(UINT8* MsgBuffer, UINT8* retBuffer)
{
    if (MsgBuffer[1] != 0)
    {
        retBuffer[1] = 0x01;             
        return;
    }

    GetPDStateLog(&retBuffer[3]);   
}

void ProcessPDBufferRead(UINT8* MsgBuffer, UINT8* retBuffer)
{
    if (MsgBuffer[1] != 0)
        retBuffer[1] = 0x01;                                             
    else
    {
        retBuffer[4] = GetUSBPDBufferNumBytes();                         
        retBuffer[5] = ReadUSBPDBuffer((UINT8*)&retBuffer[6], 58); 
    }
}

VOID SetVbusTransitionTime(UINT32 time_ms) {
    VbusTransitionTime = time_ms * TICK_SCALE_TO_MS;
}
