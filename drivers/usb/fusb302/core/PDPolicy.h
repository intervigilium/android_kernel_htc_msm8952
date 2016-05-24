/*********************************************************************
* FileName:        PDPolicy.h
* Dependencies:    See INCLUDES section below
* Processor:       PIC32
* Compiler:        XC32
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
********************************************************************/

#ifndef _PDPOLICY_H_
#define	_PDPOLICY_H_

#include "platform.h"
#include "PD_Types.h"
#include "vdm/vdm.h"

extern BOOL                     USBPDTxFlag;                                    
extern sopMainHeader_t          PDTransmitHeader;                               
extern sopMainHeader_t          CapsHeaderSink;                                 
extern sopMainHeader_t          CapsHeaderSource;                               
extern sopMainHeader_t          CapsHeaderReceived;                             
extern doDataObject_t           PDTransmitObjects[7];                           
extern doDataObject_t           CapsSink[7];                                    
extern doDataObject_t           CapsSource[7];                                  
extern doDataObject_t           CapsReceived[7];                                
extern doDataObject_t           USBPDContract;                                  
extern doDataObject_t           SinkRequest;                                    
extern UINT32                   SinkRequestMaxVoltage;                          
extern UINT32                   SinkRequestMaxPower;                            
extern UINT32                   SinkRequestOpPower;                             
extern BOOL                     SinkGotoMinCompatible;                          
extern BOOL                     SinkUSBSuspendOperation;                        
extern BOOL                     SinkUSBCommCapable;                             
extern BOOL                     SourceCapsUpdated;                              

extern PolicyState_t            PolicyState;                                    
extern UINT8                    PolicySubIndex;                                 
extern BOOL                     PolicyIsSource;                                 
extern BOOL                     PolicyIsDFP;                                    
extern BOOL                     PolicyHasContract;                              
extern UINT32                   VbusTransitionTime;                             

extern sopMainHeader_t          PolicyRxHeader;                                 
extern sopMainHeader_t          PolicyTxHeader;                                 
extern doDataObject_t           PolicyRxDataObj[7];                             
extern doDataObject_t           PolicyTxDataObj[7];                             

extern UINT32                   NoResponseTimer;                                

extern UINT32                   VdmTimer;
extern BOOL                     VdmTimerStarted;


VOID InitializePDPolicyVariables(VOID);
VOID USBPDEnable(BOOL DeviceUpdate, BOOL TypeCDFP);
VOID USBPDDisable(BOOL DeviceUpdate);
VOID EnableUSBPD(VOID);
VOID DisableUSBPD(VOID);
VOID USBPDPolicyEngine(VOID);
VOID PolicyErrorRecovery(VOID);
VOID PolicySourceSendHardReset(VOID);
VOID PolicySourceSoftReset(VOID);
VOID PolicySourceSendSoftReset(VOID);
VOID PolicySourceStartup(VOID);
VOID PolicySourceDiscovery(VOID);
VOID PolicySourceSendCaps(VOID);
VOID PolicySourceDisabled(VOID);
VOID PolicySourceTransitionDefault(VOID);
VOID PolicySourceNegotiateCap(VOID);
VOID PolicySourceTransitionSupply(VOID);
VOID PolicySourceCapabilityResponse(VOID);
VOID PolicySourceReady(VOID);
VOID PolicySourceGiveSourceCap(VOID);
VOID PolicySourceGetSourceCap(VOID);
VOID PolicySourceGetSinkCap(VOID);
VOID PolicySourceGetSinkCap(VOID);
VOID PolicySourceSendPing(VOID);
VOID PolicySourceGotoMin(VOID);
VOID PolicySourceGiveSinkCap(VOID);
VOID PolicySourceSendDRSwap(VOID);
VOID PolicySourceEvaluateDRSwap(VOID);
VOID PolicySourceSendVCONNSwap(VOID);
VOID PolicySourceSendPRSwap(VOID);
VOID PolicySourceEvaluatePRSwap(VOID);
void PolicySourceWaitNewCapabilities(void);
VOID PolicySinkSendHardReset(VOID);
VOID PolicySinkSoftReset(VOID);
VOID PolicySinkSendSoftReset(VOID);
VOID PolicySinkTransitionDefault(VOID);
VOID PolicySinkStartup(VOID);
VOID PolicySinkDiscovery(VOID);
VOID PolicySinkWaitCaps(VOID);
VOID PolicySinkEvaluateCaps(VOID);
VOID PolicySinkSelectCapability(VOID);
VOID PolicySinkTransitionSink(VOID);
VOID PolicySinkReady(VOID);
VOID PolicySinkGiveSinkCap(VOID);
VOID PolicySinkGetSinkCap(VOID);
VOID PolicySinkGiveSourceCap(VOID);
VOID PolicySinkGetSourceCap(VOID);
VOID PolicySinkSendDRSwap(VOID);
VOID PolicySinkEvaluateDRSwap(VOID);
VOID PolicySinkEvaluateVCONNSwap(VOID);
VOID PolicySinkSendPRSwap(VOID);
VOID PolicySinkEvaluatePRSwap(VOID);
VOID PolicyGiveVdm(VOID);
VOID PolicyVdm(VOID);
VOID PolicyInvalidState(VOID);
VOID policyBISTReceiveMode(VOID);
VOID policyBISTFrameReceived(VOID);
VOID policyBISTCarrierMode2(VOID);
BOOL PolicySendHardReset(PolicyState_t nextState, UINT32 delay);
UINT8 PolicySendCommand(UINT8 Command, PolicyState_t nextState, UINT8 subIndex);
UINT8 PolicySendCommandNoReset(UINT8 Command, PolicyState_t nextState, UINT8 subIndex);
UINT8 PolicySendData(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex, SopType sop);
UINT8 PolicySendDataNoReset(UINT8 MessageType, UINT8 NumDataObjects, doDataObject_t* DataObjects, PolicyState_t nextState, UINT8 subIndex);
VOID UpdateCapabilitiesRx(BOOL IsSourceCaps);

VOID WriteSourceCapabilities(UINT8* abytData);
VOID ReadSourceCapabilities(UINT8* abytData);
VOID WriteSinkCapabilities(UINT8* abytData);
VOID ReadSinkCapabilities(UINT8* abytData);
VOID WriteSinkRequestSettings(UINT8* abytData);
VOID ReadSinkRequestSettings(UINT8* abytData);
VOID processDMTBIST(VOID);

VOID InitializeVdmManager(VOID);
VOID convertAndProcessVdmMessage(SopType sop);
BOOL sendVdmMessage(SopType sop, UINT32 * arr, UINT32 length, PolicyState_t next_ps);
VOID doVdmCommand(VOID);
VOID doDiscoverIdentity(VOID);
VOID doDiscoverSvids(VOID);

SopType TokenToSopType(UINT8 data);
VOID autoVdmDiscovery(VOID);

BOOL GetPDStateLog(UINT8 * data);
VOID PolicyTickAt100us(VOID);

VOID ProcessReadPDStateLog(UINT8* MsgBuffer, UINT8* retBuffer);
VOID ProcessPDBufferRead(UINT8* MsgBuffer, UINT8* retBuffer);

VOID SetVbusTransitionTime(UINT32 time_ms);

#endif	
