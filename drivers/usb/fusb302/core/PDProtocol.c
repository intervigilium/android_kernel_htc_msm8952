/****************************************************************************
 * FileName:        PDProtocol.c
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

#include "PDProtocol.h"
#include "PDPolicy.h"
#include "TypeC.h"
#include "fusb30X.h"
#include "vdm/vdm_callbacks.h"
#include "vdm/vdm_callbacks_defs.h"
#include "vdm/vdm.h"
#include "vdm/vdm_types.h"
#include "vdm/bitfield_translators.h"

#include "platform.h"
#include "PD_Types.h"


extern BOOL                     g_Idle;                                         
extern volatile UINT16          Timer_S;                                        
extern volatile UINT16          Timer_tms;                                      
extern StateLog                 PDStateLog;                                     

static UINT8                    USBPDBuf[PDBUFSIZE];                            
static UINT8                    USBPDBufStart;                                  
static UINT8                    USBPDBufEnd;                                    
static BOOL                     USBPDBufOverflow;                               


       ProtocolState_t          ProtocolState;                                  
       PDTxStatus_t             PDTxStatus;                                     
static UINT8                    MessageIDCounter;                               
static UINT8                    MessageID;                                      
       BOOL                     ProtocolMsgRx;                                  
       SopType                  ProtocolMsgRxSop;                               
static UINT8                    ProtocolTxBytes;                                
static UINT8                    ProtocolTxBuffer[FSC_HOSTCOMM_BUFFER_SIZE];     
static UINT8                    ProtocolRxBuffer[FSC_HOSTCOMM_BUFFER_SIZE];     
static UINT16                   ProtocolTimer;                                  
static UINT8                    ProtocolCRC[4];
static UINT8                    BISTErrorCounter;
       BOOL                     ProtocolCheckRxBeforeTx;
UINT8                           manualRetries = 0;                              
UINT8                           nTries = 4;                                   

void ProtocolTickAt100us( void )
{
    if( !USBPDActive )
        return;

    if (ProtocolTimer)                                                          
        ProtocolTimer--;                                                        
}

void InitializePDProtocolVariables(void)
{
}





void USBPDProtocol(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
    if(g_Idle == TRUE)
    {
        g_Idle = FALSE;                                                             
        Registers.Mask.byte = 0x00;
        DeviceWrite(regMask, 1, &Registers.Mask.byte);
        Registers.MaskAdv.byte[0] = 0x00;
        DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
        Registers.MaskAdv.M_GCRCSENT = 0;
        DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
        platform_enable_timer(TRUE);
    }
#endif
    if (Registers.Status.I_HARDRST)
    {
        ResetProtocolLayer(TRUE);                                               
        if (PolicyIsSource)                                                     
            PolicyState = peSourceTransitionDefault;                            
        else                                                                    
            PolicyState = peSinkTransitionDefault;                              
        PolicySubIndex = 0;
        StoreUSBPDToken(FALSE, pdtHardReset);                                   
    }
    else
    {
        switch (ProtocolState)
        {
            case PRLReset:
                ProtocolSendHardReset();                                        
                PDTxStatus = txWait;                                            
                ProtocolState = PRLResetWait;                                   
                ProtocolTimer = tBMCTimeout;                                    
                break;
            case PRLResetWait:                                                  
                ProtocolResetWait();
                break;
            case PRLIdle:                                                       
                ProtocolIdle();
                break;
            case PRLTxSendingMessage:                                           
                ProtocolSendingMessage();                                       
                break;
            case PRLTxVerifyGoodCRC:                                            
                ProtocolVerifyGoodCRC();
                break;
            case PRL_BIST_Rx_Reset_Counter:                                     
                protocolBISTRxResetCounter();
                break;
            case PRL_BIST_Rx_Test_Frame:                                        
                protocolBISTRxTestFrame();
                break;
            case PRL_BIST_Rx_Error_Count:                                       
                protocolBISTRxErrorCount();
                break;
            case PRL_BIST_Rx_Inform_Policy:                                     
                protocolBISTRxInformPolicy();
                break;
            case PRLDisabled:                                                   
                break;
            default:
                break;
        }
    }
}

void ProtocolIdle(void)
{
    if (PDTxStatus == txReset)                                                  
        ProtocolState = PRLReset;                                               
    else if (!Registers.Status.RX_EMPTY)                                       
    {
        ProtocolGetRxPacket();                                                  
        PDTxStatus = txIdle;                                                    
        Registers.Status.I_GCRCSENT = 0;
    }
    else if (PDTxStatus == txSend)                                              
    {
        ProtocolTransmitMessage();                                              
    }
}

void ProtocolResetWait(void)
{
    if (Registers.Status.I_HARDSENT)                                            
    {
        ProtocolState = PRLIdle;                                                
        PDTxStatus = txSuccess;                                                 
    }
    else if (ProtocolTimer == 0)                                                
    {
        ProtocolState = PRLIdle;                                                
        PDTxStatus = txSuccess;                                                  
    }
}

void ProtocolGetRxPacket(void)
{
    UINT32 i, j;
    UINT8 data[3];
    SopType rx_sop;
    UINT8 sop_token;

    
    UINT16 tmsTemp = Timer_tms;

    DeviceRead(regFIFO, 3, &data[0]);                                          
    PolicyRxHeader.byte[0] = data[1];
    PolicyRxHeader.byte[1] = data[2];
    
    PolicyTxHeader.word = 0;                                                    
    PolicyTxHeader.NumDataObjects = 0;                                          
    PolicyTxHeader.MessageType = CMTGoodCRC;                                    
    PolicyTxHeader.PortDataRole = PolicyIsDFP;                                  
    PolicyTxHeader.PortPowerRole = PolicyIsSource;                              
    PolicyTxHeader.SpecRevision = USBPDSPECREV;                                 
    PolicyTxHeader.MessageID = PolicyRxHeader.MessageID;                        

    
    rx_sop = TokenToSopType(data[0]);

    
    if (rx_sop == SOP_TYPE_SOP)
    {
        
        
        if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
            MessageIDCounter = 0;                                               
            MessageID = 0xFF;                                                   
            ProtocolMsgRxSop = rx_sop;
            ProtocolMsgRx = TRUE;                                               
            SourceCapsUpdated = TRUE;                                           
        }
        else if (PolicyRxHeader.MessageID != MessageID)                         
        {
            MessageID = PolicyRxHeader.MessageID;                               
            ProtocolMsgRxSop = rx_sop;
            ProtocolMsgRx = TRUE;                                               
        }

        if (PolicyRxHeader.NumDataObjects > 0)                                      
        {
            DeviceRead(regFIFO, (PolicyRxHeader.NumDataObjects<<2), &ProtocolRxBuffer[0]); 
            for (i=0; i<PolicyRxHeader.NumDataObjects; i++)                         
            {
                for (j=0; j<4; j++)                                                 
                    PolicyRxDataObj[i].byte[j] = ProtocolRxBuffer[j + (i<<2)];      
            }
        }
    } else {
        if (PolicyRxHeader.NumDataObjects > 0)                                      
        {
            DeviceRead(regFIFO, (PolicyRxHeader.NumDataObjects<<2), &ProtocolRxBuffer[0]); 
        }
    }
    DeviceRead(regFIFO, 4, &ProtocolCRC[0]);                                   
    DeviceRead(regStatus0, 2, &Registers.Status.byte[4]);                      

    if (rx_sop == SOP_TYPE_SOP1) {
        PolicyRxHeader.word |= 0x8000;
        PolicyTxHeader.word |= 0x8000;
    }
    if (rx_sop == SOP_TYPE_SOP2) {
        PolicyRxHeader.word |= 0x0010;
        PolicyTxHeader.word |= 0x0010;
    }

    StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]);     
    if (rx_sop == SOP_TYPE_SOP) sop_token = 0xE0;
    if (rx_sop == SOP_TYPE_SOP1) sop_token = 0xC0;
    if (rx_sop == SOP_TYPE_SOP2) sop_token = 0xA0;
    if (rx_sop == SOP_TYPE_SOP) 												
	StoreUSBPDMessage(PolicyTxHeader, &PolicyTxDataObj[0], TRUE, sop_token);    

    WriteStateLog(&PDStateLog, dbgGetRxPacket, Timer_tms - tmsTemp, PolicyRxHeader.NumDataObjects * 4 + 3 + 6);
}

void ProtocolTransmitMessage(void)
{
    UINT32 i, j;
    UINT8 sop_token;
    sopMainHeader_t temp_PolicyTxHeader;
    ProtocolFlushTxFIFO();                                                      

    ProtocolLoadSOP();
    sop_token = 0xE0;

    temp_PolicyTxHeader.word = PolicyTxHeader.word;
    temp_PolicyTxHeader.word &= 0x7FFF;
    temp_PolicyTxHeader.word &= 0xFFEF;

    if ((temp_PolicyTxHeader.NumDataObjects == 0) && (temp_PolicyTxHeader.MessageType == CMTSoftReset))
    {
        MessageIDCounter = 0;                                
        MessageID = 0xFF;                                                       
        SourceCapsUpdated = TRUE;                                               
    }
    temp_PolicyTxHeader.MessageID = MessageIDCounter;        

    ProtocolTxBuffer[ProtocolTxBytes++] = PACKSYM | (2+(temp_PolicyTxHeader.NumDataObjects<<2));   
    ProtocolTxBuffer[ProtocolTxBytes++] = temp_PolicyTxHeader.byte[0];               
    ProtocolTxBuffer[ProtocolTxBytes++] = temp_PolicyTxHeader.byte[1];               
    if (temp_PolicyTxHeader.NumDataObjects > 0)                                      
    {
        for (i=0; i<temp_PolicyTxHeader.NumDataObjects; i++)                         
        {
            for (j=0; j<4; j++)                                                 
                ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxDataObj[i].byte[j];  
        }
    }
    ProtocolLoadEOP();                                                          
    if(manualRetries)
    {
        manualRetriesTakeTwo();
    }
    else
    {
        DeviceWrite(regFIFO, ProtocolTxBytes, &ProtocolTxBuffer[0]);                

        
        if (ProtocolCheckRxBeforeTx)
        {
            ProtocolCheckRxBeforeTx = FALSE; 
            DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
            if (Registers.Status.I_GCRCSENT)
            {
                
                Registers.Status.I_GCRCSENT = 0;
                ProtocolFlushTxFIFO();
                PDTxStatus = txError;
                return;
            }
        }

        Registers.Control.TX_START = 1;                                             
        DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    
        Registers.Control.TX_START = 0;                                             

        PDTxStatus = txBusy;                                                        
        ProtocolState = PRLTxSendingMessage;                                        
        ProtocolTimer = tBMCTimeout;                                                
    }
        StoreUSBPDMessage(temp_PolicyTxHeader, &PolicyTxDataObj[0], TRUE, sop_token);    

}

void ProtocolSendingMessage(void)
{
    if (Registers.Status.I_TXSENT)
    {
        ProtocolFlushTxFIFO();                                                  
        ProtocolVerifyGoodCRC();
    }
    else if (Registers.Status.I_COLLISION)                                      
    {
        
        if (manualRetries)
        {
            ProtocolFlushTxFIFO();                                                  
            PDTxStatus = txCollision;                                               
            ProtocolTimer = tBMCTimeout;                                            
            ProtocolState = PRLRxWait;                                              
        }
    }
    else if (ProtocolTimer == 0)                                                
    {
        ProtocolFlushTxFIFO();                                                  
        ProtocolFlushRxFIFO();                                                  
        PDTxStatus = txError;                                                   
        ProtocolState = PRLIdle;                                                
    }
}

void ProtocolVerifyGoodCRC(void)
{
    UINT32 i, j;
    UINT8 data[3];
    SopType s;

    DeviceRead(regFIFO, 3, &data[0]);                                          
    PolicyRxHeader.byte[0] = data[1];
    PolicyRxHeader.byte[1] = data[2];
    if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTGoodCRC))
    {
        UINT8 MIDcompare;
        switch (TokenToSopType(data[0])) {
            case SOP_TYPE_SOP:
                MIDcompare = MessageIDCounter;
                break;
            default:
                MIDcompare = -1;
                break;
        }

        if (PolicyRxHeader.MessageID != MIDcompare)                             
        {
            DeviceRead(regFIFO, 4, &ProtocolCRC[0]);                           
            StoreUSBPDToken(FALSE, pdtBadMessageID);                            
            PDTxStatus = txError;                                               
            ProtocolState = PRLIdle;                                            
        }
        else                                                                    
        {
            switch (TokenToSopType(data[0])) {
                case SOP_TYPE_SOP:
                    MessageIDCounter++;                                         
                    MessageIDCounter &= 0x07;                                   
                    break;
                default:
                    
                    break;
            }

            ProtocolState = PRLIdle;                                            
            PDTxStatus = txSuccess;                                             
            DeviceRead(regFIFO, 4, &ProtocolCRC[0]);                           
            StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]);   
        }
    }
    else
    {
        ProtocolState = PRLIdle;                                                
        PDTxStatus = txError;                                                   

        s = TokenToSopType(data[0]);
        if ((PolicyRxHeader.NumDataObjects == 0) && (PolicyRxHeader.MessageType == CMTSoftReset))
        {
            DeviceRead(regFIFO, 4, &ProtocolCRC[0]);                           

            MessageIDCounter = 0;                                 
            MessageID = 0xFF;                                                   
            ProtocolMsgRx = TRUE;                                               
            ProtocolMsgRxSop = s;
            SourceCapsUpdated = TRUE;                                           
        }
        else if (PolicyRxHeader.MessageID != MessageID)                         
        {
            DeviceRead(regFIFO, 4, &ProtocolCRC[0]);                           
            MessageID = PolicyRxHeader.MessageID;                               
            ProtocolMsgRx = TRUE;                                               
            ProtocolMsgRxSop = s;
        }
        if (PolicyRxHeader.NumDataObjects > 0)                                  
        {
           DeviceRead(regFIFO, PolicyRxHeader.NumDataObjects<<2, &ProtocolRxBuffer[0]);    
            for (i=0; i<PolicyRxHeader.NumDataObjects; i++)                     
            {
                for (j=0; j<4; j++)                                             
                    PolicyRxDataObj[i].byte[j] = ProtocolRxBuffer[j + (i<<2)];  
            }
        }
        StoreUSBPDMessage(PolicyRxHeader, &PolicyRxDataObj[0], FALSE, data[0]); 
    }

}

void ProtocolSendGoodCRC(SopType sop)
{
    if (sop == SOP_TYPE_SOP) {
        ProtocolLoadSOP();                                                      
    } else {
        return; 
    }

    ProtocolTxBuffer[ProtocolTxBytes++] = PACKSYM | 0x02;                       
    ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxHeader.byte[0];               
    ProtocolTxBuffer[ProtocolTxBytes++] = PolicyTxHeader.byte[1];               
    ProtocolLoadEOP();                                                          
    DeviceWrite(regFIFO, ProtocolTxBytes, &ProtocolTxBuffer[0]);                
    DeviceRead(regStatus0, 2, &Registers.Status.byte[4]);                      
}

void ProtocolLoadSOP(void)
{
    ProtocolTxBytes = 0;                                                        
    ProtocolTxBuffer[ProtocolTxBytes++] = SYNC1_TOKEN;                          
    ProtocolTxBuffer[ProtocolTxBytes++] = SYNC1_TOKEN;                          
    ProtocolTxBuffer[ProtocolTxBytes++] = SYNC1_TOKEN;                          
    ProtocolTxBuffer[ProtocolTxBytes++] = SYNC2_TOKEN;                          
}

void ProtocolLoadEOP(void)
{
    ProtocolTxBuffer[ProtocolTxBytes++] = JAM_CRC;                              
    ProtocolTxBuffer[ProtocolTxBytes++] = EOP;                                  
    ProtocolTxBuffer[ProtocolTxBytes++] = TXOFF;                                
}

void ProtocolSendHardReset(void)
{
    UINT8 data;
    data = Registers.Control.byte[3] | 0x40;                                    
    DeviceWrite(regControl3, 1, &data);                                         
    StoreUSBPDToken(TRUE, pdtHardReset);                                        
}

void ProtocolFlushRxFIFO(void)
{
    UINT8 data;
    data = Registers.Control.byte[1];                                           
    data |= 0x04;                                                               
    DeviceWrite(regControl1, 1, &data);                                         
}

void ProtocolFlushTxFIFO(void)
{
    UINT8 data;
    data = Registers.Control.byte[0];                                           
    data |= 0x40;                                                               
    DeviceWrite(regControl0, 1, &data);                                         
}

void ResetProtocolLayer(BOOL ResetPDLogic)
{
    UINT32 i;
    UINT8 data = 0x02;
    if (ResetPDLogic)
        DeviceWrite(regReset, 1, &data);                                       
    ProtocolFlushRxFIFO();                                                      
    ProtocolFlushTxFIFO();                                                      
    ProtocolState = PRLIdle;                                                    
    PDTxStatus = txIdle;                                                        
    ProtocolTimer = 0;                                                          
    VdmTimer = 0;
    VdmTimerStarted = FALSE;
    ProtocolTxBytes = 0;                                                        
    MessageIDCounter = 0;                                                       
    MessageID = 0xFF;                                                           
    ProtocolMsgRx = FALSE;                                                      
    ProtocolMsgRxSop = SOP_TYPE_SOP;
    USBPDTxFlag = FALSE;                                                        
    PolicyHasContract = FALSE;                                                  
    USBPDContract.object = 0;                                                   
    SourceCapsUpdated = TRUE;                                                   
    CapsHeaderReceived.word = 0;                                                
    for (i=0; i<7; i++)                                                         
        CapsReceived[i].object = 0;                                             
}


void protocolBISTRxResetCounter(void)   
{
    BISTErrorCounter = 0;   
    
    ProtocolState = PRL_BIST_Rx_Test_Frame; 
}

void protocolBISTRxTestFrame(void)      
{
    
    
}

void protocolBISTRxErrorCount(void)     
{
    
    
    
}

void protocolBISTRxInformPolicy(void)   
{
    
    
}


BOOL StoreUSBPDToken(BOOL transmitter, USBPD_BufferTokens_t token)
{
    UINT8 header1 = 1;                                                          
    if (ClaimBufferSpace(2) == FALSE)                                           
        return FALSE;                                                           
    if (transmitter)                                                            
        header1 |= 0x40;                                                        
    USBPDBuf[USBPDBufEnd++] = header1;                                          
    USBPDBufEnd %= PDBUFSIZE;                                                   
    token &= 0x0F;                                                              
    USBPDBuf[USBPDBufEnd++] = token;                                            
    USBPDBufEnd %= PDBUFSIZE;                                                   
    return TRUE;
}

BOOL StoreUSBPDMessage(sopMainHeader_t Header, doDataObject_t* DataObject, BOOL transmitter, UINT8 SOPToken)
{
    UINT32 i, j, required;
    UINT8 header1;
    required = Header.NumDataObjects * 4 + 2 + 2;                               
    if (ClaimBufferSpace(required) == FALSE)                                    
        return FALSE;                                                           
    header1 = (0x1F & (required-1)) | 0x80;
    if (transmitter)                                                            
        header1 |= 0x40;                                                        
    USBPDBuf[USBPDBufEnd++] = header1;                                          
    USBPDBufEnd %= PDBUFSIZE;                                                   
    SOPToken &= 0xE0;                                                           
    SOPToken >>= 5;                                                             
    USBPDBuf[USBPDBufEnd++] = SOPToken;                                         
    USBPDBufEnd %= PDBUFSIZE;                                                   
    USBPDBuf[USBPDBufEnd++] = Header.byte[0];                                   
    USBPDBufEnd %= PDBUFSIZE;                                                   
    USBPDBuf[USBPDBufEnd++] = Header.byte[1];                                   
    USBPDBufEnd %= PDBUFSIZE;                                                   
    for (i=0; i<Header.NumDataObjects; i++)                                     
    {
        for (j=0; j<4; j++)
        {
            USBPDBuf[USBPDBufEnd++] = DataObject[i].byte[j];                    
            USBPDBufEnd %= PDBUFSIZE;                                           
        }
    }
    return TRUE;
}

UINT8 GetNextUSBPDMessageSize(void)
{
    UINT8 numBytes;
    if (USBPDBufStart == USBPDBufEnd)                                           
        numBytes = 0;                                                           
    else                                                                        
        numBytes = (USBPDBuf[USBPDBufStart] & 0x1F) + 1;                        
    return numBytes;
}

UINT8 GetUSBPDBufferNumBytes(void)
{
    UINT8 bytes;
    if (USBPDBufStart == USBPDBufEnd)                                           
        bytes = 0;                                                              
    else if (USBPDBufEnd > USBPDBufStart)                                       
        bytes = USBPDBufEnd - USBPDBufStart;                                    
    else                                                                        
        bytes = USBPDBufEnd + (PDBUFSIZE - USBPDBufStart);                      
    return bytes;
}

BOOL ClaimBufferSpace(INT32 intReqSize)
{
    INT32 available;
    UINT8 numBytes;
    if (intReqSize >= PDBUFSIZE)                                                
        return FALSE;                                                           
    if (USBPDBufStart == USBPDBufEnd)                                           
        available = PDBUFSIZE;                                                  
    else if (USBPDBufStart > USBPDBufEnd)                                       
        available = USBPDBufStart - USBPDBufEnd;                                
    else                                                                        
        available = PDBUFSIZE - (USBPDBufEnd - USBPDBufStart);                  
    do
    {
        if (intReqSize >= available)                                            
        {
            USBPDBufOverflow = TRUE;                                            
            numBytes = GetNextUSBPDMessageSize();                               
            if (numBytes == 0)                                                  
                return FALSE;                                                   // Return FALSE since the data cannot fit in the available buffer size (nothing written)
            available += numBytes;                                              
            USBPDBufStart += numBytes;                                          
            USBPDBufStart %= PDBUFSIZE;                                         
        }
        else
            break;
    } while (1);                                                                
    return TRUE;
}


void GetUSBPDStatus(UINT8 abytData[])
{
    UINT32 i, j;
    UINT32 intIndex = 0;
    abytData[intIndex++] = GetUSBPDStatusOverview();                            
    abytData[intIndex++] = GetUSBPDBufferNumBytes();                            
    abytData[intIndex++] = PolicyState;                                         
    abytData[intIndex++] = PolicySubIndex;                                      
    abytData[intIndex++] = (ProtocolState << 4) | PDTxStatus;                   
    for (i=0;i<4;i++)
            abytData[intIndex++] = USBPDContract.byte[i];                       
    if (PolicyIsSource)
    {
        abytData[intIndex++] = CapsHeaderSource.byte[0];                        
        abytData[intIndex++] = CapsHeaderSource.byte[1];                        
        for (i=0;i<7;i++)                                                       
        {
            for (j=0;j<4;j++)                                                   
                abytData[intIndex++] = CapsSource[i].byte[j];                   
        }
    }
    else
    {
        abytData[intIndex++] = CapsHeaderReceived.byte[0];                      
        abytData[intIndex++] = CapsHeaderReceived.byte[1];                      
        for (i=0;i<7;i++)                                                       
        {
            for (j=0;j<4;j++)                                                   
                abytData[intIndex++] = CapsReceived[i].byte[j];                 
        }
    }


    
    
    
    intIndex = 44;
    abytData[intIndex++] = Registers.DeviceID.byte;     
    abytData[intIndex++] = Registers.Switches.byte[0];  
    abytData[intIndex++] = Registers.Switches.byte[1];
    abytData[intIndex++] = Registers.Measure.byte;
    abytData[intIndex++] = Registers.Slice.byte;
    abytData[intIndex++] = Registers.Control.byte[0];   
    abytData[intIndex++] = Registers.Control.byte[1];
    abytData[intIndex++] = Registers.Mask.byte;
    abytData[intIndex++] = Registers.Power.byte;
    abytData[intIndex++] = Registers.Status.byte[4];    
    abytData[intIndex++] = Registers.Status.byte[5];    
    abytData[intIndex++] = Registers.Status.byte[6];    
}

UINT8 GetUSBPDStatusOverview(void)
{
    UINT8 status = 0;
    if (USBPDEnabled)
        status |= 0x01;
    if (USBPDActive)
        status |= 0x02;
    if (PolicyIsSource)
        status |= 0x04;
    if (PolicyIsDFP)
        status |= 0x08;
    if (PolicyHasContract)
        status |= 0x10;
    if (SourceCapsUpdated)
        status |= 0x20;
    SourceCapsUpdated = FALSE;
    if (USBPDBufOverflow)
        status |= 0x80;
    return status;
}

UINT8 ReadUSBPDBuffer(UINT8* pData, UINT8 bytesAvail)
{
    UINT8 i, msgSize, bytesRead;
    bytesRead = 0;
    do
    {
        msgSize = GetNextUSBPDMessageSize();                                    
        if ((msgSize != 0) && (msgSize <= bytesAvail))                          
        {
            for (i=0; i<msgSize; i++)                                           
            {
                *pData++ = USBPDBuf[USBPDBufStart++];                           
                USBPDBufStart %= PDBUFSIZE;                                     
            }
            bytesAvail -= msgSize;                                              
            bytesRead += msgSize;                                               
        }
        else                                                                    
            break;                                                              
    } while (1);
    return bytesRead;
}



void SendUSBPDMessage(UINT8* abytData)
{
    UINT32 i, j;
    PDTransmitHeader.byte[0] = *abytData++;                                     
    PDTransmitHeader.byte[1] = *abytData++;                                     
    for (i=0; i<PDTransmitHeader.NumDataObjects; i++)                           
    {
        for (j=0; j<4; j++)                                                     
        {
            PDTransmitObjects[i].byte[j] = *abytData++;                         
        }
    }
    USBPDTxFlag = TRUE;                                                         
}

void manualRetriesTakeTwo(void)
{
    UINT8 tries = nTries;
    
    Registers.Mask.byte = ~0x02;
    DeviceWrite(regMask, 1, &Registers.Mask.byte);
    Registers.MaskAdv.byte[0] = ~0x14;
    DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
    Registers.MaskAdv.M_GCRCSENT = 1;
    DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);

    
    DeviceRead(regInterrupt, 1, &Registers.Status.byte[6]);
    DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]);
    DeviceRead(regInterruptb, 1, &Registers.Status.byte[3]);
    Registers.Status.I_TXSENT = 0;                                              
    Registers.Status.I_RETRYFAIL = 0;                                           
    Registers.Status.I_COLLISION = 0;                                           

    
    DeviceWrite(regFIFO, ProtocolTxBytes, &ProtocolTxBuffer[0]);            

    while(tries)
    {
        
        Registers.Control.TX_START = 1;                                         
        DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                
        Registers.Control.TX_START = 0;                                         

        
        while(!platform_get_device_irq_state());               
                    DeviceRead(regInterrupt, 1, &Registers.Status.byte[6]);             

        DeviceRead(regInterrupta, 1, &Registers.Status.byte[2]);

        if(Registers.Status.I_TXSENT)
        {
            
            Registers.Status.I_TXSENT = 0;                                      
            ProtocolVerifyGoodCRC();
            ProtocolState = PRLIdle;                                            
            PDTxStatus = txSuccess;                                             
            tries = 0;
        }
        else if(Registers.Status.I_RETRYFAIL)
        {
            Registers.Status.I_RETRYFAIL = 0;                                   
            tries--;                                                            
            if(!tries)                                                          
            {
                
                ProtocolState = PRLIdle;                                        
                PDTxStatus = txError;                                           
            }
            else
            {
                
                DeviceWrite(regFIFO, ProtocolTxBytes, &ProtocolTxBuffer[0]);            
            }
        }
        else if(Registers.Status.I_COLLISION)    
        {
            Registers.Status.I_COLLISION = 0;                                   
            DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);               
        }
    }

    
    Registers.Mask.byte = 0x00;
    DeviceWrite(regMask, 1, &Registers.Mask.byte);
    Registers.MaskAdv.byte[0] = 0x00;
    DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
    Registers.MaskAdv.M_GCRCSENT = 0;
    DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
}

void setManualRetries(UINT8 mode)
{
    manualRetries = mode;
}

UINT8 getManualRetries(void)
{
    return manualRetries;
}
