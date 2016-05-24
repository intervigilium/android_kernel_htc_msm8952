#include "AlternateModes.h"

BOOL startDRPSource = FALSE;
extern BOOL g_Idle;


void SetStateAlternateUnattached(void)
{
    if ((PortType == USBTypeC_DRP) || ((PortType == USBTypeC_Sink) && (blnAccSupport)))   
    {
        SetStateAlternateDRP();
    }
    else if (PortType == USBTypeC_Source)                                       
    {
        SetStateAlternateUnattachedSource();
    }
    else                                                                        
    {
        SetStateAlternateUnattachedSink();
    }
}

void StateMachineAlternateUnattached(void)
{
    if ((PortType == USBTypeC_DRP) || ((PortType == USBTypeC_Sink) && (blnAccSupport)))   
    {
        StateMachineAlternateDRP();
    }
    else if (PortType == USBTypeC_Source)                                       
    {
        StateMachineAlternateUnattachedSource();
    }
    else                                                                        
    {
        StateMachineAlternateUnattachedSink();
    }
}

void SetStateAlternateDRP(void)
{
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
    platform_set_vbus_5v_enable(FALSE);                                         
    platform_set_vbus_lvl1_enable(FALSE);                                       
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
    if(ConnState == AttachedSource)                                             
    {
        while(!VbusVSafe0V());                                                  
    }
    ConnState = Unattached;                                                     

    if((blnCCPinIsCC1 && startDRPSource) || (blnCCPinIsCC2 && (startDRPSource == 0)))
    {
        sourceOrSink = Source;                                                  
        Registers.Switches.byte[0] = 0x46;                                      
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);              
    }
    else
    {
        sourceOrSink = Sink;                                                    
        Registers.Switches.byte[0] = 0x89;                                      
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);              
    }
    startDRPSource = 0;
    Registers.Power.PWR = 0x7;                                                  
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            
    updateSourceCurrent();                                                      
    USBPDDisable(TRUE);                                                         
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                                      
    blnCCPinIsCC2 = FALSE;                                                      
    StateTimer = tAlternateDRPSwap;
    PDDebounce = T_TIMER_DISABLE;                                              
    CCDebounce = T_TIMER_DISABLE;                                              
    ToggleTimer = tDeviceToggle;                                                
    OverPDDebounce = tPDDebounce;                                           
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void StateMachineAlternateDRP(void)
{
    debounceCC();

    if(StateTimer == 0)
    {
        AlternateDRPSwap();
        StateTimer = tAlternateDRPSwap;
    }

    if((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Source))
    {
        blnCCPinIsCC1 = TRUE;
        blnCCPinIsCC2 = FALSE;
        if ((PortType == USBTypeC_Sink) && (blnAccSupport))             
            checkForAccessory();                                        
        else                                                            
            SetStateAttachWaitSource();
    }
    
    else if((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Sink))
    {
        blnCCPinIsCC1 = FALSE;
        blnCCPinIsCC2 = TRUE;
        if ((PortType == USBTypeC_Sink) && (blnAccSupport))             
            checkForAccessory();                                        
        else                                                            
            SetStateAttachWaitSource();
    }
    else if((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Sink))
    {
        blnCCPinIsCC1 = TRUE;
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSink();
    }
    
    else if((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Source))
    {
        blnCCPinIsCC1 = FALSE;
        blnCCPinIsCC2 = TRUE;
        SetStateAttachWaitSink();
    }

}

void AlternateDRPSwap(void)
{
    if(sourceOrSink == Source)
    {
        Registers.Switches.byte[0] = 0x89;                              
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      
        sourceOrSink = Sink;
    }
    else
    {
        Registers.Switches.byte[0] = 0x46;                              
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      
        sourceOrSink = Source;
    }
}

void AlternateDRPSourceSinkSwap(void)
{
    if(ConnState == Unattached)
    {
        if(Registers.Switches.MEAS_CC2 == 1)                
        {
            if(sourceOrSink == Source)
            {
                sourceOrSink = Sink;
            }
            else
            {
                sourceOrSink = Source;
            }
        }
    }
}

void SetStateAlternateUnattachedSource(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
    g_Idle = TRUE;                                                             
    Registers.Mask.byte = 0xFF;
    Registers.Mask.M_COMP_CHNG = 0;
    Registers.Mask.M_BC_LVL = 0;
    DeviceWrite(regMask, 1, &Registers.Mask.byte);
    Registers.MaskAdv.byte[0] = 0xFF;
    DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
    Registers.MaskAdv.M_GCRCSENT = 1;
    DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
    platform_enable_timer(FALSE);
#endif
    if(PortType == USBTypeC_DRP)
    {
        startDRPSource = 1;
        SetStateAlternateDRP();
        return;
    }
    platform_set_vbus_5v_enable(FALSE);                                         
    platform_set_vbus_lvl1_enable(FALSE);                                       
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
    while(!VbusVSafe0V());                                                      
    ConnState = UnattachedSource;                                               
    sourceOrSink = Source;
    Registers.Switches.byte[0] = 0xC4;                                          
    Registers.Power.PWR = 0x7;                                                  
    Registers.Control.HOST_CUR = 0b11;                                          
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            
    DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  
    USBPDDisable(TRUE);                                                         
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                                      
    blnCCPinIsCC2 = FALSE;                                                      
    StateTimer = T_TIMER_DISABLE;                                               
    PDDebounce = tPDDebounce;                                               
    CCDebounce = tCCDebounce;                                               
    ToggleTimer = T_TIMER_DISABLE;                                              
    DRPToggleTimer = T_TIMER_DISABLE;                                           
    OverPDDebounce = T_TIMER_DISABLE;                                          
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void StateMachineAlternateUnattachedSource(void) 
{
    CCTermType previous = AlternateDecodeCCTerminationSource();

    if(blnAccSupport)
    {
        if (previous == CCTypeRa)
        {
            blnCCPinIsCC1 = FALSE;                                                  
            blnCCPinIsCC2 = FALSE;
            SetStateAttachWaitSource();
            return;
        }
    }

    if ((previous == CCTypeRd3p0) || (previous == CCTypeRd1p5))                 
    {
        blnCCPinIsCC1 = FALSE;                                                  
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSource();                                             
    }
    else if(previous == CCTypeRdUSB)                                            
    {
        peekCC1Source();                                                        
        if((CC1TermPrevious == CCTypeRdUSB) || (CC1TermPrevious == CCTypeRd1p5) || (CC1TermPrevious == CCTypeRd3p0))
        {
            blnCCPinIsCC1 = FALSE;
            blnCCPinIsCC2 = FALSE;
            SetStateAttachWaitSource();
        }
        else
        {
            peekCC2Source();                                                    
            if((CC2TermPrevious == CCTypeRdUSB) || (CC2TermPrevious == CCTypeRd1p5) || (CC2TermPrevious == CCTypeRd3p0))
            {
                blnCCPinIsCC1 = FALSE;
                blnCCPinIsCC2 = FALSE;
                SetStateAttachWaitSource();
            }
        }
    }
}

void StateMachineAlternateUnattachedSink(void)
{
    debounceCC();

    if ((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious < CCTypeUndefined) && ((CC2TermPrevious == CCTypeRa) || CC2TermPrevious == CCTypeOpen))    
    {
        blnCCPinIsCC1 = TRUE;                                                   
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSink();                                                  
    }
    else if ((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious < CCTypeUndefined) && ((CC1TermPrevious == CCTypeRa) || CC1TermPrevious == CCTypeOpen))   
    {
        blnCCPinIsCC1 = FALSE;                                                  
        blnCCPinIsCC2 = TRUE;
        SetStateAttachWaitSink();                                                  
    }
}

void SetStateAlternateUnattachedSink(void)
{
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
    platform_set_vbus_5v_enable( FALSE );                                       
    platform_set_vbus_lvl1_enable( FALSE );                                     
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
    while(!VbusVSafe0V());  

    ConnState = Unattached;                                                     
    sourceOrSink = Sink;
    Registers.Switches.byte[0] = 0x07;                                          
    Registers.Power.PWR = 0x7;                                                  
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  
    USBPDDisable(TRUE);                                                         
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                                      
    blnCCPinIsCC2 = FALSE;                                                      
    StateTimer = T_TIMER_DISABLE;                                               
    PDDebounce = tPDDebounce;                                               
    CCDebounce = tCCDebounce;                                               
    ToggleTimer = tDeviceToggle;                                                
    DRPToggleTimer = tDeviceToggle;                                             
    OverPDDebounce = T_TIMER_DISABLE;                                          
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAlternateAudioAccessory(void)
{
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
    platform_set_vbus_5v_enable( FALSE );                                       
    platform_set_vbus_lvl1_enable( FALSE );                                      
    ConnState = AudioAccessory;                                                 
    sourceOrSink = Source;
    Registers.Power.PWR = 0x7;                                                  
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            
    Registers.Control.HOST_CUR = 0b11;                                          
    DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    
    Registers.Switches.byte[0] = 0xC4;                                          
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  
    SinkCurrent = utccNone;                                                     
    OverPDDebounce = T_TIMER_DISABLE;                                       
    StateTimer = T_TIMER_DISABLE;                                            
    PDDebounce = tCCDebounce;                                               
    CCDebounce = T_TIMER_DISABLE;                                           
    ToggleTimer = T_TIMER_DISABLE;                                           
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

CCTermType AlternateDecodeCCTerminationSource(void)
{
    CCTermType Termination = CCTypeUndefined;            

    Registers.Measure.MDAC = MDAC_2P05V;                                        
    DeviceWrite(regMeasure, 1, &Registers.Measure.byte);                        
    platform_delay_10us(25);                                                    
    DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);

    if (Registers.Status.COMP == 1)
    {
        Termination = CCTypeOpen;
        return Termination;
    }
    else if(Registers.Status.BC_LVL == 0)
    {
        Termination = CCTypeRa;
    }
    else if(Registers.Status.BC_LVL == 3)
    {
        Termination = CCTypeRd3p0;
    }
    else if(Registers.Status.BC_LVL == 2)
    {
        Termination = CCTypeRd1p5;
    }
    else
    {
        Termination = CCTypeRdUSB;
    }
    return Termination;                             
}

