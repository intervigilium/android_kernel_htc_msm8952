/****************************************************************************
 * FileName:		TypeC.c
 * Processor:		PIC32MX250F128B
 * Compiler:		MPLAB XC32
 * Company:			Fairchild Semiconductor
 *
 * Author			Date		  Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * M. Smith			12/04/2014	  Initial Version
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the ?Company?)
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN ?AS IS? CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include "TypeC.h"
#include "Log.h"
#include "fusb30X.h"
#include "AlternateModes.h"

#include "PDPolicy.h"
#include "vdm/vdm_config.h"
#include <linux/switch.h>


#define MAX_DEB_FAKE_IRQ_COUNT	2


DeviceReg_t				Registers;			
BOOL					USBPDActive;		
BOOL					USBPDEnabled;		
UINT32					PRSwapTimer;		
SourceOrSink			sourceOrSink;		
BOOL					g_Idle;				

USBTypeCPort			PortType;			
BOOL					blnCCPinIsCC1;		
BOOL					blnCCPinIsCC2;		
BOOL					blnSMEnabled;		
ConnectionState			ConnState;			
StateLog				TypeCStateLog;		
volatile UINT16			Timer_S;			
volatile UINT16			Timer_tms;			

static BOOL				blnCurrentUpdate;	   
static BOOL				blnMDACHigh;		 
static BOOL				blnSrcPreferred;	
static BOOL				blnSnkPreferred;	
BOOL			 blnAccSupport;		 
static BOOL				blnINTActive;		
UINT16			 StateTimer;		 
UINT16			 PDDebounce;	 
UINT16			 CCDebounce;	 
UINT16			 ToggleTimer;		 
UINT16			 DRPToggleTimer;	 
UINT16			 OverPDDebounce;	
CCTermType		 CC1TermPrevious;		  
CCTermType		 CC2TermPrevious;		  
CCTermType		 CC1TermCCDebounce;			
CCTermType		 CC2TermCCDebounce;			
CCTermType		 CC1TermPDDebounce;
CCTermType		 CC2TermPDDebounce;
CCTermType		 CC1TermPDDebouncePrevious;
CCTermType		 CC2TermPDDebouncePrevious;

USBTypeCCurrent  SinkCurrent;		 
static USBTypeCCurrent	SourceCurrent;		

static UINT8 alternateModes = 0;			  
int host_mode_status = 0;
extern void msm_otg_set_id_state(int);
extern int fusb_debug_level;
static int fake_irq_counter = 0;
extern int fusb_ReStartIrq(void);


void TypeCTickAt100us(void)
{
	if ((StateTimer<T_TIMER_DISABLE) && (StateTimer>0))
		StateTimer--;
	if ((PDDebounce<T_TIMER_DISABLE) && (PDDebounce>0))
		PDDebounce--;
	if ((CCDebounce<T_TIMER_DISABLE) && (CCDebounce>0))
		CCDebounce--;
	if ((ToggleTimer<T_TIMER_DISABLE) && (ToggleTimer>0))
		ToggleTimer--;
	if (PRSwapTimer)
		PRSwapTimer--;
	if ((OverPDDebounce<T_TIMER_DISABLE) && (OverPDDebounce>0))
		OverPDDebounce--;
	if ((DRPToggleTimer<T_TIMER_DISABLE) && (DRPToggleTimer>0))
		DRPToggleTimer--;
}


void LogTickAt100us(void)
{
	Timer_tms++;
	if(Timer_tms==10000)
	{
		Timer_S++;
		Timer_tms = 0;
	}
}

void InitializeRegisters(void)
{
	DeviceRead(regDeviceID, 1, &Registers.DeviceID.byte);
	DeviceRead(regSwitches0, 1, &Registers.Switches.byte[0]);
	DeviceRead(regSwitches1, 1, &Registers.Switches.byte[1]);
	DeviceRead(regMeasure, 1, &Registers.Measure.byte);
	DeviceRead(regSlice, 1, &Registers.Slice.byte);
	DeviceRead(regControl0, 1, &Registers.Control.byte[0]);
	DeviceRead(regControl1, 1, &Registers.Control.byte[1]);
	DeviceRead(regControl2, 1, &Registers.Control.byte[2]);
	DeviceRead(regControl3, 1, &Registers.Control.byte[3]);
	DeviceRead(regMask, 1, &Registers.Mask.byte);
	DeviceRead(regPower, 1, &Registers.Power.byte);
	DeviceRead(regReset, 1, &Registers.Reset.byte);
	DeviceRead(regOCPreg, 1, &Registers.OCPreg.byte);
	DeviceRead(regMaska, 1, &Registers.MaskAdv.byte[0]);
	DeviceRead(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	DeviceRead(regControl4, 1, &Registers.Control4.byte);
	DeviceRead(regStatus0a, 1, &Registers.Status.byte[0]);
	DeviceRead(regStatus1a, 1, &Registers.Status.byte[1]);
	
	
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
	DeviceRead(regStatus1, 1, &Registers.Status.byte[5]);
	
}

void InitializeTypeCVariables(void)
{
	InitializeRegisters();				

	Registers.Control.INT_MASK = 0;		   
	DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);

	Registers.Control.TOG_RD_ONLY = 1;
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);

	blnSMEnabled = FALSE;				 
	blnAccSupport = FALSE;				
	blnSrcPreferred = FALSE;			
	blnSnkPreferred = FALSE;
	g_Idle = FALSE;

	
	switch (Registers.Control.MODE)
	{
		case 0b10:
			PortType = USBTypeC_Sink;
			break;
		case 0b11:
			PortType = USBTypeC_Source;
			break;
		case 0b01:
			PortType = USBTypeC_DRP;
			break;
		default:
			PortType = USBTypeC_DRP;
			break;
	}

	
	
	ConnState = Disabled;				

	blnINTActive = FALSE;				
	blnCCPinIsCC1 = FALSE;				
	blnCCPinIsCC2 = FALSE;				
	StateTimer = T_TIMER_DISABLE;			  
	PDDebounce = T_TIMER_DISABLE;		  
	CCDebounce = T_TIMER_DISABLE;		  
	ToggleTimer = T_TIMER_DISABLE;			  
	resetDebounceVariables();			
	SinkCurrent = utccNone;				
	SourceCurrent = utccDefault;		
	updateSourceCurrent();
	USBPDActive = FALSE;				
	USBPDEnabled = TRUE;				
	PRSwapTimer = 0;					
	IsHardReset = FALSE;				
	blnCurrentUpdate = TRUE;			
	blnMDACHigh = FALSE;				
}

void InitializeTypeC(void)
{
	Timer_tms = 0;
	Timer_S = 0;

	InitializeStateLog(&TypeCStateLog); 

	SetStateDelayUnattached();
}

void DisableTypeCStateMachine(void)
{
	blnSMEnabled = FALSE;

	
}

void EnableTypeCStateMachine(void)
{
	blnSMEnabled = TRUE;

	Timer_tms = 0;
	Timer_S = 0;
}

void StateMachineTypeC(void)
{
#ifdef	FSC_INTERRUPT_TRIGGERED
	int ret = 0;
	do{
#endif
		if (!blnSMEnabled)
			return;

		if (platform_get_device_irq_state())
		{
			DeviceRead(regStatus0a, 7, &Registers.Status.byte[0]);	   
			Registers.Status2 = Registers.Status; 
			if (fusb_debug_level >= 1)
				printk("FUSB  %s: register status: 0x%x:%d, 0x%x:%d, 0x%x:%d, 0x%x:%d, 0x%x:%d, 0x%x:%d, 0x%x:%d\n", __func__,
					regStatus0a, Registers.Status.byte[0],
					regStatus1a, Registers.Status.byte[1],
					regInterrupta, Registers.Status.byte[2],
					regInterruptb, Registers.Status.byte[3],
					regStatus0, Registers.Status.byte[4],
					regStatus1, Registers.Status.byte[5],
					regInterrupt, Registers.Status.byte[6]);
		}

		if (fusb_debug_level >= 2)
			printk("FUSB  [%s]: Enter StateMachineTypeC(), now ConnState:%d, g_Idle:%d, USBPDActive:%d\n", __func__, ConnState, g_Idle, USBPDActive);

		if (USBPDActive)												
		{
			USBPDProtocol();											
			USBPDPolicyEngine();										
		}


		switch (ConnState)
		{
			case Disabled:
				StateMachineDisabled();
				break;
			case ErrorRecovery:
				StateMachineErrorRecovery();
				break;
			case Unattached:
				StateMachineUnattached();
				break;
			case AttachWaitSink:
				StateMachineAttachWaitSink();
				break;
			case AttachedSink:
				StateMachineAttachedSink();
				break;
			case AttachWaitSource:
				StateMachineAttachWaitSource();
				break;
			case AttachedSource:
				StateMachineAttachedSource();
				break;
			case TrySource:
				StateMachineTrySource();
				break;
			case TryWaitSink:
				StateMachineTryWaitSink();
				break;
			case TrySink:
				stateMachineTrySink();
				break;
			case TryWaitSource:
				stateMachineTryWaitSource();
				break;
			case AudioAccessory:
				StateMachineAudioAccessory();
				break;
			case DebugAccessory:
				StateMachineDebugAccessory();
				break;
			case AttachWaitAccessory:
				StateMachineAttachWaitAccessory();
				break;
			case PoweredAccessory:
				StateMachinePoweredAccessory();
				break;
			case UnsupportedAccessory:
				StateMachineUnsupportedAccessory();
				break;
			case DelayUnattached:
				StateMachineDelayUnattached();
				break;
			case UnattachedSource:
				stateMachineUnattachedSource();
				break;
			default:
				SetStateDelayUnattached();											
				break;
		}
		Registers.Status.Interrupt1 = 0;			
		Registers.Status.InterruptAdv = 0;			

#ifdef	FSC_INTERRUPT_TRIGGERED
		platform_delay_10us(SLEEP_DELAY);
		
	}while(g_Idle == FALSE);
#endif
	if (fusb_debug_level >= 0)
		printk("FUSB  [%s]: leave StateMachineTypeC(), now ConnState:%d, CC1TermPrevious:%d, CC2TermPrevious:%d, CC1TermCCDebounce:%d, CC2TermCCDebounce:%d\n",
			__func__, ConnState, CC1TermPrevious, CC2TermPrevious, CC1TermCCDebounce, CC2TermCCDebounce);

	if (fake_irq_counter > MAX_DEB_FAKE_IRQ_COUNT) {
#ifdef FSC_INTERRUPT_TRIGGERED
		fake_irq_counter = 0;
		ret = fusb_ReStartIrq();
		if (ret) {
			printk("FUSB  [%s] - Error: Unable to disable interrupts! Error code: %d\n", __func__, ret);
		}
#endif
	}

}

void StateMachineDisabled(void)
{
	
}

void StateMachineErrorRecovery(void)
{
	if (StateTimer == 0)
	{
		SetStateDelayUnattached();
	}
}

void StateMachineDelayUnattached(void)
{
	if (StateTimer == 0)
	{
		SetStateUnattached();
	}
}

void StateMachineUnattached(void)	
{
	if(alternateModes)
	{
		StateMachineAlternateUnattached();
		return;
	}

	if (Registers.Control.HOST_CUR != 0b01) 
	{
		Registers.Control.HOST_CUR = 0b01;
		DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);
	}

	if (Registers.Status.I_TOGDONE)
	{
		switch (Registers.Status.TOGSS)
		{
			case 0b101: 
				blnCCPinIsCC1 = TRUE;
				blnCCPinIsCC2 = FALSE;
				if (fusb_debug_level >= 0)
					printk("FUSB  [%s] - Registers.Status.TOGSS: 0x%x(%d), Rp detected on CC1\n",
						__func__, Registers.Status.TOGSS, Registers.Status.TOGSS);
				SetStateAttachWaitSink();										 
				break;
			case 0b110: 
				blnCCPinIsCC1 = FALSE;
				blnCCPinIsCC2 = TRUE;
				if (fusb_debug_level >= 0)
					printk("FUSB  [%s] - Registers.Status.TOGSS: 0x%x(%d), Rp detected on CC2\n",
						__func__, Registers.Status.TOGSS, Registers.Status.TOGSS);
				SetStateAttachWaitSink();										 
				break;
			case 0b001: 
				blnCCPinIsCC1 = TRUE;
				blnCCPinIsCC2 = FALSE;
				if (fusb_debug_level >= 0)
					printk("FUSB  [%s] - Registers.Status.TOGSS: 0x%x(%d), Rd detected on CC1\n",
						__func__, Registers.Status.TOGSS, Registers.Status.TOGSS);
				if ((PortType == USBTypeC_Sink) && (blnAccSupport))				
					checkForAccessory();										
				else															
					SetStateAttachWaitSource();									   
				break;
			case 0b010: 
				blnCCPinIsCC1 = FALSE;
				blnCCPinIsCC2 = TRUE;
				if (fusb_debug_level >= 0)
					printk("FUSB  [%s] - Registers.Status.TOGSS: 0x%x(%d), Rd detected on CC2\n",
						__func__, Registers.Status.TOGSS, Registers.Status.TOGSS);
				if ((PortType == USBTypeC_Sink) && (blnAccSupport))				
					checkForAccessory();									   
				else															
					SetStateAttachWaitSource();									   
				break;
			case 0b111: 
				blnCCPinIsCC1 = FALSE;
				blnCCPinIsCC2 = FALSE;
				if (fusb_debug_level >= 0)
					printk("FUSB  [%s] - Registers.Status.TOGSS: 0x%x(%d), Ra detected on both CC1 and CC2\n",
						__func__, Registers.Status.TOGSS, Registers.Status.TOGSS);
				if ((PortType == USBTypeC_Sink) && (blnAccSupport))				
					SetStateAttachWaitAccessory();									  
				else															
					SetStateAttachWaitSource();									   
				break;
			default:	
				Registers.Control.TOGGLE = 0;									
				DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);	   
				platform_delay_10us(1);
				Registers.Control.TOGGLE = 1;									
				DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);	   
				break;
		}
	}

#ifdef FPGA_BOARD
	rand();
#endif 
}

void StateMachineAttachWaitSink(void)
{
	debounceCC();

	if ((CC1TermPDDebounce == CCTypeOpen) && (CC2TermPDDebounce == CCTypeOpen)) 
	{
		if (PortType == USBTypeC_DRP)
		{
			
			fake_irq_counter++;
			SetStateUnattachedSource();											
		}
		else
		{
			SetStateDelayUnattached();
		}
	}
	else if (Registers.Status.VBUSOK)											
	{
		fake_irq_counter = 0;
		if ((CC1TermCCDebounce > CCTypeOpen) && (CC2TermCCDebounce == CCTypeOpen)) 
		{
			blnCCPinIsCC1 = TRUE;												
			blnCCPinIsCC2 = FALSE;
			if ((PortType == USBTypeC_DRP) && blnSrcPreferred)					
				SetStateTrySource();											
			else																
			{
				SetStateAttachedSink();											
			}
		}
		else if ((CC1TermCCDebounce == CCTypeOpen) && (CC2TermCCDebounce > CCTypeOpen)) 
		{
			blnCCPinIsCC1 = FALSE;												
			blnCCPinIsCC2 = TRUE;
			if ((PortType == USBTypeC_DRP) && blnSrcPreferred)					
				SetStateTrySource();											
			else																
			{
				SetStateAttachedSink();											
			}
		}
	}
}

void StateMachineAttachWaitSource(void)
{
	debounceCC();

	if(blnCCPinIsCC1)															
	{
		if(CC1TermCCDebounce != CCTypeUndefined)
		{
			peekCC2Source();
		}
	}
	else if (blnCCPinIsCC2)
	{
		if(CC2TermCCDebounce != CCTypeUndefined)
		{
			peekCC1Source();
		}
	}

	if(blnAccSupport)															
	{
		if ((CC1TermCCDebounce == CCTypeRa) && (CC2TermCCDebounce == CCTypeRa))				  
			SetStateAudioAccessory();
		else if ((CC1TermCCDebounce >= CCTypeRdUSB) && (CC1TermCCDebounce < CCTypeUndefined) && (CC2TermCCDebounce >= CCTypeRdUSB) && (CC2TermCCDebounce < CCTypeUndefined))		   
			SetStateDebugAccessory();
	}
	if (((CC1TermCCDebounce >= CCTypeRdUSB) && (CC1TermCCDebounce < CCTypeUndefined)) && ((CC2TermCCDebounce == CCTypeOpen) || (CC2TermCCDebounce == CCTypeRa)))	  
	{
		if (VbusVSafe0V()) {												
			if(blnSnkPreferred)
			{
				SetStateTrySink();
			}
			else
			{
				SetStateAttachedSource();										   
			}										   
		}
	}
	else if (((CC2TermCCDebounce >= CCTypeRdUSB) && (CC2TermCCDebounce < CCTypeUndefined)) && ((CC1TermCCDebounce == CCTypeOpen) || (CC1TermCCDebounce == CCTypeRa))) 
	{
		if (VbusVSafe0V()) {												
			if(blnSnkPreferred)
			{
				SetStateTrySink();
			}
			else
			{
				SetStateAttachedSource();										   
			}
		}
	}
	else if ((CC1TermPrevious == CCTypeOpen) && (CC2TermPrevious == CCTypeOpen))	  
		SetStateDelayUnattached();
	else if ((CC1TermPrevious == CCTypeOpen) && (CC2TermPrevious == CCTypeRa))		  
		SetStateDelayUnattached();
	else if ((CC1TermPrevious == CCTypeRa) && (CC2TermPrevious == CCTypeOpen))		  
		SetStateDelayUnattached();

}

void StateMachineAttachWaitAccessory(void)
{
	debounceCC();

	if ((CC1TermCCDebounce == CCTypeRa) && (CC2TermCCDebounce == CCTypeRa))				  
	{
		SetStateAudioAccessory();
	}
	else if ((CC1TermCCDebounce >= CCTypeRdUSB) && (CC1TermCCDebounce < CCTypeUndefined) && (CC2TermCCDebounce >= CCTypeRdUSB) && (CC2TermCCDebounce < CCTypeUndefined))			
	{
		SetStateDebugAccessory();
	}
	else if ((CC1TermCCDebounce == CCTypeOpen) && (CC2TermCCDebounce == CCTypeOpen))	  
	{
		SetStateDelayUnattached();
	}
	else if ((CC1TermCCDebounce >= CCTypeRdUSB) && (CC1TermCCDebounce < CCTypeUndefined) && (CC2TermCCDebounce == CCTypeRa))		   
	{
		SetStatePoweredAccessory();
	}
	else if ((CC1TermCCDebounce == CCTypeRa) && (CC2TermCCDebounce >= CCTypeRdUSB) && (CC2TermCCDebounce < CCTypeUndefined))		   
	{
		SetStatePoweredAccessory();
	}
}

void StateMachineAttachedSink(void)
{
	

	if ((!PRSwapTimer) && (IsHardReset == FALSE) && VbusUnder5V())				 
	{
		SetStateDelayUnattached();												
	}

	if (blnCCPinIsCC1)
	{
		UpdateSinkCurrent(CC1TermCCDebounce);									
	}
	else if(blnCCPinIsCC2)
	{
		UpdateSinkCurrent(CC2TermCCDebounce);									
	}

}

void StateMachineAttachedSource(void)
{
	debounceCC();

	if(Registers.Switches.MEAS_CC1)
	{
		if ((CC1TermPrevious == CCTypeOpen) && (!PRSwapTimer))						 
		{
			if ((PortType == USBTypeC_DRP) && blnSrcPreferred) {				 
				SetStateTryWaitSink();
			}
			else {																 
				SetStateDelayUnattached();
			}
		}
	}
	else if (Registers.Switches.MEAS_CC2)
	{
		if ((CC2TermPrevious == CCTypeOpen) && (!PRSwapTimer))						 
		{
			if ((PortType == USBTypeC_DRP) && blnSrcPreferred) {				 
				SetStateTryWaitSink();
			}
			else {																 
				SetStateDelayUnattached();
			}
		}
	}

}

void StateMachineTryWaitSink(void)
{
	debounceCC();

	if ((StateTimer == 0) && (CC1TermPrevious == CCTypeOpen) && (CC2TermPrevious == CCTypeOpen))  
		SetStateDelayUnattached();												
	else if (Registers.Status.VBUSOK)				   
	{
		if ((CC1TermCCDebounce > CCTypeOpen) && (CC2TermCCDebounce == CCTypeOpen))				  
		{											 
			SetStateAttachedSink();												
		}
		else if ((CC1TermCCDebounce == CCTypeOpen) && (CC2TermCCDebounce > CCTypeOpen))			  
		{
			SetStateAttachedSink();												
		}
	}
}

void StateMachineTrySource(void)
{
	debounceCC();

	if ((CC1TermPDDebounce > CCTypeRa) && (CC1TermPDDebounce < CCTypeUndefined) && ((CC2TermPDDebounce == CCTypeOpen) || (CC2TermPDDebounce == CCTypeRa)))	  
	{
		SetStateAttachedSource();												   
	}
	else if ((CC2TermPDDebounce > CCTypeRa) && (CC2TermPDDebounce < CCTypeUndefined) && ((CC1TermPDDebounce == CCTypeOpen) || (CC1TermPDDebounce == CCTypeRa)))   
	{
		SetStateAttachedSource();												   
	}
	else if (StateTimer == 0)													
		SetStateTryWaitSink();													 
}

void StateMachineDebugAccessory(void)
{
	debounceCC();

	if ((CC1TermCCDebounce == CCTypeOpen) || (CC2TermCCDebounce == CCTypeOpen)) 
	{
		if(PortType == USBTypeC_Source)
		{
			SetStateUnattachedSource();
		}
		else
		{
			SetStateDelayUnattached();
		}
	}

}

void StateMachineAudioAccessory(void)
{
	debounceCC();

	if ((CC1TermCCDebounce == CCTypeOpen) || (CC2TermCCDebounce == CCTypeOpen)) 
	{
		if(PortType == USBTypeC_Source)
		{
			SetStateUnattachedSource();
		}
		else
		{
			SetStateDelayUnattached();
		}
	}
#ifdef FSC_INTERRUPT_TRIGGERED
	if(((CC1TermPrevious == CCTypeOpen) || (CC2TermPrevious == CCTypeOpen)) && (g_Idle == TRUE))
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
	if((CC1TermPDDebounce == CCTypeRa) && (CC2TermPDDebounce == CCTypeRa))		
	{
		g_Idle = TRUE;															
		Registers.Mask.byte = 0xFF;
		Registers.Mask.M_COMP_CHNG = 0;
		DeviceWrite(regMask, 1, &Registers.Mask.byte);
		Registers.MaskAdv.byte[0] = 0xFF;
		DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
		Registers.MaskAdv.M_GCRCSENT = 1;
		DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
		platform_enable_timer(FALSE);
	}
#endif
}

void StateMachinePoweredAccessory(void) 
{
	debounceCC();

	if(blnCCPinIsCC1 && (CC1TermPrevious == CCTypeOpen))	
	{
		SetStateDelayUnattached();
	}
	else if(blnCCPinIsCC2 && (CC2TermPrevious == CCTypeOpen))	 
	{
		SetStateDelayUnattached();
	}
	
	else if (StateTimer == 0)													
		SetStateUnsupportedAccessory();											
}

void StateMachineUnsupportedAccessory(void)
{
	debounceCC();

	if((blnCCPinIsCC1) && (CC1TermPrevious == CCTypeOpen))	  
	{
		SetStateDelayUnattached();
	}
	else if((blnCCPinIsCC2) && (CC2TermPrevious == CCTypeOpen))    
	{
		SetStateDelayUnattached();
	}
}

void stateMachineTrySink(void)
{
	if (StateTimer == 0)
	{
		debounceCC();
	}

	if(Registers.Status.VBUSOK)
	{
		if ((CC1TermPDDebounce >= CCTypeRdUSB) && (CC1TermPDDebounce < CCTypeUndefined) && (CC2TermPDDebounce == CCTypeOpen))	 
		{
			SetStateAttachedSink();													 
		}
		else if ((CC2TermPDDebounce >= CCTypeRdUSB) && (CC2TermPDDebounce < CCTypeUndefined) && (CC1TermPDDebounce == CCTypeOpen))	 
		{
			SetStateAttachedSink();													 
		}
	}

	if ((CC1TermPDDebounce == CCTypeOpen) && (CC2TermPDDebounce == CCTypeOpen))
	{
		SetStateTryWaitSource();
	}
}

void stateMachineTryWaitSource(void)
{
	debounceCC();

	if(VbusVSafe0V())
	{
		if (((CC1TermPDDebounce >= CCTypeRdUSB) && (CC1TermPDDebounce < CCTypeUndefined)) && ((CC2TermPDDebounce == CCTypeRa) || CC2TermPDDebounce == CCTypeOpen))	  
		{
			SetStateAttachedSource();												   
		}
		else if (((CC2TermPDDebounce >= CCTypeRdUSB) && (CC2TermPDDebounce < CCTypeUndefined)) && ((CC1TermPDDebounce == CCTypeRa) || CC1TermPDDebounce == CCTypeOpen))   
		{
			SetStateAttachedSource();												   
		}
	}

	if(StateTimer == 0)
	{
		if ((CC1TermPrevious == CCTypeOpen) && (CC1TermPrevious == CCTypeOpen))
		{
			SetStateDelayUnattached();
		}
	}
}

void stateMachineUnattachedSource(void)
{
	if(alternateModes)
	{
		StateMachineAlternateUnattachedSource();
		return;
	}

	debounceCC();

	if ((CC1TermPrevious == CCTypeRa) && (CC2TermPrevious == CCTypeRa))
	{
		SetStateAttachWaitSource();
	}

	if ((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious < CCTypeUndefined) && ((CC2TermPrevious == CCTypeRa) || CC2TermPrevious == CCTypeOpen))	
	{
		blnCCPinIsCC1 = TRUE;													
		blnCCPinIsCC2 = FALSE;
		SetStateAttachWaitSource();													 
	}
	else if ((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious < CCTypeUndefined) && ((CC1TermPrevious == CCTypeRa) || CC1TermPrevious == CCTypeOpen))	
	{
		blnCCPinIsCC1 = FALSE;													
		blnCCPinIsCC2 = TRUE;
		SetStateAttachWaitSource();													 
	}

	if (DRPToggleTimer == 0)
	{
		SetStateDelayUnattached();
	}

}


void SetStateDisabled(void)
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
	Registers.Power.PWR = 0x1;									   
	Registers.Control.TOGGLE = 0;									
	Registers.Control.HOST_CUR = 0b00;								
	Registers.Switches.byte[0] = 0x00;								
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	DeviceWrite(regControl0, 3, &Registers.Control.byte[0]);	   
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	USBPDDisable(TRUE);											   
	resetDebounceVariables();
	blnCCPinIsCC1 = FALSE;											
	blnCCPinIsCC2 = FALSE;											
	ConnState = Disabled;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = T_TIMER_DISABLE;									  
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateErrorRecovery(void)
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
	Registers.Power.PWR = 0x1;									   
	Registers.Control.TOGGLE = 0;									
	Registers.Control.HOST_CUR = 0b00;								
	Registers.Switches.byte[0] = 0x00;								 
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	DeviceWrite(regControl0, 3, &Registers.Control.byte[0]);	   
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	USBPDDisable(TRUE);											   
	resetDebounceVariables();
	blnCCPinIsCC1 = FALSE;											
	blnCCPinIsCC2 = FALSE;											
	ConnState = ErrorRecovery;										
	StateTimer = tErrorRecovery;									
	PDDebounce = T_TIMER_DISABLE;									  
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	IsHardReset = FALSE;
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateDelayUnattached(void)
{
#ifndef FPGA_BOARD
	SetStateUnattached();
	return;
#else
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
	Registers.Power.PWR = 0x1;									   
	Registers.Control.TOGGLE = 0;									
	Registers.Switches.word &= 0x6800;								
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	DeviceWrite(regControl0, 3, &Registers.Control.byte[0]);	   
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	USBPDDisable(TRUE);											   
	resetDebounceVariables();
	blnCCPinIsCC1 = FALSE;											
	blnCCPinIsCC2 = FALSE;											
	ConnState = DelayUnattached;									
	StateTimer = rand() % 64;										
	PDDebounce = T_TIMER_DISABLE;									  
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
#endif 
}

void SetStateUnattached(void)
{
	if (host_mode_status == 1) {
		printk("FUSB  [%s]:  Leave Host mode\n", __func__);
		host_mode_status = 0;
		msm_otg_set_id_state(1);
	}

	if(alternateModes)
	{
		SetStateAlternateUnattached();
		return;
	}
#ifdef FSC_INTERRUPT_TRIGGERED
	g_Idle = TRUE;																
	Registers.Mask.byte = 0xFF;
	DeviceWrite(regMask, 1, &Registers.Mask.byte);
	Registers.MaskAdv.byte[0] = 0xFF;
	Registers.MaskAdv.M_TOGDONE = 0;
	DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	platform_enable_timer(FALSE);
#endif

	
	
	platform_set_vbus_5v_enable(FALSE);										  
	platform_set_vbus_lvl1_enable(FALSE);									   
	Registers.Switches.byte[0] = 0x00;								 
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	if(ConnState == AttachedSource)												
	{
		while(!VbusVSafe0V());											
	}
	Registers.Control.TOGGLE = 0;
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);	   
	Registers.Switches.byte[0] = 0x03;								 
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	if ((PortType == USBTypeC_DRP) || ((PortType == USBTypeC_Sink) && (blnAccSupport)))   
		Registers.Control.MODE = 0b01;								
	else if (PortType == USBTypeC_Source)							
		Registers.Control.MODE = 0b11;								
	else															
		Registers.Control.MODE = 0b10;								
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	Registers.Control.HOST_CUR = 0b01;								
	Registers.Control.TOGGLE = 1;									
	platform_delay_10us(1);													 
	DeviceWrite(regControl0, 3, &Registers.Control.byte[0]);	   
	USBPDDisable(TRUE);											   
	ConnState = Unattached;											
	SinkCurrent = utccNone;
	resetDebounceVariables();
	blnCCPinIsCC1 = FALSE;											
	blnCCPinIsCC2 = FALSE;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = T_TIMER_DISABLE;									  
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAttachWaitSink(void)
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
	ConnState = AttachWaitSink;										
	sourceOrSink = Sink;
	Registers.Control.TOGGLE = 0;												
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);					
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;													
	DeviceWrite(regPower, 1, &Registers.Power.byte);							
	Registers.Switches.byte[0] = 0x07;											
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);					
	resetDebounceVariables();
	SinkCurrent = utccNone;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = tCCDebounce;									  
	ToggleTimer = tDeviceToggle;								   
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAttachWaitSource(void)
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
	Registers.Control.TOGGLE = 0;									
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);	   

	updateSourceCurrent();
	ConnState = AttachWaitSource;									
	sourceOrSink = Source;
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)												
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;							 
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else
	{
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;							 
		setDebounceVariablesCC2(CCTypeUndefined);
	}

	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	OverPDDebounce = T_TIMER_DISABLE;  
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = tCCDebounce;									  
	ToggleTimer = T_TIMER_DISABLE;											   
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAttachWaitAccessory(void)
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
	ConnState = AttachWaitAccessory;								
	sourceOrSink = Source;
	updateSourceCurrent();
	Registers.Control.TOGGLE = 0;									
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);	   
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)												
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;							 
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else
	{
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;							 
		setDebounceVariablesCC2(CCTypeUndefined);
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	OverPDDebounce = T_TIMER_DISABLE;  
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tCCDebounce;								 
	CCDebounce = tCCDebounce;									  
	ToggleTimer = T_TIMER_DISABLE;									 
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAttachedSource(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
	g_Idle = TRUE;																
	Registers.Mask.byte = 0xFF;
	Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(regMask, 1, &Registers.Mask.byte);
	Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	platform_enable_timer(FALSE);
#endif
	platform_set_vbus_5v_enable(TRUE);											
	platform_set_vbus_lvl1_enable(FALSE);										
	ConnState = AttachedSource;													
	sourceOrSink = Source;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;													
	DeviceWrite(regPower, 1, &Registers.Power.byte);							

	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))					
	{
		DetectCCPinSource();
	}

	if ((host_mode_status == 0) && ((CC1TermPrevious == CCTypeRdUSB) || (CC2TermPrevious == CCTypeRdUSB)) ) {
		host_mode_status = 1;
		printk("FUSB  [%s]: Enter Host mode\n", __func__);
		msm_otg_set_id_state(0);
	} else if ((host_mode_status == 1) && ((CC1TermPrevious == CCTypeRdUSB) || (CC2TermPrevious == CCTypeRdUSB)) ) {
		printk("FUSB  [%s]: Already in Host mode, skip this interrupt\n", __func__);
	}

	if (blnCCPinIsCC1)															
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;										
		if(CC2TermPrevious == CCTypeRa)											
		{
			Registers.Switches.VCONN_CC2 = 1;
		}
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else																		
	{
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;										
		if(CC1TermPrevious == CCTypeRa)											
		{
			Registers.Switches.VCONN_CC1 = 1;
		}
		setDebounceVariablesCC2(CCTypeUndefined);
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);					
	USBPDEnable(TRUE, TRUE);													
	SinkCurrent = utccNone;														
	StateTimer = T_TIMER_DISABLE;												
	PDDebounce = tPDDebounce;												
	CCDebounce = tCCDebounce;												
	ToggleTimer = T_TIMER_DISABLE;												
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);

}

void SetStateAttachedSink(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
	g_Idle = TRUE;																
	Registers.Mask.byte = 0xFF;
	Registers.Mask.M_VBUSOK = 0;
	DeviceWrite(regMask, 1, &Registers.Mask.byte);
	Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	platform_enable_timer(FALSE);
#endif
	platform_set_vbus_5v_enable(FALSE);										  
	platform_set_vbus_lvl1_enable(FALSE);									   
	ConnState = AttachedSink;										
	sourceOrSink = Sink;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;													
	DeviceWrite(regPower, 1, &Registers.Power.byte);							
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))					
	{
		DetectCCPinSink();
	}
	if (blnCCPinIsCC1)															
	{
		peekCC2Sink();
		peekCC1Sink();
		Registers.Switches.byte[0] = 0x07;
	}
	else																		
	{
		peekCC1Sink();
		peekCC2Sink();
		Registers.Switches.byte[0] = 0x0B;
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);					
	USBPDEnable(TRUE, FALSE);									   
	SinkCurrent = utccDefault;										
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void RoleSwapToAttachedSink(void)
{
	platform_set_vbus_5v_enable( FALSE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	while(!VbusVSafe0V());										  
	ConnState = AttachedSink;										
	sourceOrSink = Sink;
	updateSourceCurrent();
	if (blnCCPinIsCC1)												
	{
		
		Registers.Switches.PU_EN1 = 0;								
		Registers.Switches.PDWN1 = 1;								
	}
	else
	{
		
		Registers.Switches.PU_EN2 = 0;								
		Registers.Switches.PDWN2 = 1;								
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = tCCDebounce;									  
	ToggleTimer = T_TIMER_DISABLE;
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void RoleSwapToAttachedSource(void)
{
	platform_set_vbus_5v_enable( TRUE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	ConnState = AttachedSource;										
	sourceOrSink = Source;
	updateSourceCurrent();
	if (blnCCPinIsCC1)												
	{
		Registers.Switches.PU_EN1 = 1;								
		Registers.Switches.PDWN1 = 0;								
		Registers.Switches.MEAS_CC1 = 1;
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else
	{
		Registers.Switches.PU_EN2 = 1;								
		Registers.Switches.PDWN2 = 0;								
		Registers.Switches.MEAS_CC2 = 1;
		setDebounceVariablesCC2(CCTypeUndefined);
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = tCCDebounce;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateTryWaitSink(void)
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
	ConnState = TryWaitSink;										
	sourceOrSink = Sink;
	updateSourceCurrent();
	Registers.Switches.byte[0] = 0x07;								 
	Registers.Power.PWR = 0x7;									   
	resetDebounceVariables();
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	SinkCurrent = utccNone;											
	StateTimer = tDRPTryWait;										
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = tDeviceToggle;								   
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);

}

void SetStateTrySource(void)
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
	ConnState = TrySource;											
	sourceOrSink = Source;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)												
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;								
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else
	{
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;								
		setDebounceVariablesCC2(CCTypeUndefined);
	}

	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	OverPDDebounce = T_TIMER_DISABLE;  
	StateTimer = tDRPTry;											
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;									 
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateTrySink(void)
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
	ConnState = TrySink;										  
	sourceOrSink = Sink;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	Registers.Switches.byte[0] = 0x07;								 
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	resetDebounceVariables();
	SinkCurrent = utccNone;											
	StateTimer = tDRPTry;											
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = tDeviceToggle;								   
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateTryWaitSource(void)
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
	updateSourceCurrent();
	ConnState = TryWaitSource;									 
	sourceOrSink = Source;
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)												
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;							 
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else
	{
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;							 
		setDebounceVariablesCC2(CCTypeUndefined);
	}

	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	OverPDDebounce = T_TIMER_DISABLE;  
	StateTimer = tDRPTry;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;											   
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateDebugAccessory(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
	g_Idle = TRUE;																
	Registers.Mask.byte = 0xFF;
	Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(regMask, 1, &Registers.Mask.byte);
	Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	platform_enable_timer(FALSE);
#endif
	platform_set_vbus_5v_enable( FALSE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	ConnState = DebugAccessory;										
	sourceOrSink = Source;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)										
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;							
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else														   
	{
		peekCC1Source();
		setDebounceVariablesCC2(CCTypeUndefined);
		Registers.Switches.byte[0] = 0x88;						   
	}

	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = T_TIMER_DISABLE;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAudioAccessory(void)
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
	if(alternateModes)
	{
		SetStateAlternateAudioAccessory();
		return;
	}
	platform_set_vbus_5v_enable( FALSE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	ConnState = AudioAccessory;										
	sourceOrSink = Source;
	updateSourceCurrent();
	Registers.Power.PWR = 0x7;									   
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)										
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;							
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else														   
	{
		peekCC1Source();
		setDebounceVariablesCC2(CCTypeUndefined);
		Registers.Switches.byte[0] = 0x88;						   
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	OverPDDebounce = T_TIMER_DISABLE;  
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tCCDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStatePoweredAccessory(void)
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
	ConnState = PoweredAccessory;									
	sourceOrSink = Source;
	Registers.Power.PWR = 0x7;
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if (SourceCurrent == utccDefault)	
	{
		Registers.Control.HOST_CUR = 0b10;
		DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);
	}
	else
	{
		updateSourceCurrent();
	}
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)										
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x64;							
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else														   
	{
		blnCCPinIsCC2 = TRUE;
		peekCC1Source();
		setDebounceVariablesCC2(CCTypeUndefined);
		Registers.Switches.byte[0] = 0x98;						   
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	
	USBPDEnable(TRUE, TRUE);

	SinkCurrent = utccNone;											
	StateTimer = tAMETimeout;										
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateUnsupportedAccessory(void)
{
#ifdef FSC_INTERRUPT_TRIGGERED
	g_Idle = TRUE;																
	Registers.Mask.byte = 0xFF;
	Registers.Mask.M_COMP_CHNG = 0;
	DeviceWrite(regMask, 1, &Registers.Mask.byte);
	Registers.MaskAdv.byte[0] = 0xFF;
	DeviceWrite(regMaska, 1, &Registers.MaskAdv.byte[0]);
	Registers.MaskAdv.M_GCRCSENT = 1;
	DeviceWrite(regMaskb, 1, &Registers.MaskAdv.byte[1]);
	platform_enable_timer(FALSE);
#endif
	platform_set_vbus_5v_enable( FALSE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	ConnState = UnsupportedAccessory;								
	sourceOrSink = Source;
	Registers.Control.HOST_CUR = 0b01;								
	DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);
	Registers.Power.PWR = 0x7;
	DeviceWrite(regPower, 1, &Registers.Power.byte);			   
	if ((blnCCPinIsCC1 == FALSE) && (blnCCPinIsCC2 == FALSE))			
	{
		DetectCCPinSource();
	}
	if (blnCCPinIsCC1)										
	{
		peekCC2Source();
		Registers.Switches.byte[0] = 0x44;
		setDebounceVariablesCC1(CCTypeUndefined);
	}
	else														   
	{
		blnCCPinIsCC2 = TRUE;
		peekCC1Source();
		Registers.Switches.byte[0] = 0x88;
		setDebounceVariablesCC2(CCTypeUndefined);
	}
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);	   
	SinkCurrent = utccNone;											
	StateTimer = T_TIMER_DISABLE;										  
	PDDebounce = tPDDebounce;								 
	CCDebounce = T_TIMER_DISABLE;									  
	ToggleTimer = T_TIMER_DISABLE;										  
	OverPDDebounce = T_TIMER_DISABLE;  
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateUnattachedSource(void) 
{
	if(alternateModes)
	{
		SetStateAlternateUnattachedSource();
		return;
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
	platform_set_vbus_5v_enable( FALSE );										
	platform_set_vbus_lvl1_enable( FALSE );										 
	ConnState = UnattachedSource;										  
	sourceOrSink = Source;
	Registers.Switches.byte[0] = 0x44;								
	Registers.Power.PWR = 0x7;										
	updateSourceCurrent();											
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
	DRPToggleTimer = tTOG2;												
	OverPDDebounce = tPDDebounce;								   
	WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void updateSourceCurrent(void)
{
	blnCurrentUpdate = TRUE;

	switch(SourceCurrent)
	{
		case utccDefault:
			Registers.Control.HOST_CUR = 0b01;						
			break;
		case utcc1p5A:
			Registers.Control.HOST_CUR = 0b10;						
			break;
		case utcc3p0A:
			Registers.Control.HOST_CUR = 0b11;						
			break;
		default:													
			Registers.Control.HOST_CUR = 0b00;						
			break;
	}
	DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);	   
}

void updateSourceMDACHigh(void)
{
	blnMDACHigh = TRUE;
	blnCurrentUpdate = FALSE;
	switch(SourceCurrent)
	{
		case utccDefault:
			Registers.Measure.MDAC = MDAC_1P6V;						
			break;
		case utcc1p5A:
			Registers.Measure.MDAC = MDAC_1P6V;						
			break;
		case utcc3p0A:
			Registers.Measure.MDAC = MDAC_2P6V;						
			break;
		default:													
			Registers.Measure.MDAC = MDAC_1P6V;						
			break;
	}
	DeviceWrite(regMeasure, 1, &Registers.Measure.byte);		   
}

void updateSourceMDACLow(void)
{
	blnMDACHigh = FALSE;
	blnCurrentUpdate = FALSE;

	switch(SourceCurrent)
	{
		case utccDefault:
			Registers.Measure.MDAC = MDAC_0P2V;						
			break;
		case utcc1p5A:
			Registers.Measure.MDAC = MDAC_0P4V;						
			break;
		case utcc3p0A:
			Registers.Measure.MDAC = MDAC_0P8V;						
			break;
		default:													
			Registers.Measure.MDAC = MDAC_1P6V;						
			break;
	}
	DeviceWrite(regMeasure, 1, &Registers.Measure.byte);		   
}


void ToggleMeasureCC1(void)
{
	if(!alternateModes)
	{
		Registers.Switches.PU_EN1 = Registers.Switches.PU_EN2;					
		Registers.Switches.PU_EN2 = 0;											
	}
	Registers.Switches.MEAS_CC1 = 1;										
	Registers.Switches.MEAS_CC2 = 0;										
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);			   

}

void ToggleMeasureCC2(void)
{
	if(!alternateModes)
	{
		Registers.Switches.PU_EN2 = Registers.Switches.PU_EN1;					
		Registers.Switches.PU_EN1 = 0;											
	}
	Registers.Switches.MEAS_CC1 = 0;										
	Registers.Switches.MEAS_CC2 = 1;										
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);			   
 }

CCTermType DecodeCCTermination(void)
{
	if(alternateModes)
	{
		AlternateDRPSourceSinkSwap();
	}
	switch (sourceOrSink)
	{
		case Source:
			if(alternateModes)
			{
				AlternateDRPSourceSinkSwap();
			}
			return DecodeCCTerminationSource();
			break;
		case Sink:
			if(alternateModes)
			{
				AlternateDRPSourceSinkSwap();
			}
			return DecodeCCTerminationSink();
			break;
		default:
			return CCTypeUndefined;
			break;
	}
}

CCTermType DecodeCCTerminationSource(void)
{
	CCTermType Termination = CCTypeUndefined;			 

	updateSourceMDACHigh();
	platform_delay_10us(25);															  
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);

	if (Registers.Status.COMP == 1)
	{
		Termination = CCTypeOpen;
		return Termination;
	}
	
	else if((ConnState == AttachedSource) && ((Registers.Switches.MEAS_CC1 && blnCCPinIsCC1) || (Registers.Switches.MEAS_CC2 && blnCCPinIsCC2)))
	{
		switch(SourceCurrent)
		{
			case utccDefault:
				Termination = CCTypeRdUSB;
				break;
			case utcc1p5A:
				Termination = CCTypeRd1p5;
				break;
			case utcc3p0A:
				Termination = CCTypeRd3p0;
				break;
			case utccNone:
				break;
		}
		return Termination;
	}

	updateSourceMDACLow();
	platform_delay_10us(25);															  
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);

	if(Registers.Status.COMP == 0)
	{
		Termination = CCTypeRa;
	}
	else
	{
		switch(SourceCurrent)
		{
			case utccDefault:
				Termination = CCTypeRdUSB;
				break;
			case utcc1p5A:
				Termination = CCTypeRd1p5;
				break;
			case utcc3p0A:
				Termination = CCTypeRd3p0;
				break;
			case utccNone:
				break;
		}
	}
	return Termination;								
}

CCTermType DecodeCCTerminationSink(void) 
{
	CCTermType Termination;
	platform_delay_10us(25);															  
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);

	{
		switch (Registers.Status.BC_LVL)			
		{
			case 0b00:								
				Termination = CCTypeOpen;
				break;
			case 0b01:								
				Termination = CCTypeRdUSB;
				break;
			case 0b10:								
				Termination = CCTypeRd1p5;
				break;
			default:								
				Termination = CCTypeRd3p0;
				break;
		}
	}
	return Termination;								
}

void UpdateSinkCurrent(CCTermType Termination)
{
	switch (Termination)
	{
		case CCTypeRdUSB:						
			SinkCurrent = utccDefault;
			break;
		case CCTypeRd1p5:						
			SinkCurrent = utcc1p5A;
			break;
		case CCTypeRd3p0:						
			SinkCurrent = utcc3p0A;
			break;
		default:
			SinkCurrent = utccNone;
			break;
	}
}


void ConfigurePortType(UINT8 Control)
{
	UINT8 value;
	BOOL setUnattached = FALSE;
	DisableTypeCStateMachine();
	value = Control & 0x03;
	if (PortType != value)
	{
		switch (value)
		{
			case 1:
				PortType = USBTypeC_Source;
				break;
			case 2:
				PortType = USBTypeC_DRP;
				break;
			default:
				PortType = USBTypeC_Sink;
				break;
		}

		setUnattached = TRUE;
	}

	if (((Control & 0x04) >> 2) != blnAccSupport)
	{
		if (Control & 0x04)
		{
			blnAccSupport = TRUE;
		}
		else
		{
			blnAccSupport = FALSE;
		}
		setUnattached = TRUE;
	}

	if (((Control & 0x08) >> 3) != blnSrcPreferred)
	{
		if (Control & 0x08)
		{
			blnSrcPreferred = TRUE;
		}
		else
		{
			blnSrcPreferred = FALSE;
		}
		setUnattached = TRUE;
	}

	if (((Control & 0x40) >> 5) != blnSnkPreferred)
	{
		if (Control & 0x40)
		{
			blnSnkPreferred = TRUE;
		}
		else
		{
			blnSnkPreferred = FALSE;
		}
		setUnattached = TRUE;
	}

	if(setUnattached)
	{
		SetStateDelayUnattached();
	}

	value = (Control & 0x30) >> 4;
	if (SourceCurrent != value){
		switch (value)
		{
			case 1:
				SourceCurrent = utccDefault;
				break;
			case 2:
				SourceCurrent = utcc1p5A;
				break;
			case 3:
				SourceCurrent = utcc3p0A;
				break;
			default:
				SourceCurrent = utccNone;
				break;
		}
		updateSourceCurrent();
	}
	if (Control & 0x80)
		EnableTypeCStateMachine();
}

void UpdateCurrentAdvert(UINT8 Current)
{
	switch (Current)
	{
		case 1:
			SourceCurrent = utccDefault;
			break;
		case 2:
			SourceCurrent = utcc1p5A;
			break;
		case 3:
			SourceCurrent = utcc3p0A;
			break;
		default:
			SourceCurrent = utccNone;
			break;
	}
		updateSourceCurrent();
}

void GetDeviceTypeCStatus(UINT8 abytData[])
{
	INT32 intIndex = 0;
	abytData[intIndex++] = GetTypeCSMControl();		
	abytData[intIndex++] = ConnState & 0xFF;		
	abytData[intIndex++] = GetCCTermination();		
	abytData[intIndex++] = SinkCurrent;				
}

UINT8 GetTypeCSMControl(void)
{
	UINT8 status = 0;
	status |= (PortType & 0x03);			
	switch(PortType)						
	{
		case USBTypeC_Source:
			status |= 0x01;					
			break;
		case USBTypeC_DRP:
			status |= 0x02;					
			break;
		default:							
			break;
	}
	if (blnAccSupport)						
		status |= 0x04;
	if (blnSrcPreferred)					
		status |= 0x08;
	status |= (SourceCurrent << 4);
	if (blnSnkPreferred)
		status |= 0x40;
	if (blnSMEnabled)						
		status |= 0x80;
	return status;
}

UINT8 GetCCTermination(void)
{
	UINT8 status = 0;

			status |= (CC1TermPrevious & 0x07);			 
		
		
			status |= ((CC2TermPrevious & 0x07) << 4);	 
		
		

	return status;
}

BOOL VbusVSafe0V (void)
{
#ifdef FPGA_BOARD
	DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);
	if (Registers.Status.VBUSOK) {
		return FALSE;
	} else {
		return TRUE;
	}
#else
	regSwitches_t switches;
	regSwitches_t saved_switches;

	regMeasure_t measure;
	regMeasure_t saved_measure;

	UINT8 val;
	BOOL ret;

	DeviceRead(regSwitches0, 1, &(switches.byte[0]));							
	saved_switches = switches;
	switches.MEAS_CC1 = 0;														
	switches.MEAS_CC2 = 0;
	DeviceWrite(regSwitches0, 1, &(switches.byte[0]));							

	DeviceRead(regMeasure, 1, &measure.byte);									
	saved_measure = measure;
	measure.MEAS_VBUS = 1;														
	measure.MDAC = VBUS_MDAC_0P8V;												
	DeviceWrite(regMeasure, 1, &measure.byte);									

	platform_delay_10us(25);													

	DeviceRead(regStatus0, 1, &val);											
	val &= 0x20;																

	if (val)	ret = FALSE;													
	else		ret = TRUE;

	DeviceWrite(regSwitches0, 1, &(saved_switches.byte[0]));					
	DeviceWrite(regMeasure, 1, &saved_measure.byte);
	platform_delay_10us(25);													
	return ret;
#endif
}

BOOL VbusUnder5V(void)															 
{
	regMeasure_t measure;

	UINT8 val;
	BOOL ret;

	measure = Registers.Measure;
	measure.MEAS_VBUS = 1;														
	measure.MDAC = VBUS_MDAC_3p8;
	DeviceWrite(regMeasure, 1, &measure.byte);									

	platform_delay_10us(25);													

	DeviceRead(regStatus0, 1, &val);											
	val &= 0x20;																

	if (val)	ret = FALSE;													
	else		ret = TRUE;

				   
	DeviceWrite(regMeasure, 1, &Registers.Measure.byte);
	return ret;
}

void DetectCCPinSource(void)  
{
	CCTermType CCTerm;

	Registers.Switches.byte[0] = 0x44;
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));
	CCTerm = DecodeCCTermination();
	if ((CCTerm >= CCTypeRdUSB) && (CCTerm < CCTypeUndefined))
	{
		blnCCPinIsCC1 = TRUE;													
		blnCCPinIsCC2 = FALSE;
		return;
	}

	Registers.Switches.byte[0] = 0x88;
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));
	CCTerm = DecodeCCTermination();
	if ((CCTerm >= CCTypeRdUSB) && (CCTerm < CCTypeUndefined))
	{

		blnCCPinIsCC1 = FALSE;													
		blnCCPinIsCC2 = TRUE;
		return;
	}
}

void DetectCCPinSink(void)	
{
	CCTermType CCTerm;

	Registers.Switches.byte[0] = 0x07;
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));
	CCTerm = DecodeCCTermination();
	if ((CCTerm > CCTypeRa) && (CCTerm < CCTypeUndefined))
	{
		blnCCPinIsCC1 = TRUE;													
		blnCCPinIsCC2 = FALSE;
		return;
	}

	Registers.Switches.byte[0] = 0x0B;
	DeviceWrite(regSwitches0, 1, &(Registers.Switches.byte[0]));
	CCTerm = DecodeCCTermination();
	if ((CCTerm > CCTypeRa) && (CCTerm < CCTypeUndefined))
	{

		blnCCPinIsCC1 = FALSE;													
		blnCCPinIsCC2 = TRUE;
		return;
	}
}

void resetDebounceVariables(void)
{
	CC1TermPrevious = CCTypeUndefined;
	CC2TermPrevious = CCTypeUndefined;
	CC1TermCCDebounce = CCTypeUndefined;
	CC2TermCCDebounce = CCTypeUndefined;
	CC1TermPDDebounce = CCTypeUndefined;
	CC2TermPDDebounce = CCTypeUndefined;
	CC1TermPDDebouncePrevious = CCTypeUndefined;
	CC2TermPDDebouncePrevious = CCTypeUndefined;
}

void setDebounceVariablesCC1(CCTermType term)
{
	CC1TermPrevious = term;
	CC1TermCCDebounce = term;
	CC1TermPDDebounce = term;
	CC1TermPDDebouncePrevious = term;

}

void setDebounceVariablesCC2(CCTermType term)
{

	CC2TermPrevious = term;
	CC2TermCCDebounce = term;
	CC2TermPDDebounce = term;
	CC2TermPDDebouncePrevious = term;
}

BOOL GetLocalRegisters(UINT8 * data, INT32 size){  
	if (size!=23) return FALSE;



	data[0] = Registers.DeviceID.byte;
	data[1]= Registers.Switches.byte[0];
	data[2]= Registers.Switches.byte[1];
	data[3]= Registers.Measure.byte;
	data[4]= Registers.Slice.byte;
	data[5]= Registers.Control.byte[0];
	data[6]= Registers.Control.byte[1];
	data[7]= Registers.Control.byte[2];
	data[8]= Registers.Control.byte[3];
	data[9]= Registers.Mask.byte;
	data[10]= Registers.Power.byte;
	data[11]= Registers.Reset.byte;
	data[12]= Registers.OCPreg.byte;
	data[13]= Registers.MaskAdv.byte[0];
	data[14]= Registers.MaskAdv.byte[1];
	data[15]= Registers.Control4.byte;
	data[16]=Registers.Status.byte[0];
	data[17]=Registers.Status.byte[1];
	data[18]=Registers.Status.byte[2];
	data[19]=Registers.Status.byte[3];
	data[20]=Registers.Status.byte[4];
	data[21]=Registers.Status.byte[5];
	data[22]=Registers.Status.byte[6];

	return TRUE;
}

BOOL GetStateLog(UINT8 * data){   
	INT32 i;
	INT32 entries = TypeCStateLog.Count;
	UINT16 state_temp;
	UINT16 time_tms_temp;
	UINT16 time_s_temp;


	for(i=0; ((i<entries) && (i<12)); i++)
	{
		ReadStateLog(&TypeCStateLog, &state_temp, &time_tms_temp, &time_s_temp);

		data[i*5+1] = state_temp;
		data[i*5+2] = (time_tms_temp>>8);
		data[i*5+3] = (UINT8)time_tms_temp;
		data[i*5+4] = (time_s_temp)>>8;
		data[i*5+5] = (UINT8)time_s_temp;
	}

	data[0] = i;	


	return TRUE;
}

void debounceCC(void)
{
	
	CCTermType CCTermCurrent = DecodeCCTermination();					  
	if (Registers.Switches.MEAS_CC1)											
	{
		if (CC1TermPrevious != CCTermCurrent)								 
		{
			CC1TermPrevious = CCTermCurrent;								 
			PDDebounce = tPDDebounce;										
			if(OverPDDebounce == T_TIMER_DISABLE)							
			{
				OverPDDebounce = tPDDebounce;
			}
		}
	}
	else if(Registers.Switches.MEAS_CC2)										
	{
		if (CC2TermPrevious != CCTermCurrent)								 
		{
			CC2TermPrevious = CCTermCurrent;								 
			PDDebounce = tPDDebounce;										
			if(OverPDDebounce == T_TIMER_DISABLE)							
			{
				OverPDDebounce = tPDDebounce;
			}
		}
	}
	if (PDDebounce == 0)													   
	{
		CC1TermPDDebounce = CC1TermPrevious;							  
		CC2TermPDDebounce = CC2TermPrevious;
		PDDebounce = T_TIMER_DISABLE;
		OverPDDebounce = T_TIMER_DISABLE;
	}
	if (OverPDDebounce == 0)
	{
		CCDebounce = tCCDebounce;
	}

	
	if ((CC1TermPDDebouncePrevious != CC1TermPDDebounce) || (CC2TermPDDebouncePrevious != CC2TermPDDebounce))	
	{
		CC1TermPDDebouncePrevious = CC1TermPDDebounce;					  
		CC2TermPDDebouncePrevious = CC2TermPDDebounce;
		CCDebounce = tCCDebounce - tPDDebounce;							 
		CC1TermCCDebounce = CCTypeUndefined;							  
		CC2TermCCDebounce = CCTypeUndefined;
	}
	if (CCDebounce == 0)
	{
		CC1TermCCDebounce = CC1TermPDDebouncePrevious;					  
		CC2TermCCDebounce = CC2TermPDDebouncePrevious;
		CCDebounce = T_TIMER_DISABLE;
		OverPDDebounce = T_TIMER_DISABLE;
	}

	if (ToggleTimer == 0)														
	{
		if (Registers.Switches.MEAS_CC1)										
			ToggleMeasureCC2();													
		else																	
			ToggleMeasureCC1();													
		ToggleTimer = tDeviceToggle;										   
	}
}

void peekCC1Source(void)
{
	UINT8 saveRegister = Registers.Switches.byte[0];							

	Registers.Switches.byte[0] = 0x44;											
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	setDebounceVariablesCC1(DecodeCCTerminationSource());

	Registers.Switches.byte[0] = saveRegister;									
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
}

void peekCC2Source(void)
{
	UINT8 saveRegister = Registers.Switches.byte[0];							

	Registers.Switches.byte[0] = 0x88;	
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	setDebounceVariablesCC2(DecodeCCTerminationSource());

	Registers.Switches.byte[0] = saveRegister;									
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
}

void peekCC1Sink(void)
{
	UINT8 saveRegister = Registers.Switches.byte[0];							

	Registers.Switches.byte[0] = 0x07;
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	setDebounceVariablesCC1(DecodeCCTermination());

	Registers.Switches.byte[0] = saveRegister;									
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
}

void peekCC2Sink(void)
{
	UINT8 saveRegister = Registers.Switches.byte[0];							

	Registers.Switches.byte[0] = 0x0B;
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
	setDebounceVariablesCC2(DecodeCCTermination());

	Registers.Switches.byte[0] = saveRegister;									
	DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
}

void checkForAccessory(void)
{
	Registers.Control.TOGGLE = 0;
	DeviceWrite(regControl2, 1, &Registers.Control.byte[2]);
	peekCC1Source();
	peekCC2Source();

	if((CC1TermPrevious > CCTypeOpen) && (CC2TermPrevious > CCTypeOpen))	
	{
		SetStateAttachWaitAccessory();
	}
	else																	
	{
		SetStateDelayUnattached();
	}
}

void ProcessTypeCPDStatus(UINT8* MsgBuffer, UINT8* retBuffer)
{
	if (MsgBuffer[1] != 1)									
		retBuffer[1] = 0x01;						 
	else
	{
		GetDeviceTypeCStatus((UINT8*)&retBuffer[4]); 
		GetUSBPDStatus((UINT8*)&retBuffer[8]);		  
	}
}

void ProcessTypeCPDControl(UINT8* MsgBuffer, UINT8* retBuffer)
{
	if (MsgBuffer[1] != 0)
	{
		retBuffer[1] = 0x01;			 
		return;
	}
	switch (MsgBuffer[4])						
	{
		case 0x01:								
			DisableTypeCStateMachine();
			EnableTypeCStateMachine();
			break;
		case 0x02:								
			DisableTypeCStateMachine();
			break;
		case 0x03:								
			EnableTypeCStateMachine();
			break;
		case 0x04:								
			ConfigurePortType(MsgBuffer[5]);
			break;
		case 0x05:								
			UpdateCurrentAdvert(MsgBuffer[5]);
			break;
		case 0x06:								
			EnableUSBPD();
			break;
		case 0x07:								
			DisableUSBPD();
			break;
		case 0x08:								
			SendUSBPDMessage((UINT8*) &MsgBuffer[5]);
			break;
		case 0x09:								
			WriteSourceCapabilities((UINT8*) &MsgBuffer[5]);
			break;
		case 0x0A:								
			retBuffer[4] = MsgBuffer[4]; 
			ReadSourceCapabilities((UINT8*) &retBuffer[5]);
			break;
		case 0x0B:								
			WriteSinkCapabilities((UINT8*) &MsgBuffer[5]);
			break;
		case 0x0C:								
			retBuffer[4] = MsgBuffer[4]; 
			ReadSinkCapabilities((UINT8*) &retBuffer[5]);
			break;
		case 0x0D:								
			WriteSinkRequestSettings((UINT8*) &MsgBuffer[5]);
			break;
		case 0x0E:								
			retBuffer[4] = MsgBuffer[4]; 
			ReadSinkRequestSettings((UINT8*) &retBuffer[5]);
			break;
		case 0x0F:								
			retBuffer[4] = MsgBuffer[4]; 
			SendUSBPDHardReset();				
			break;
		case 0x10:
			retBuffer[4] = MsgBuffer[4]; 
			ConfigureVdmResponses((UINT8*) &MsgBuffer[5]);			  
			break;
		case 0x11:
			retBuffer[4] = MsgBuffer[4]; 
			ReadVdmConfiguration((UINT8*) &retBuffer[5]);			 
			break;
		case 0x12:
			retBuffer[4] = MsgBuffer[4];
			WriteDpControls((UINT8*) &MsgBuffer[5]);
			break;
		case 0x13:
			retBuffer[4] = MsgBuffer[4];
			ReadDpControls((UINT8*) &retBuffer[5]);
			break;
		case 0x14:
			retBuffer[4] = MsgBuffer[4];
			ReadDpStatus((UINT8*) &retBuffer[5]);
			break;
		default:
			break;
	}
}

void ProcessLocalRegisterRequest(UINT8* MsgBuffer, UINT8* retBuffer)  
{
	if (MsgBuffer[1] != 0)
	{
		retBuffer[1] = 0x01;			 
		return;
	}

	GetLocalRegisters(&retBuffer[3], 23);  
}

void ProcessSetTypeCState(UINT8* MsgBuffer, UINT8* retBuffer)	
{
	ConnectionState state = MsgBuffer[3];

	if (MsgBuffer[1] != 0)
	{
		retBuffer[1] = 0x01;			 
		return;
	}

	switch(state)
	{
		case(Disabled):
			SetStateDisabled();
			break;
		case(ErrorRecovery):
			SetStateErrorRecovery();
			break;
		case(Unattached):
			SetStateUnattached();
			break;
		case(AttachWaitSink):
			SetStateAttachWaitSink();
			break;
		case(AttachedSink):
			SetStateAttachedSink();
			break;
		case(AttachWaitSource):
			SetStateAttachWaitSource();
			break;
		case(AttachedSource):
			SetStateAttachedSource();
			break;
		case(TrySource):
			SetStateTrySource();
			break;
		case(TryWaitSink):
			SetStateTryWaitSink();
			break;
		case(TrySink):
			SetStateTrySink();
			break;
		case(TryWaitSource):
			SetStateTryWaitSource();
			break;
		case(AudioAccessory):
			SetStateAudioAccessory();
			break;
		case(DebugAccessory):
			SetStateDebugAccessory();
			break;
		case(AttachWaitAccessory):
			SetStateAttachWaitAccessory();
			break;
		case(PoweredAccessory):
			SetStatePoweredAccessory();
			break;
		case(UnsupportedAccessory):
			SetStateUnsupportedAccessory();
			break;
		case(DelayUnattached):
			SetStateDelayUnattached();
			break;
		case(UnattachedSource):
			SetStateUnattachedSource();
			break;
		default:
			SetStateDelayUnattached();
			break;
	}
}

void ProcessReadTypeCStateLog(UINT8* MsgBuffer, UINT8* retBuffer)
{
	if (MsgBuffer[1] != 0)
	{
		retBuffer[1] = 0x01;			 
		return;
	}

	GetStateLog(&retBuffer[3]);   
}


void setAlternateModes(UINT8 mode){
	alternateModes = mode;
}

UINT8 getAlternateModes(void)
{
	return alternateModes;
}
