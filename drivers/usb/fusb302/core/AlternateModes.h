
#ifndef ALTERNATEMODES_H
#define	ALTERNATEMODES_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "TypeC.h"
#include "platform.h"
#include "Log.h"

#define tAlternateDRPSwap 40 * 10        

extern DeviceReg_t             Registers;          
extern BOOL                    USBPDActive;        
extern BOOL                    USBPDEnabled;       
extern UINT32                  PRSwapTimer;        
extern BOOL                    IsHardReset;        
extern SourceOrSink            sourceOrSink;       

extern USBTypeCPort            PortType;           
extern BOOL                    blnCCPinIsCC1;      
extern BOOL                    blnCCPinIsCC2;      
extern BOOL                    blnSMEnabled;       
extern ConnectionState         ConnState;          
extern StateLog                TypeCStateLog;      
extern volatile UINT16         Timer_S;            
extern volatile UINT16         Timer_tms;          

extern BOOL             blnAccSupport;      
extern UINT16           StateTimer;         
extern UINT16           PDDebounce;     
extern UINT16           CCDebounce;     
extern UINT16           ToggleTimer;        
extern UINT16           DRPToggleTimer;     
extern UINT16           OverPDDebounce;    
extern CCTermType       CC1TermPrevious;         
extern CCTermType       CC2TermPrevious;         
extern CCTermType       CC1TermCCDebounce;         
extern CCTermType       CC2TermCCDebounce;         
extern CCTermType       CC1TermPDDebounce;
extern CCTermType       CC2TermPDDebounce;
extern CCTermType       CC1TermPDDebouncePrevious;
extern CCTermType       CC2TermPDDebouncePrevious;
extern USBTypeCCurrent  SinkCurrent;        

void SetStateAlternateUnattached(void);
void StateMachineAlternateUnattached(void);
void SetStateAlternateDRP(void);
void StateMachineAlternateDRP(void);
void AlternateDRPSwap(void);
void AlternateDRPSourceSinkSwap(void);
void SetStateAlternateUnattachedSource(void);
void StateMachineAlternateUnattachedSource(void);
void StateMachineAlternateUnattachedSink(void);
void SetStateAlternateUnattachedSink(void);
void SetStateAlternateAudioAccessory(void);
CCTermType AlternateDecodeCCTerminationSource(void);


#ifdef	__cplusplus
}
#endif

#endif	

