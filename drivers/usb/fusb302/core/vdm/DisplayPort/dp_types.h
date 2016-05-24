#ifndef __DISPLAYPORT_TYPES_H__
#define __DISPLAYPORT_TYPES_H__

#include "../../platform.h"

typedef enum {
	DP_COMMAND_STATUS	=	0x10,	
	DP_COMMAND_CONFIG	=	0x11	
} DpCommand;

typedef union {
	UINT32 word;
	UINT8 byte[4] __PACKED;
	struct {
		unsigned int    UfpDCapable:1;      
        unsigned int    DfpDCapable:1;      
		unsigned int	SupportsDPv1p3:1;	
		unsigned int 	SupportsUSBGen2:1;	
		unsigned int	Rsvd0:2;
		unsigned int	ReceptacleIndication:1;	
		unsigned int	USB2p0NotUsed:1;		
		unsigned int 	DFP_DPinAssignA:1;		
		unsigned int 	DFP_DPinAssignB:1;
		unsigned int 	DFP_DPinAssignC:1;
		unsigned int 	DFP_DPinAssignD:1;
		unsigned int 	DFP_DPinAssignE:1;
		unsigned int 	DFP_DPinAssignF:1;
		unsigned int 	DFP_DPinAssignRsvd:2;
		unsigned int 	UFP_DPinAssignA:1;		
		unsigned int 	UFP_DPinAssignB:1;
		unsigned int 	UFP_DPinAssignC:1;
		unsigned int 	UFP_DPinAssignD:1;
		unsigned int 	UFP_DPinAssignE:1;
		unsigned int 	UFP_DPinAssignRsvd:3;
		unsigned int	Rsvd1:8;
	};
    struct {
        unsigned int    field0:2;
        unsigned int    field1:2;
        unsigned int    fieldrsvd0:2;
        unsigned int    field2:1;
        unsigned int    field3:1;
        unsigned int    field4:6;
        unsigned int    fieldrsvd1:2;
        unsigned int    field5:5;
        unsigned int    fieldrsvd2:11;
    };
} DisplayPortCaps_t;

typedef enum {
	DP_CONN_NEITHER	= 0,	
	DP_CONN_DFP_D	= 1,	
	DP_CONN_UFP_D	= 2,	
	DP_CONN_BOTH	= 3 	
} DpConn_t;

typedef union {
	UINT32 word;
	UINT8 byte[4] __PACKED;
	struct {
		DpConn_t		Connection:2;				
		unsigned int	PowerLow:1;					
		unsigned int	Enabled:1;					
		unsigned int	MultiFunctionPreferred:1; 	
		unsigned int 	UsbConfigRequest:1;			
		unsigned int 	ExitDpModeRequest:1;		
		unsigned int 	HpdState:1;					
		unsigned int	IrqHpd:1;					
		unsigned int	Rsvd:23;
	};
} DisplayPortStatus_t;

typedef enum {
	DP_CONF_USB		= 0,	
	DP_CONF_DFP_D	= 1,	
	DP_CONF_UFP_D	= 2,	
	DP_CONF_RSVD	= 3
} DpConf_t;

typedef enum {
	DP_CONF_SIG_UNSPECIFIED	= 0,	
	DP_CONF_SIG_DP_V1P3		= 1,	
	DP_CONF_SIG_GEN2		= 2		
} DpConfSig_t;

typedef enum {
	DP_DFPPA_DESELECT	= 0,		
	DP_DFPPA_A			= 1,		
	DP_DFPPA_B			= 2,
	DP_DFPPA_C			= 4,
	DP_DFPPA_D			= 8,
	DP_DFPPA_E			= 16,
	DP_DFPPA_F			= 32
} DpDfpPa_t;

typedef enum {
	DP_UFPPA_DESELECT	= 0,		
	DP_UFPPA_A			= 1,		
	DP_UFPPA_B			= 2,
	DP_UFPPA_C			= 4,
	DP_UFPPA_D			= 8,
	DP_UFPPA_E			= 16
} DpUfpPa_t;

typedef union {
	UINT32 word;
	UINT8 byte[4] __PACKED;
	struct {
		DpConf_t		Conf:2;		
		DpConfSig_t		SigConf:4;	
		unsigned int	Rsvd:2;
		DpDfpPa_t		DfpPa:8;	
		DpUfpPa_t		UfpPa:8;	
	};
} DisplayPortConfig_t;

#endif
