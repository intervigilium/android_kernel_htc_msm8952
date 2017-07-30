#ifndef _FSC_PLATFORM_H_
#define _FSC_PLATFORM_H_

#ifdef PLATFORM_NONE
#include "../Platform_None/GenericTypeDefs.h"
#endif 

#ifdef PLATFORM_PIC32
#include "../Platform_PIC32/GenericTypeDefs.h"
#endif 

#ifdef PLATFORM_ARM
#include "../Platform_ARM/app/GenericTypeDefs.h"
#endif 

#ifdef PLATFORM_LINUX
#include "GenericTypeDefs.h"
#endif 

void platform_set_vbus_5v_enable( BOOL blnEnable );
BOOL platform_get_vbus_5v_enable( void );

void platform_set_vbus_lvl1_enable( BOOL blnEnable );
BOOL platform_get_vbus_lvl1_enable( void );

BOOL platform_get_device_irq_state( void );

BOOL platform_i2c_write(unsigned char SlaveAddress,
                        unsigned char RegAddrLength,
                        unsigned char DataLength,
                        unsigned char PacketSize,
                        unsigned char IncSize,
                        unsigned long RegisterAddress,
                        unsigned char* Data);

BOOL platform_i2c_read( unsigned char SlaveAddress,
                        unsigned char RegAddrLength,
                        unsigned char DataLength,
                        unsigned char PacketSize,
                        unsigned char IncSize,
                        unsigned long RegisterAddress,
                        unsigned char* Data);

void platform_enable_timer(BOOL enable);

void platform_delay_10us( UINT32 delayCount );

#endif  
