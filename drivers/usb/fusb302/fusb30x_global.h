
#ifndef FUSB30X_TYPES_H
#define FUSB30X_TYPES_H

#include <linux/i2c.h>                              
#include <linux/hrtimer.h>                          
#include "GenericTypeDefs.h"                        

struct fusb30x_chip                                 
{
    struct mutex lock;                              

#ifdef DEBUG
    u8 dbgTimerTicks;                               
    u8 dbgTimerRollovers;                           
    u8 dbgSMTicks;                                  
    u8 dbgSMRollovers;                              
    int dbg_gpio_StateMachine;                      
    int dbg_gpio_StateMachine_value;                
#endif

    
    INT InitDelayMS;                                
    INT numRetriesI2C;                              

    
    char HostCommBuf[FSC_HOSTCOMM_BUFFER_SIZE];     

    
    struct i2c_client* client;                      
    bool use_i2c_blocks;                            

    
    int gpio_VBus5V;                                
    bool gpio_VBus5V_value;                         
    int gpio_VBusOther;                             
    bool gpio_VBusOther_value;                      
    int gpio_IntN;                                  

#ifdef FSC_INTERRUPT_TRIGGERED
    int gpio_IntN_irq;                              
#endif  

    
    struct delayed_work init_worker;                
    struct work_struct worker;                      
    struct delayed_work debounce_work;

    
    struct hrtimer timer_state_machine;             
};

extern struct fusb30x_chip* fg_chip;

struct fusb30x_chip* fusb30x_GetChip(void);         
void fusb30x_SetChip(struct fusb30x_chip* newChip); 

#endif 
