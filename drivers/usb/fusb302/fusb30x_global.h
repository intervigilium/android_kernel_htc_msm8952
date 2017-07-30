/*
* File:   fusb30x_global.h
* Author: Tim Bremm <tim.bremm@fairchildsemi.com>
* Company: Fairchild Semiconductor
*
* Created on September 11, 2015, 15:28 AM
*/

#ifndef FUSB30X_TYPES_H
#define FUSB30X_TYPES_H

#include <linux/i2c.h>                              // i2c_client, spinlock_t
#include <linux/hrtimer.h>                          // hrtimer
#include <linux/wakelock.h>/*++ 2016/5/26, USB Team, PCN00027 ++*/
#include "FSCTypes.h"                               // FUSB30x custom types
#include <linux/usb/class-dual-role.h>/*++ 2016/1/18, USB Team, PCN00013 ++*/

#ifdef FSC_DEBUG
#define FSC_HOSTCOMM_BUFFER_SIZE    64              // Length of the hostcomm buffer
#endif // FSC_DEBUG

/*++ 2016/1/18, USB Team, PCN00013 ++*/
enum port_mode {
	PMODE_UFP = 0,
	PMODE_DFP,
	PMODE_UNKNOWN,
};

enum power_role {
	PR_SOURCE = 0,
	PR_SINK,
	UNKNOWN_POWER_ROLE,
};

enum data_role {
	DR_HOST = 0,
	DR_DEVICE,
	UNKNOWN_DATA_ROLE,
};

enum vconn_supply {
	VCONN_SUPPLY_NO = 0,
	VCONN_SUPPLY_YES,
};
/*-- 2016/1/18, USB Team, PCN00013 --*/

struct fusb30x_chip                                 // Contains data required by this driver
{
    struct mutex lock;                              // Synchronization lock

#ifdef FSC_DEBUG
    FSC_U8 dbgTimerTicks;                           // Count of timer ticks
    FSC_U8 dbgTimerRollovers;                       // Timer tick counter rollover counter
    FSC_U8 dbgSMTicks;                              // Count of state machine ticks
    FSC_U8 dbgSMRollovers;                          // State machine tick counter rollover counter
    FSC_S32 dbg_gpio_StateMachine;                  // Gpio that toggles every time the state machine is triggered
    FSC_BOOL dbg_gpio_StateMachine_value;           // Value of sm toggle state machine
    char HostCommBuf[FSC_HOSTCOMM_BUFFER_SIZE];     // Buffer used to communicate with HostComm
#endif // FSC_DEBUG

    /* Internal config data */
    FSC_S32 InitDelayMS;                            // Number of milliseconds to wait before initializing the fusb30x
    FSC_S32 numRetriesI2C;                          // Number of times to retry I2C reads/writes

    /* I2C */
    struct i2c_client* client;                      // I2C client provided by kernel
    FSC_BOOL use_i2c_blocks;                        // True if I2C_FUNC_SMBUS_I2C_BLOCK is supported

    struct pinctrl *fusb_pinctrl;                   // Pinctrl for fusb302
    struct pinctrl_state *fusb_default_state;       // Pinctrl default state for fusb302

    /* GPIO */
    FSC_S32 gpio_VBus5V;                            // VBus 5V GPIO pin
    FSC_BOOL gpio_VBus5V_value;                     // true if active, false otherwise
    FSC_S32 gpio_VBusOther;                         // VBus other GPIO pin (eg. VBus 12V) (NOTE: Optional feature - if set to <0 during GPIO init, then feature is disabled)
    FSC_BOOL gpio_VBusOther_value;                  // true if active, false otherwise
    FSC_S32 gpio_IntN;                              // INT_N GPIO pin

	int gpio_Vconn;                                  // VCONN GPIO pin /*++ 2015/12/30, USB Team, PCN00006 ++*/

#ifdef FSC_INTERRUPT_TRIGGERED
    FSC_S32 gpio_IntN_irq;                          // IRQ assigned to INT_N GPIO pin
#endif  // FSC_INTERRUPT_TRIGGERED

    /* Threads */
    struct delayed_work init_worker;                // Kicks off our runtime worker
    struct work_struct worker;                      // Main state machine actions
    struct delayed_work debounce_work;/*++ 2015/11/16, USB Team, PCN00001 ++*/
	struct delayed_work delayHost_work;/*++ 2016/3/15, USB Team, PCN00022 ++*/
/*++ 2016/1/18, USB Team, PCN00013 ++*/
	struct dual_role_phy_instance *fusb_instance;
	enum port_mode pmode;
	enum power_role prole;
	enum data_role drole;
	enum vconn_supply vconn;
/*-- 2016/1/18, USB Team, PCN00013 --*/

    /* Timers */
    struct hrtimer timer_state_machine;             // High-resolution timer for the state machine
/*++ 2016/5/26, USB Team, PCN00027 ++*/
	atomic_t pm_suspended;
	struct wake_lock fusb_wlock;
/*-- 2016/5/26, USB Team, PCN00027 --*/
};

extern struct fusb30x_chip* fg_chip;/*++ 2015/11/16, USB Team, PCN00001 ++*/

struct fusb30x_chip* fusb30x_GetChip(void);         // Getter for the global chip structure
void fusb30x_SetChip(struct fusb30x_chip* newChip); // Setter for the global chip structure

#endif /* FUSB30X_TYPES_H */
