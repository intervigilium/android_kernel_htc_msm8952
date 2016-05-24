#include <linux/kernel.h>
#include <linux/stat.h>															
#include <linux/types.h>														
#include <linux/i2c.h>															
#include <linux/errno.h>														
#include <linux/hrtimer.h>														
#include <linux/workqueue.h>													
#include <linux/delay.h>														
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include "fusb30x_global.h"														
#include "core/core.h"														 
#include "hostcomm.h"
#include "platform_helpers.h"

const char* FUSB_DT_INTERRUPT_INTN =	"fsc_interrupt_int_n";		
#define FUSB_DT_GPIO_INTN				"fairchild,int_n"			
#define FUSB_DT_GPIO_VBUS_5V			"fairchild,vbus5v"			
#define FUSB_DT_GPIO_VBUS_OTHER			"fairchild,vbusOther"		
#define FUSB_DT_GPIO_POWER		   "fairchild,power-gpio"


#ifdef DEBUG
#define FUSB_DT_GPIO_DEBUG_SM_TOGGLE	"fairchild,dbg_sm"			
#endif	

#ifdef FSC_INTERRUPT_TRIGGERED
static irqreturn_t _fusb_isr_intn(int irq, void *dev_id);
#endif	

bool fusb_PowerFusb302(void)
{
	struct device_node* node;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int fairchild_power = 0;
	int val = -100;
	printk("FUSB  [%s] - Power on FUSB302\n", __func__);

	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return FALSE;
	}

	
	node = chip->client->dev.of_node;
	fairchild_power = of_get_named_gpio(node, FUSB_DT_GPIO_POWER, 0);
	gpio_direction_output( fairchild_power, 1 );

	val = gpio_get_value(fairchild_power);
	if (val == 1)
		return true;
	else
		return false;

}

int fusb_InitializeGPIO(void)
{
	int ret = 0;
	struct device_node* node;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}
	
	node = chip->client->dev.of_node;

	
	chip->gpio_IntN = of_get_named_gpio(node, FUSB_DT_GPIO_INTN, 0);
	if (!gpio_is_valid(chip->gpio_IntN))
	{
		dev_err(&chip->client->dev, "%s - Error: Could not get GPIO for Int_N! Error code: %d\n", __func__, chip->gpio_IntN);
		return chip->gpio_IntN;
	}
	ret = gpio_direction_input(chip->gpio_IntN);
	if (ret != 0)
	{
		dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to input for Int_N! Error code: %d\n", __func__, ret);
		return ret;
	}

	
	gpio_export(chip->gpio_IntN, false);
	gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_INTN, chip->gpio_IntN);


#ifdef VBUS_5V_SUPPORTED
	
	chip->gpio_VBus5V = of_get_named_gpio(node, FUSB_DT_GPIO_VBUS_5V, 0);
	if (!gpio_is_valid(chip->gpio_VBus5V))
	{
		dev_err(&chip->client->dev, "%s - Error: Could not get GPIO for VBus5V! Error code: %d\n", __func__, chip->gpio_VBus5V);
		fusb_GPIO_Cleanup();
		return chip->gpio_VBus5V;
	}
	ret = gpio_direction_output(chip->gpio_VBus5V, chip->gpio_VBus5V_value);
	if (ret != 0)
	{
		dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for VBus5V! Error code: %d\n", __func__, ret);
		fusb_GPIO_Cleanup();
		return ret;
	}

	
	gpio_export(chip->gpio_VBus5V, false);
	gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_VBUS_5V, chip->gpio_VBus5V);

	printk(KERN_DEBUG "FUSB  %s - VBus 5V initialized as pin '%d' and is set to '%d'\n", __func__, chip->gpio_VBus5V, chip->gpio_VBus5V_value ? 1 : 0);
#endif

#ifdef VBUS_OTHER_SUPPORTED
	
	
	chip->gpio_VBusOther = of_get_named_gpio(node, FUSB_DT_GPIO_VBUS_OTHER, 0);
	chip->gpio_VBusOther = -1;

	if (!gpio_is_valid(chip->gpio_VBusOther))
	{
		
		printk(KERN_WARNING "%s - Error: Could not get GPIO for VBusOther! Error code: %d\n", __func__, chip->gpio_VBusOther);
	}
	else
	{
		ret = gpio_direction_output(chip->gpio_VBusOther, chip->gpio_VBusOther_value);
		if (ret != 0)
		{
			dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for VBus5V! Error code: %d\n", __func__, ret);
			return ret;
		}
	}
#endif

#ifdef FUSB_DEBUG
	
	
	chip->dbg_gpio_StateMachine = of_get_named_gpio(node, FUSB_DT_GPIO_DEBUG_SM_TOGGLE, 0);
	if (!gpio_is_valid(chip->dbg_gpio_StateMachine))
	{
		
		printk(KERN_WARNING "%s - Error: Could not get GPIO for SM Debug Toggle! Error code: %d\n", __func__, chip->dbg_gpio_StateMachine);
	}
	else
	{
		ret = gpio_direction_output(chip->dbg_gpio_StateMachine, chip->dbg_gpio_StateMachine_value);
		if (ret != 0)
		{
			dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for SM Debug Toggle! Error code: %d\n", __func__, ret);
			return ret;
		}

		
		gpio_export(chip->dbg_gpio_StateMachine, false);
		gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_DEBUG_SM_TOGGLE, chip->dbg_gpio_StateMachine);
	}
#endif	

	return 0;	
}

void fusb_GPIO_Set_VBus5v(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}
#ifdef VBUS_5V_SUPPORTED
	
	gpio_set_value(chip->gpio_VBus5V, set ? 1 : 0);
	chip->gpio_VBus5V_value = set;

	printk(KERN_DEBUG "FUSB  %s - VBus 5V set to: %d\n", __func__, chip->gpio_VBus5V_value ? 1 : 0);
#endif
	printk(KERN_DEBUG "FUSB  %s - We don't support VBus 5V, just skip\n", __func__);
}

void fusb_GPIO_Set_VBusOther(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}

#ifdef VBUS_OTHER_SUPPORTED
	
	if (gpio_is_valid(chip->gpio_VBusOther))
	{
		gpio_set_value(chip->gpio_VBusOther, set ? 1 : 0);
	}
	chip->gpio_VBusOther_value = set;
#endif
	printk(KERN_DEBUG "FUSB  %s - We don't support VBus other, just skip\n", __func__);
}

bool fusb_GPIO_Get_VBus5v(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}

	printk(KERN_DEBUG "FUSB  %s - We don't support Vbus 5V, return false\n", __func__);
	return false;
#ifdef VBUS_5V_SUPPORTED
	if (!gpio_is_valid(chip->gpio_VBus5V))
	{
		printk(KERN_DEBUG "FUSB  %s - Error: VBus 5V pin invalid! Pin value: %d\n", __func__, chip->gpio_VBus5V);
	}

	return chip->gpio_VBus5V_value;
#endif
}

bool fusb_GPIO_Get_VBusOther(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}

	printk(KERN_DEBUG "FUSB  %s - We don't support Vbus Other, return false\n", __func__);
	return false;
#ifdef VBUS_OTHER_SUPPORTED
	if (!gpio_is_valid(chip->gpio_VBusOther))
	{
		printk(KERN_DEBUG "FUSB  %s - Error: VBus Other pin invalid! Pin value: %d\n", __func__, chip->gpio_VBusOther);
	}

	return chip->gpio_VBusOther_value;
#endif
}

bool fusb_GPIO_Get_IntN(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}
	else
	{
		ret = !gpio_get_value(chip->gpio_IntN); 
		if (ret < 0)
		{
			dev_err(&chip->client->dev, "%s - Error: Could not get GPIO value for gpio_IntN! Error code: %d\n", __func__, ret);
			return false;
		}
		return (ret > 0);
	}
}

#ifdef DEBUG
void dbg_fusb_GPIO_Set_SM_Toggle(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}

	if (gpio_is_valid(chip->dbg_gpio_StateMachine))
	{
		gpio_set_value(chip->dbg_gpio_StateMachine, set ? 1 : 0);
		chip->dbg_gpio_StateMachine_value = set;
		printk(KERN_DEBUG "FUSB  %s - State machine toggle GPIO set to: %d\n", __func__, chip->dbg_gpio_StateMachine_value ? 1 : 0);
	}
	else
	{
		printk(KERN_DEBUG "FUSB  %s - State machine toggle GPIO unavailable!\n", __func__);
	}
}

bool dbg_fusb_GPIO_Get_SM_Toggle(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}
	if (!gpio_is_valid(chip->dbg_gpio_StateMachine))
	{
		printk(KERN_DEBUG "FUSB  %s - Error: State machine toggle debug pin invalid! Pin number: %d\n", __func__, chip->dbg_gpio_StateMachine);
	}
	return chip->dbg_gpio_StateMachine_value;
}
#endif	

void fusb_GPIO_Cleanup(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

#ifdef FSC_INTERRUPT_TRIGGERED
	if (gpio_is_valid(chip->gpio_IntN) && chip->gpio_IntN_irq != -1)	
	{
		devm_free_irq(&chip->client->dev, chip->gpio_IntN_irq, chip);
	}
#endif

	if (gpio_is_valid(chip->gpio_IntN) >= 0)
	{
		gpio_unexport(chip->gpio_IntN);
		gpio_free(chip->gpio_IntN);
	}

#ifdef VBUS_5V_SUPPORTED
	if (gpio_is_valid(chip->gpio_VBus5V) >= 0)
	{
		gpio_unexport(chip->gpio_VBus5V);
		gpio_free(chip->gpio_VBus5V);
	}
#endif
#ifdef VBUS_OTHER_SUPPORTED
	if (gpio_is_valid(chip->gpio_VBusOther) >= 0)
	{
		gpio_free(chip->gpio_VBusOther);
	}
#endif
#ifdef FUSB_DEBUG
	if (gpio_is_valid(chip->dbg_gpio_StateMachine) >= 0)
	{
		gpio_unexport(chip->dbg_gpio_StateMachine);
		gpio_free(chip->dbg_gpio_StateMachine);
	}
#endif	
}

bool fusb_I2C_WriteData(unsigned char address, unsigned char length, unsigned char* data)
{
	int i = 0;
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL || chip->client == NULL || data == NULL)				
	{
		printk(KERN_ERR "%s - Error: %s is NULL!\n", __func__, (chip == NULL ? "Internal chip structure"
			: (chip->client == NULL ? "I2C Client"
			: "Write data buffer")));
		return false;
	}

	mutex_lock(&chip->lock);
	
	for (i = 0; i <= chip->numRetriesI2C; i++)
	{
		ret = i2c_smbus_write_i2c_block_data(chip->client,						
											 address,							
											 length,							
											 data);								
		if (ret < 0)															
		{
			if (ret == ERANGE) { dev_err(&chip->client->dev, "%s - I2C Error writing byte data. Address: '0x%02x', Return: -ERANGE.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
			else if (ret == EINVAL) { dev_err(&chip->client->dev, "%s - I2C Error writing byte data. Address: '0x%02x', Return: -EINVAL.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
			else { dev_err(&chip->client->dev, "%s - Unexpected I2C error writing byte data. Address: '0x%02x', Return: '%04x'.  Attempt #%d / %d...\n", __func__, address, ret, i, chip->numRetriesI2C); }
		}
		else																	
		{
			break;
		}
	}
	mutex_unlock(&chip->lock);

	return (ret >= 0);
}

bool fusb_I2C_ReadData(unsigned char address, unsigned char* data)
{
	int i = 0;
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL || chip->client == NULL || data == NULL)
	{
		printk(KERN_ERR "%s - Error: %s is NULL!\n", __func__, (chip == NULL ? "Internal chip structure"
			: (chip->client == NULL ? "I2C Client"
			: "read data buffer")));
		return false;
	}
	mutex_lock(&chip->lock);
	
	for (i = 0; i <= chip->numRetriesI2C; i++)
	{
		ret = i2c_smbus_read_byte_data(chip->client, (u8)address);		   
		if (ret < 0)															
		{
			if (ret == ERANGE) { dev_err(&chip->client->dev, "%s - I2C Error reading byte data. Address: '0x%02x', Return: -ERANGE.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
			else if (ret == EINVAL) { dev_err(&chip->client->dev, "%s - I2C Error reading byte data. Address: '0x%02x', Return: -EINVAL.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
			else { dev_err(&chip->client->dev, "%s - Unexpected I2C error reading byte data. Address: '0x%02x', Return: '%04x'.  Attempt #%d / %d...\n", __func__, address, ret, i, chip->numRetriesI2C); }
		}
		else																	
		{
			*data = (unsigned char)ret;
			break;
		}
	}
	mutex_unlock(&chip->lock);

	return (ret >= 0);
}

bool fusb_I2C_ReadBlockData(unsigned char address, unsigned char length, unsigned char* data)
{
	int i = 0;
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL || chip->client == NULL || data == NULL)
	{
		printk(KERN_ERR "%s - Error: %s is NULL!\n", __func__, (chip == NULL ? "Internal chip structure"
			: (chip->client == NULL ? "I2C Client"
			: "block read data buffer")));
		return false;
	}

	mutex_lock(&chip->lock);

	ret = i2c_smbus_read_i2c_block_data(chip->client, (u8)address, (u8)length, (u8*)data);			
	if (ret < 0)																					
	{
		if (ret == ERANGE) { dev_err(&chip->client->dev, "%s - I2C Error block reading byte data. Address: '0x%02x', Return: -ERANGE.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
		else if (ret == EINVAL) { dev_err(&chip->client->dev, "%s - I2C Error block reading byte data. Address: '0x%02x', Return: -EINVAL.	Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C); }
		else { dev_err(&chip->client->dev, "%s - Unexpected I2C error block reading byte data. Address: '0x%02x', Return: '%04x'.  Attempt #%d / %d...\n", __func__, address, ret, i, chip->numRetriesI2C); }
	}
	else if (ret != length) 
	{
		printk(KERN_ERR "%s - Error: Block read request of %u bytes truncated to %u bytes.\n", __func__, length, I2C_SMBUS_BLOCK_MAX);
	}

	mutex_unlock(&chip->lock);

	return (ret == length);
}


static const unsigned long g_fusb_timer_tick_period_ns = 100000;	

enum hrtimer_restart _fusb_TimerHandler(struct hrtimer* timer)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return HRTIMER_NORESTART;
	}

	if (!timer)
	{
		printk(KERN_ALERT "FUSB  %s - Error: High-resolution timer is NULL!\n", __func__);
		return HRTIMER_NORESTART;
	}

	core_tick_at_100us();

#ifdef DEBUG
	if (chip->dbgTimerTicks++ >= U8_MAX)
	{
		chip->dbgTimerRollovers++;
	}
#endif	

	
	hrtimer_forward(timer, ktime_get(), ktime_set(0, g_fusb_timer_tick_period_ns));

	return HRTIMER_RESTART;			
}

void fusb_InitializeTimer(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	hrtimer_init(&chip->timer_state_machine, CLOCK_MONOTONIC, HRTIMER_MODE_REL);			
	chip->timer_state_machine.function = _fusb_TimerHandler;								

	printk(KERN_DEBUG "FUSB  %s - Timer initialized!\n", __func__);
}

void fusb_StartTimers(void)
{
	ktime_t ktime;
	struct fusb30x_chip* chip;

	chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	ktime = ktime_set(0, g_fusb_timer_tick_period_ns);										
	hrtimer_start(&chip->timer_state_machine, ktime, HRTIMER_MODE_REL);						
}

void fusb_StopTimers(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	mutex_lock(&chip->lock);
	if (hrtimer_active(&chip->timer_state_machine) != 0)
	{
		ret = hrtimer_cancel(&chip->timer_state_machine);
		printk(KERN_DEBUG "%s - Active state machine hrtimer canceled: %d\n", __func__, ret);
	}
	if (hrtimer_is_queued(&chip->timer_state_machine) != 0)
	{
		ret = hrtimer_cancel(&chip->timer_state_machine);
		printk(KERN_DEBUG "%s - Queued state machine hrtimer canceled: %d\n", __func__, ret);
	}
	mutex_unlock(&chip->lock);
	printk(KERN_DEBUG "FUSB  %s - Timer stopped!\n", __func__);
}

static const unsigned int MAX_DELAY_10US = (UINT_MAX / 10);
void fusb_Delay10us(u32 delay10us)
{
	unsigned int us = 0;
	if (delay10us > MAX_DELAY_10US)
	{
		printk(KERN_ALERT "%s - Error: Delay of '%u' is too long! Must be less than '%u'.\n", __func__, delay10us, MAX_DELAY_10US);
		return;
	}

	us = delay10us * 10;									

	if (us <= 10)											
	{
		udelay(us);											
	}
	else if (us < 20000)									
	{
		
		usleep_range(us, us + (us / 10));					
	}
	else													
	{
		msleep(us / 1000);									
	}
}


void fusb_timestamp_bytes_to_time(unsigned int* outSec, unsigned int* outMS10ths, char* inBuf)
{
	if (outSec && outMS10ths && inBuf)
	{
		*outMS10ths = inBuf[0];
		*outMS10ths = *outMS10ths << 8;
		*outMS10ths |= inBuf[1];

		*outSec = inBuf[2];
		*outSec = *outSec << 8;
		*outSec |= inBuf[3];
	}
}

/*******************************************************************************
* Function:		   fusb_Sysfs_Handle_Read
* Input:		   output: Buffer to which the output will be written
* Return:		   Number of chars written to output
* Description:	   Reading this file will output the most recently saved hostcomm output buffer
********************************************************************************/
#define FUSB_MAX_BUF_SIZE 256	
static ssize_t _fusb_Sysfs_Hostcomm_show(struct device* dev, struct device_attribute* attr, char* buf)
{
	int i = 0;
	int numLogs = 0;
	int numChars = 0;
	unsigned int TimeStampSeconds = 0;	 
	unsigned int TimeStampMS10ths = 0;	 
	char tempBuf[FUSB_MAX_BUF_SIZE] = { 0 };
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
	{
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
	}
	else if (buf == NULL || chip->HostCommBuf == NULL)
	{
		printk(KERN_ERR "%s - Buffer is null!\n", __func__);
	}
	else if (chip->HostCommBuf[0] == CMD_READ_PD_STATE_LOG)  
	{
		numLogs = chip->HostCommBuf[3];
		
		numChars += sprintf(tempBuf, "PD State Log has %u entries:\n", numLogs); 
		strcat(buf, tempBuf);

		
		for (i = 4; (i + 4 < FSC_HOSTCOMM_BUFFER_SIZE) && (numChars < PAGE_SIZE) && (numLogs > 0); i += 5, numLogs--) 
		{
			
			fusb_timestamp_bytes_to_time(&TimeStampSeconds, &TimeStampMS10ths, &chip->HostCommBuf[i + 1]);

			
			switch (chip->HostCommBuf[i])
			{
				case peDisabled:			
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDisabled\t\tPolicy engine is disabled\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peErrorRecovery:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeErrorRecovery\t\tError recovery state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceHardReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceHardReset\t\tReceived a hard reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendHardReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendHardReset\t\tSource send a hard reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSoftReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSoftReset\t\tReceived a soft reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendSoftReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendSoftReset\t\tSend a soft reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceStartup:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceStartup\t\tInitial state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendCaps\t\tSend the source capabilities\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceDiscovery:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceDiscovery\t\tWaiting to detect a USB PD sink\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceDisabled:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceDisabled\t\tDisabled state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceTransitionDefault:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceTransitionDefault\t\tTransition to default 5V state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceNegotiateCap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceNegotiateCap\t\tNegotiate capability and PD contract\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceCapabilityResponse:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceCapabilityResponse\t\tRespond to a request message with a reject/wait\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceTransitionSupply:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceTransitionSupply\t\tTransition the power supply to the new setting (accept request)\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceReady:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceReady\t\tContract is in place and output voltage is stable\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceGiveSourceCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceGiveSourceCaps\t\tState to resend source capabilities\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceGetSinkCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceGetSinkCaps\t\tState to request the sink capabilities\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendPing:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendPing\t\tState to send a ping message\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceGotoMin:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceGotoMin\t\tState to send the gotoMin and ready the power supply\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceGiveSinkCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceGiveSinkCaps\t\tState to send the sink capabilities if dual-role\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceGetSourceCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceGetSourceCaps\t\tState to request the source caps from the UFP\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendDRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendDRSwap\t\tState to send a DR_Swap message\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceEvaluateDRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceEvaluateDRSwap\t\tEvaluate whether we are going to accept or reject the swap\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkHardReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkHardReset\t\tReceived a hard reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSendHardReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSendHardReset\t\tSink send hard reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSoftReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSoftReset\t\tSink soft reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSendSoftReset:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSendSoftReset\t\tSink send soft reset\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkTransitionDefault:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkTransitionDefault\t\tTransition to the default state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkStartup:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkStartup\t\tInitial sink state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkDiscovery:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkDiscovery\t\tSink discovery state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkWaitCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkWaitCaps\t\tSink wait for capabilities state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkEvaluateCaps:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkEvaluateCaps\t\tSink state to evaluate the received source capabilities\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSelectCapability:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSelectCapability\t\tSink state for selecting a capability\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkTransitionSink:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkTransitionSink\t\tSink state for transitioning the current power\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkReady:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkReady\t\tSink ready state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkGiveSinkCap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkGiveSinkCap\t\tSink send capabilities state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkGetSourceCap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkGetSourceCap\t\tSink get source capabilities state\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkGetSinkCap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkGetSinkCap\t\tSink state to get the sink capabilities of the connected source\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkGiveSourceCap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkGiveSourceCap\t\tSink state to send the source capabilities if dual-role\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSendDRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSendDRSwap\t\tState to send a DR_Swap message\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkEvaluateDRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkEvaluateDRSwap\t\tEvaluate whether we are going to accept or reject the swap\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendVCONNSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendVCONNSwap\t\tInitiate a VCONN swap sequence\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkEvaluateVCONNSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkEvaluateVCONNSwap\t\tEvaluate whether we are going to accept or reject the swap\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceSendPRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceSendPRSwap\t\tInitiate a PR_Swap sequence\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceEvaluatePRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceEvaluatePRSwap\t\tEvaluate whether we are going to accept or reject the swap\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkSendPRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkSendPRSwap\t\tInitiate a PR_Swap sequence\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSinkEvaluatePRSwap:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSinkEvaluatePRSwap\t\tEvaluate whether we are going to accept or reject the swap\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peGiveVdm:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeGiveVdm\t\tSend VDM data\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmGetIdentity:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmGetIdentity\t\tRequesting Identity information from DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmSendIdentity:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmSendIdentity\t\tSending Discover Identity ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmGetSvids:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmGetSvids\t\tRequesting SVID info from DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmSendSvids:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmSendSvids\t\tSending Discover SVIDs ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmGetModes:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmGetModes\t\tRequesting Mode info from DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmSendModes:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmSendModes\t\tSending Discover Modes ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmEvaluateModeEntry:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmEvaluateModeEntry\t\tRequesting DPM to evaluate request to enter a mode, and enter if OK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmModeEntryNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmModeEntryNak\t\tSending Enter Mode NAK response\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmModeEntryAck:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmModeEntryAck\t\tSending Enter Mode ACK response\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmModeExit:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmModeExit\t\tRequesting DPM to evalute request to exit mode\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmModeExitNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmModeExitNak\t\tSending Exit Mode NAK reponse\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmModeExitAck:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmModeExitAck\t\tSending Exit Mode ACK Response\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peUfpVdmAttentionRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeUfpVdmAttentionRequest\t\tSending Attention Command\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpUfpVdmIdentityRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpUfpVdmIdentityRequest\t\tSending Identity Request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpUfpVdmIdentityAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpUfpVdmIdentityAcked\t\tInform DPM of Identity\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpUfpVdmIdentityNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpUfpVdmIdentityNaked\t\tInform DPM of result\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpCblVdmIdentityRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpCblVdmIdentityRequest\t\tSending Identity Request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpCblVdmIdentityAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpCblVdmIdentityAcked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpCblVdmIdentityNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpCblVdmIdentityNaked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmSvidsRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmSvidsRequest\t\tSending Discover SVIDs request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmSvidsAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmSvidsAcked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmSvidsNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmSvidsNaked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModesRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModesRequest\t\tSending Discover Modes request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModesAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModesAcked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModesNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModesNaked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModeEntryRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModeEntryRequest\t\tSending Mode Entry request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModeEntryAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModeEntryAcked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModeEntryNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModeEntryNaked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmModeExitRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmModeExitRequest\t\tSending Exit Mode request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmExitModeAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmExitModeAcked\t\tInform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSrcVdmIdentityRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSrcVdmIdentityRequest\t\tsending Discover Identity request\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSrcVdmIdentityAcked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSrcVdmIdentityAcked\t\tinform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSrcVdmIdentityNaked:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSrcVdmIdentityNaked\t\tinform DPM\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDfpVdmAttentionRequest:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDfpVdmAttentionRequest\t\tAttention Request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblReady:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblReady\t\tCable power up state?\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetIdentity:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetIdentity\t\tDiscover Identity request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetIdentityNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetIdentityNak\t\tRespond with NAK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblSendIdentity:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblSendIdentity\t\tRespond with Ack\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetSvids:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetSvids\t\tDiscover SVIDs request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetSvidsNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetSvidsNak\t\tRespond with NAK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblSendSvids:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblSendSvids\t\tRespond with ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetModes:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetModes\t\tDiscover Modes request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblGetModesNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblGetModesNak\t\tRespond with NAK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblSendModes:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblSendModes\t\tRespond with ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblEvaluateModeEntry:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblEvaluateModeEntry\t\tEnter Mode request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblModeEntryAck:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblModeEntryAck\t\tRespond with NAK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblModeEntryNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblModeEntryNak\t\tRespond with ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblModeExit:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblModeExit\t\tExit Mode request received\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblModeExitAck:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblModeExitAck\t\tRespond with NAK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peCblModeExitNak:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeCblModeExitNak\t\tRespond with ACK\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peDpRequestStatus:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeDpRequestStatus\t\tRequesting PP Status\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case PE_BIST_Receive_Mode:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tPE_BIST_Receive_Mode\t\tBist Receive Mode\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case PE_BIST_Frame_Received:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tPE_BIST_Frame_Received\t\tTest Frame received by Protocol layer\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case PE_BIST_Carrier_Mode_2:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tPE_BIST_Carrier_Mode_2\t\tBIST Carrier Mode 2\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case peSourceWaitNewCapabilities:		
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tpeSourceWaitNewCapabilities\t\tWait for new Source Capabilities from Policy Manager\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case dbgGetRxPacket:				
				{
					
					numChars += sprintf(tempBuf, "%02u|0.%04u\tdbgGetRxPacket\t\t\tNumber of I2C bytes read | Time elapsed\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				default:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tUKNOWN STATE: 0x%02x\n", TimeStampSeconds, TimeStampMS10ths, chip->HostCommBuf[i]);
					strcat(buf, tempBuf);
					break;
				}
			}
		}
		strcat(buf, "\n");	 
		numChars++;			 
	}
	else if (chip->HostCommBuf[0] == CMD_READ_STATE_LOG)  
	{
		numLogs = chip->HostCommBuf[3];
		
		numChars += sprintf(tempBuf, "Type-C State Log has %u entries:\n", numLogs); 
		strcat(buf, tempBuf);

		
		for (i = 4; (i + 4 < FSC_HOSTCOMM_BUFFER_SIZE) && (numChars < PAGE_SIZE) && (numLogs > 0); i += 5, numLogs--) 
		{
			
			fusb_timestamp_bytes_to_time(&TimeStampSeconds, &TimeStampMS10ths, &chip->HostCommBuf[i + 1]);

			
			switch (chip->HostCommBuf[i])
			{
				case Disabled:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tDisabled\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case ErrorRecovery:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tErrorRecovery\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case Unattached:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tUnattached\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AttachWaitSink:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAttachWaitSink\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AttachedSink:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAttachedSink\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AttachWaitSource:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAttachWaitSource\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AttachedSource:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAttachedSource\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case TrySource:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tTrySource\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case TryWaitSink:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tTryWaitSink\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case TrySink:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tTrySink\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case TryWaitSource:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tTryWaitSource\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AudioAccessory:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAudioAccessory\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case DebugAccessory:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tDebugAccessory\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case AttachWaitAccessory:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tAttachWaitAccessory\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case PoweredAccessory:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tPoweredAccessory\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case UnsupportedAccessory:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tUnsupportedAccessory\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case DelayUnattached:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tDelayUnattached\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}

				case UnattachedSource:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tUnattachedSource\n", TimeStampSeconds, TimeStampMS10ths);
					strcat(buf, tempBuf);
					break;
				}
				default:
				{
					numChars += sprintf(tempBuf, "[%u.%04u]\tUKNOWN STATE: 0x%02x\n", TimeStampSeconds, TimeStampMS10ths, chip->HostCommBuf[i]);
					strcat(buf, tempBuf);
					break;
				}
			}
		}
		strcat(buf, "\n");	 
		numChars++;			 
	}
	else
	{
		for (i = 0; i < FSC_HOSTCOMM_BUFFER_SIZE; i++)
		{
			numChars += scnprintf(tempBuf, 6 * sizeof(char), "0x%02x ", chip->HostCommBuf[i]); 
			strcat(buf, tempBuf);	
		}
		strcat(buf, "\n");	 
		numChars++;			 
	}
	return numChars;
}

/*******************************************************************************
* Function:		   fusb_Sysfs_Handle_Write
* Input:		   input: Buffer passed in from OS (space-separated list of 8-bit hex values)
*				   size: Number of chars in input
*				   output: Buffer to which the output will be written
* Return:		   Number of chars written to output
* Description:	   Performs hostcomm duties, and stores output buffer in chip structure
********************************************************************************/
static ssize_t _fusb_Sysfs_Hostcomm_store(struct device* dev, struct device_attribute* attr, const char* input, size_t size)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	char tempByte = 0;
	int numBytes = 0;
	char temp[6] = { 0 };	
	char temp_input[FSC_HOSTCOMM_BUFFER_SIZE] = { 0 };
	char output[FSC_HOSTCOMM_BUFFER_SIZE] = { 0 };
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
	{
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
	}
	else if (input == NULL)
	{
		printk(KERN_ERR "%s - Error: Input buffer is NULL!\n", __func__);
	}
	else
	{
		
		for (i = 0; i < size; i = i + j)
		{
			
			for (j = 0; (j < 5) && (j + i < size); j++)
			{
				
				if (input[i + j] == ' ')
				{
					break;					
				}

				temp[j] = input[i + j];		
			}

			temp[++j] = 0;					

			
			ret = kstrtou8(temp, 16, &tempByte);
			if (ret != 0)
			{
				printk(KERN_ERR "FUSB  %s - Error: Hostcomm input is not a valid hex value! Return: '%d'\n", __func__, ret);
				return 0;  
			}
			else
			{
				temp_input[numBytes++] = tempByte;
				if (numBytes >= FSC_HOSTCOMM_BUFFER_SIZE)
				{
					break;
				}
			}
		}

		fusb_ProcessMsg(temp_input, output);												
		memcpy(chip->HostCommBuf, output, FSC_HOSTCOMM_BUFFER_SIZE);						
	}

	return size;
}

static DEVICE_ATTR(fusb30x_hostcomm, S_IRWXU | S_IRWXG | S_IROTH, _fusb_Sysfs_Hostcomm_show, _fusb_Sysfs_Hostcomm_store);

void fusb_Sysfs_Init(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
	{
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
		return;
	}
	ret = device_create_file(&chip->client->dev, &dev_attr_fusb30x_hostcomm);
	if (ret != 0)
	{
		dev_err(&chip->client->dev, "FUSB  %s - Error: Unable to initialize sysfs device file 'fusb30x_io'! ret = %d\n", __func__, ret);
		return;
	}
}

void fusb_InitializeCore(void)
{
	core_initialize();
	printk(KERN_DEBUG "FUSB  %s - Core is initialized!\n", __func__);
	fusb_StartTimers();
	printk(KERN_DEBUG "FUSB  %s - Timers are started!\n", __func__);
	core_enable_typec(TRUE);
	printk(KERN_DEBUG "FUSB  %s - Type-C State Machine is enabled!\n", __func__);
}

bool fusb_IsDeviceValid(void)
{
	unsigned char val = 0;
	struct device_node* node;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return FALSE;
	}

	
		node = chip->client->dev.of_node;

	
	if (!fusb_I2C_ReadData((unsigned char)0x01, &val))
	{
		printk(KERN_ALERT "FUSB  %s - Error: Could not communicate with device over I2C!\n", __func__);
		return FALSE;
	}
	printk("FUSB [%s] - device id = %x \n", __func__, val);

	return TRUE;
}

void fusb_InitChipData(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL)
	{
		printk(KERN_ALERT "%s - Chip structure is null!\n", __func__);
		return;
	}

#ifdef DEBUG
	chip->dbgTimerTicks = 0;
	chip->dbgTimerRollovers = 0;
	chip->dbgSMTicks = 0;
	chip->dbgSMRollovers = 0;
	chip->dbg_gpio_StateMachine = false;
#endif	

	
	chip->gpio_VBus5V = -1;
	chip->gpio_VBus5V_value = false;
	chip->gpio_VBusOther = -1;
	chip->gpio_VBusOther_value = false;
	chip->gpio_IntN = -1;

#ifdef FSC_INTERRUPT_TRIGGERED
	chip->gpio_IntN_irq = -1;
#endif

	
	chip->InitDelayMS = INIT_DELAY_MS;												
	chip->numRetriesI2C = RETRIES_I2C;												
	chip->use_i2c_blocks = false;													
}


#ifdef FSC_INTERRUPT_TRIGGERED

int fusb_EnableInterrupts(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	
	ret = gpio_to_irq(chip->gpio_IntN); 
	if (ret < 0)
	{
		dev_err(&chip->client->dev, "FUSB %s - Error: Unable to request IRQ for INT_N GPIO! Error code: %d\n", __func__, ret);
		chip->gpio_IntN_irq = -1;	
		fusb_GPIO_Cleanup();
		return ret;
	}
	chip->gpio_IntN_irq = ret;
	printk(KERN_DEBUG "FUSB %s - Success: Requested INT_N IRQ: '%d'\n", __func__, chip->gpio_IntN_irq);

	
	ret = devm_request_threaded_irq(&chip->client->dev, chip->gpio_IntN_irq, NULL, _fusb_isr_intn, IRQF_ONESHOT | IRQF_TRIGGER_LOW, FUSB_DT_INTERRUPT_INTN, chip);	
	if (ret)
	{
		dev_err(&chip->client->dev, "%s - Error: Unable to request threaded IRQ for INT_N GPIO! Error code: %d\n", __func__, ret);
		fusb_GPIO_Cleanup();
		return ret;
	}

	return 0;
}

int fusb_ReStartIrq(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}

	if (gpio_is_valid(chip->gpio_IntN) && chip->gpio_IntN_irq != -1)
	{
		printk("FUSB  [%s] -(error handle) Re-start chip->gpio_IntN_irq after 5 seconds to prevent too many interrupts while non-standard cable plugged\n", __func__);
		disable_irq_nosync(chip->gpio_IntN_irq);
		schedule_delayed_work(&chip->debounce_work, 5 * HZ);
	}

	return 0;
}


static irqreturn_t _fusb_isr_intn(int irq, void *dev_id)
{
	struct fusb30x_chip* chip = dev_id;
	printk("FUSB  [%s]: FUSB-interrupt triggered ++ \n", __func__);

	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return IRQ_NONE;
	}

#ifdef DEBUG
	dbg_fusb_GPIO_Set_SM_Toggle(!chip->dbg_gpio_StateMachine_value);	

	if (chip->dbgSMTicks++ >= U8_MAX)									
	{
		chip->dbgSMRollovers++;											
	}
#endif	

	core_state_machine();												

	return IRQ_HANDLED;
}

#else

void _fusb_InitWorker(struct work_struct* delayed_work)
{
	struct fusb30x_chip* chip = container_of(delayed_work, struct fusb30x_chip, init_worker.work);
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	
	schedule_work(&chip->worker);
}

void _fusb_MainWorker(struct work_struct* work)
{
	struct fusb30x_chip* chip = container_of(work, struct fusb30x_chip, worker);
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

#ifdef DEBUG
	dbg_fusb_GPIO_Set_SM_Toggle(!chip->dbg_gpio_StateMachine_value);	

	if (chip->dbgSMTicks++ >= U8_MAX)									
	{
		chip->dbgSMRollovers++;											
	}
#endif	

	core_state_machine();												
	schedule_work(&chip->worker);										
}

void fusb_InitializeWorkers(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	printk(KERN_DEBUG "FUSB  %s - Initializing threads!\n", __func__);
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	
	INIT_DELAYED_WORK(&chip->init_worker, _fusb_InitWorker);
	INIT_WORK(&chip->worker, _fusb_MainWorker);
}

void fusb_StopThreads(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	
	cancel_delayed_work_sync(&chip->init_worker);
	flush_delayed_work(&chip->init_worker);

	
	flush_work(&chip->worker);
	cancel_work_sync(&chip->worker);
}

void fusb_ScheduleWork(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	schedule_delayed_work(&chip->init_worker, msecs_to_jiffies(chip->InitDelayMS));
}

#endif 

