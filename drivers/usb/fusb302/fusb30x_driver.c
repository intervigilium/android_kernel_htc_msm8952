
#include <linux/init.h>															
#include <linux/module.h>														
#include <linux/kernel.h>														
#include <linux/i2c.h>															
#include <linux/slab.h>															
#include <linux/types.h>														
#include <linux/errno.h>														
#include <linux/of_device.h>													
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#include "fusb30x_global.h"														
#include "platform_helpers.h"													
#include "core/core.h"														 
#include "fusb30x_driver.h"


UINT8 regOffset = 0;
static ssize_t dump_register_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	UINT8 temp = 0;

	switch (regOffset)
	{
		
		case regControl0:
			temp = 0xff;
			break;
		
		case regControl1:
			temp = 0xff;
			break;
		case regInterrupta:
			temp = Registers.Status2.byte[2];
			break;
		case regInterruptb:
			temp = Registers.Status2.byte[3];
			break;
		case regInterrupt:
			temp = Registers.Status2.byte[6];
			break;
		default:
			DeviceRead(regOffset, 1, &temp);
			break;
	}

	printk("FUSB  %s : Read dump_register offset:0x%.2x(%d), data:0x%.2x(%d)\n", __func__, regOffset, regOffset, temp, temp);

	return snprintf(buf, PAGE_SIZE, "0x%.2x (%d)\n", temp, temp);
}

static ssize_t dump_register_store(struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	unsigned int store_val = 0;
	sscanf(buff, "%d", &store_val);

	regOffset = store_val;
	printk("FUSB  %s : Store dump_register: 0x%2x (%d)\n", __func__, regOffset, regOffset);

	return size;
}



int fusb_debug_level = 0;
static ssize_t fusb_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", fusb_debug_level);
	printk("FUSB  %s : Show fusb_debug_level: %d\n", __func__, fusb_debug_level);
	return length;
}

static ssize_t fusb_debug_store(struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	sscanf(buff, "%d", &fusb_debug_level);

	printk("FUSB  %s : Store fusb_debug_level: %d\n", __func__, fusb_debug_level);

	return size;
}

static ssize_t PD_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", USBPDEnabled);
	printk("FUSB  %s : Show PD_on(USBPDEnabled): %d\n", __func__, USBPDEnabled);
	return length;
}

static ssize_t PD_on_store(struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	int PD_on = 0;
	sscanf(buff, "%d", &PD_on);

	printk("FUSB  %s : Store PD_on(USBPDEnabled): %d, and trigger %s PD detection\n", __func__, PD_on, ((PD_on == 1) ? "Enable" : "Disable"));
	if (PD_on == 1) {
		core_enable_pd(true);
	} else if (PD_on == 0) {
		core_enable_pd(false);
	}

	return size;
}

static DEVICE_ATTR(dump_register, S_IWUSR|S_IRUGO, dump_register_show, dump_register_store);
static DEVICE_ATTR(fusb_debug_level, S_IWUSR|S_IRUGO, fusb_debug_show, fusb_debug_store);
static DEVICE_ATTR(PD_on, S_IWUSR|S_IRUGO, PD_on_show, PD_on_store);

static struct device_attribute *fusb_attributes[] = {
	&dev_attr_dump_register,
	&dev_attr_fusb_debug_level,
	&dev_attr_PD_on,
	NULL
};

void fusb_debounce_work_func(struct work_struct* delayed_work)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	enable_irq(chip->gpio_IntN_irq);
	printk("FUSB  [%s] - Enable chip->gpio_IntN_irq IRQ\n", __func__);
}

static int __init fusb30x_init(void)
{
	printk(KERN_DEBUG "FUSB  %s - Start driver initialization...\n", __func__);
	return i2c_add_driver(&fusb30x_driver);
}

static void __exit fusb30x_exit(void)
{
	i2c_del_driver(&fusb30x_driver);
	printk(KERN_DEBUG "FUSB  %s - Driver deleted...\n", __func__);
}

static int fusb30x_probe (struct i2c_client* client,
						  const struct i2c_device_id* id)
{
	int ret = 0;
	struct fusb30x_chip* chip;
	struct i2c_adapter* adapter;
	struct device_attribute **attrs = fusb_attributes;
	struct device_attribute *attr;

	if (!client)
	{
		printk(KERN_ALERT "FUSB  %s - Error: Client structure is NULL!\n", __func__);
		return -EINVAL;
	}
	dev_info(&client->dev, "%s\n", __func__);

	
	if (!of_match_device(fusb30x_dt_match, &client->dev))
	{
		dev_err(&client->dev, "FUSB  %s - Error: Device tree mismatch!\n", __func__);
		return -EINVAL;
	}
	printk(KERN_DEBUG "FUSB  %s - Device tree matched!\n", __func__);

	
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to allocate memory for fg_chip!\n", __func__);
		return -ENOMEM;
	}
	chip->client = client;														
	fusb30x_SetChip(chip);														
	printk(KERN_DEBUG "FUSB  %s - Chip structure is set! Chip: %p ... fg_chip: %p\n", __func__, chip, fusb30x_GetChip());

	
	mutex_init(&chip->lock);

	
	fusb_InitChipData();
	printk(KERN_DEBUG "FUSB  %s - Chip struct data initialized!\n", __func__);

	
	adapter = to_i2c_adapter(client->dev.parent);
	if (i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_BLOCK_REQUIRED_FUNC))
	{
		chip->use_i2c_blocks = true;
	}
	else
	{
		
		
		dev_warn(&client->dev, "FUSB  %s - Warning: I2C/SMBus block read/write functionality not supported, checking single-read mode...\n", __func__);
		if (!i2c_check_functionality(adapter, FUSB30X_I2C_SMBUS_REQUIRED_FUNC))
		{
			dev_err(&client->dev, "FUSB  %s - Error: Required I2C/SMBus functionality not supported!\n", __func__);
			dev_err(&client->dev, "FUSB  %s - I2C Supported Functionality Mask: 0x%x\n", __func__, i2c_get_functionality(adapter));
			return -EIO;
		}
	}
	printk(KERN_DEBUG "FUSB  %s - I2C Functionality check passed! Block reads: %s\n", __func__, chip->use_i2c_blocks ? "YES" : "NO");

	
	i2c_set_clientdata(client, chip);
	printk(KERN_DEBUG "FUSB  %s - I2C client data set!\n", __func__);

	if (!fusb_PowerFusb302())
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to power on FUSB302!\n", __func__);
	}

	
	if (!fusb_IsDeviceValid())
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to communicate with device!\n", __func__);
		return -EIO;
	}
	printk(KERN_DEBUG "FUSB  %s - Device check passed!\n", __func__);


	
	ret = fusb_InitializeGPIO();
	if (ret)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to initialize GPIO!\n", __func__);
		return ret;
	}
	printk(KERN_DEBUG "FUSB  %s - GPIO initialized!\n", __func__);


	
	fusb_InitializeTimer();
	printk(KERN_DEBUG "FUSB  %s - Timers initialized!\n", __func__);

	
	fusb_InitializeCore();
	printk(KERN_DEBUG "FUSB  %s - Core is initialized!\n", __func__);

	
	fusb_Sysfs_Init();
	printk(KERN_DEBUG "FUSB  %s - Sysfs device file created!\n", __func__);

#ifdef FSC_INTERRUPT_TRIGGERED
	
	ret = fusb_EnableInterrupts();
	if (ret)
	{
		dev_err(&client->dev, "FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
		return -EIO;
	}
	
	INIT_DELAYED_WORK(&chip->debounce_work, fusb_debounce_work_func);
#else
	
	fusb_InitializeWorkers();
	
	fusb_ScheduleWork();
	printk(KERN_DEBUG "FUSB  %s - Workers initialized and scheduled!\n", __func__);
#endif	


	while ((attr = *attrs++))
		ret = device_create_file(&client->dev, attr);

	dev_info(&client->dev, "FUSB  %s - FUSB30X Driver loaded successfully!\n", __func__);
	return ret;
}

static int fusb30x_remove(struct i2c_client* client)
{
	printk(KERN_DEBUG "FUSB  %s - Removing fusb30x device!\n", __func__);

#ifndef FSC_INTERRUPT_TRIGGERED 
	fusb_StopThreads();
#endif	

	fusb_StopTimers();
	fusb_GPIO_Cleanup();
	printk(KERN_DEBUG "FUSB  %s - FUSB30x device removed from driver...\n", __func__);
	return 0;
}


module_init(fusb30x_init);														
module_exit(fusb30x_exit);														

MODULE_LICENSE("GPL");															
MODULE_DESCRIPTION("Fairchild FUSB30x Driver");									
MODULE_AUTHOR("Tim Bremm<tim.bremm@fairchildsemi.com>");						
