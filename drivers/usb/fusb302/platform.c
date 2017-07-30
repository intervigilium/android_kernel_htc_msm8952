#include <linux/kernel.h>                                                       
#include "fusb30x_global.h"                                                     
#include "platform_helpers.h"                                                   
#include "core/platform.h"

void platform_set_vbus_5v_enable(BOOL blnEnable)
{
    fusb_GPIO_Set_VBus5v(blnEnable == TRUE ? true : false);
    return;
}

BOOL platform_get_vbus_5v_enable(void)
{
    return fusb_GPIO_Get_VBus5v() ? TRUE : FALSE;
}

void platform_set_vbus_lvl1_enable(BOOL blnEnable)
{
    fusb_GPIO_Set_VBusOther(blnEnable);
    return;
}

BOOL platform_get_vbus_lvl1_enable(void)
{
    return fusb_GPIO_Get_VBusOther() ? TRUE : FALSE;
}

BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

BOOL platform_i2c_write(unsigned char SlaveAddress,
                        unsigned char RegAddrLength,
                        unsigned char DataLength,
                        unsigned char PacketSize,
                        unsigned char IncSize,
                        unsigned long RegisterAddress,
                        unsigned char* Data)
{
    BOOL ret = FALSE;
    if (Data == NULL)
    {
        printk(KERN_ERR "%s - Error: Write data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb_I2C_WriteData((unsigned char)RegisterAddress, DataLength, Data))
    {
        ret = TRUE;
    }
    else  
    {
        ret = FALSE;       
        printk(KERN_ERR "%s - I2C write failed! RegisterAddress: 0x%02x\n", __func__, (unsigned int)RegisterAddress);
    }
    return ret;
}

BOOL platform_i2c_read(unsigned char SlaveAddress,
                       unsigned char RegAddrLength,
                       unsigned char DataLength,
                       unsigned char PacketSize,
                       unsigned char IncSize,
                       unsigned long RegisterAddress,
                       unsigned char* Data)
{
    BOOL ret = FALSE;
    int i = 0;
    unsigned char temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return FALSE;
    }

    if (Data == NULL)
    {
        printk(KERN_ERR "%s - Error: Read data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (chip->use_i2c_blocks)
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            printk(KERN_ERR "%s - I2C block read failed! RegisterAddress: 0x%02x, Length: %u\n", __func__, (unsigned int)RegisterAddress, (u8)DataLength);
            ret = FALSE;
        }
        else
        {
            ret = TRUE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((UINT8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = TRUE;
            }
            else
            {
                printk(KERN_ERR "%s - I2C read failed! RegisterAddress: 0x%02x\n", __func__, (unsigned int)RegisterAddress);
                ret = FALSE;
                break;
            }
        }
    }

    return ret;
}

void platform_enable_timer(BOOL enable)
{
    if (enable == TRUE)
    {
        fusb_StartTimers();
    }
    else
    {
        fusb_StopTimers();
    }
}

void platform_delay_10us(UINT32 delayCount)
{
    fusb_Delay10us(delayCount);
}
