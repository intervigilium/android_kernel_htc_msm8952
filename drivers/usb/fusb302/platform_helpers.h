#ifndef __FUSB_PLATFORM_HELPERS_H_
#define __FUSB_PLATFORM_HELPERS_H_

#define INIT_DELAY_MS   500     
#define RETRIES_I2C 3           

int fusb_InitializeGPIO(void);

void fusb_GPIO_Set_VBus5v(bool set);
void fusb_GPIO_Set_VBusOther(bool set);

bool fusb_GPIO_Get_VBus5v(void);
bool fusb_GPIO_Get_VBusOther(void);
bool fusb_GPIO_Get_IntN(void);

#ifdef DEBUG
void dbg_fusb_GPIO_Set_SM_Toggle(bool set);

bool dbg_fusb_GPIO_Get_SM_Toggle(void);
#endif  

void fusb_GPIO_Cleanup(void);


bool fusb_I2C_WriteData(unsigned char address, unsigned char length, unsigned char* data);

bool fusb_I2C_ReadData(unsigned char address, unsigned char* data);

bool fusb_I2C_ReadBlockData(unsigned char address, unsigned char length, unsigned char* data);


void fusb_InitializeTimer(void);

void fusb_StartTimers(void);

void fusb_StopTimers(void);

void fusb_Delay10us(u32 delay10us);


void fusb_Sysfs_Init(void);

void fusb_InitializeCore(void);

bool fusb_PowerFusb302(void);

bool fusb_IsDeviceValid(void);

void fusb_InitChipData(void);


#ifdef FSC_INTERRUPT_TRIGGERED

int fusb_EnableInterrupts(void);

int fusb_ReStartIrq(void);
#else

void fusb_InitializeWorkers(void);

void fusb_StopThreads(void);

void fusb_ScheduleWork(void);

#endif  

#endif  
