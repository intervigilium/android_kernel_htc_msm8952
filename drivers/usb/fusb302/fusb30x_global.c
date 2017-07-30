#include "fusb30x_global.h"

struct fusb30x_chip* fg_chip = NULL;  // Our driver's relevant data

struct fusb30x_chip* fusb30x_GetChip(void)
{
    return fg_chip;      // return a pointer to our structs
}

void fusb30x_SetChip(struct fusb30x_chip* newChip)
{
    fg_chip = newChip;   // assign the pointer to our struct
    fg_chip->client->addr = fg_chip->client->addr >> 1;/*++ 2015/11/16, USB Team, PCN00001 ++*/
	printk("FUSB [%s] chip->client addr = 0x%x\n", __func__, fg_chip->client->addr);
}