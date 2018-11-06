//
//  lab1a_driver.c
//  
//
//  Created by Ifunanya Nnoka on 3/5/17.
//
//

#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
//#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-k.h>

#define DEVICE_NAME "hw2"

static struct file_operations dev_fops = {
    .owner			= THIS_MODULE,
  //  .unlocked_ioctl	= sbc2440_con6_ioctl,
};

static struct miscdevice misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &dev_fops,
};

static int __init dev_init(void)
{
    int ret;
    
    {
        unsigned tmp;
        /*configure GPE0(GPECON bits 0->3) to input (0000)*/
        /*GPE0 is also LCD power pin on my board mini6410*/
         /*configure GPE1(GPECON bits 4->7) to output (0001)*/
        tmp = readl(S3C64XX_GPECON);
        tmp = (tmp & ~(0xffU))|(0x10U);
        writel(tmp, S3C64XX_GPECON);
        //
        /*Initialize*/
        tmp = readl(S3C64XX_GPEDAT);
        tmp |= (0x1); //initialize
        writel(tmp, S3C64XX_GPEDAT);
        printk (DEVICE_NAME"\tinitialized\n");

        /*GPECON0 state test*/
        tmp = readl(S3C64XX_GPECON);
        tmp &= 0x0FU;
        if (!(tmp & (0xF))) 
            printk ("\nGPE0:\n\nconfig state = input\npinout = LCD3.30\n\n\n");
        else
            printk ("\nGPE0: not configured to input\n");

        /*GPECON1 state test*/
        tmp = readl(S3C64XX_GPECON);
        tmp &= 0xF0U;
        if ((tmp & 1<< (4))){
            if (~tmp & (1<< (5)))
                if (~tmp & (1<< (6)))
                    if (~tmp & (1<< (7)))
                        printk ("\nGPE1:\n\nconfig state = output\npinout = con6.3\n\n\n");
        }
        else
            printk ("\nGPE1: not configured to output\n");
    
    }
    
    ret = misc_register(&misc);
    
    return ret;
}

static void __exit dev_exit(void)
{
    misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FriendlyARM Inc.");
