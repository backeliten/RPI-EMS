/*
 * Simple RTDM demo that generates a running light on your PC keyboard
 * or on a BF537-STAMP board.
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with rtaisec-reloaded; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#include <linux/module.h>
#include "rtdm/rtdm_driver.h"

MODULE_LICENSE("GPL");

///*To setup memory access*/
//#define PAGE_SIZE (4*1024)
//#define BLOCK_SIZE (4*1024)

/*Memory base adress to reach Perpherial*/
#define BCM2708_PERI_BASE        0x20000000

#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) 	/* GPIO controller */
#define INTR_BASE				 (BCM2708_PERI_BASE + 0x00B000)		//0x7E00B000

//#define GPIO_IRQ_NUMBER 49  	//GPIO controller
#define GPIO_IRQ_NUMBER 52  	//GPIO controller
//#define GPIO_IRQ_NUMBER 89  	//GPIO controller

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_READ(g)    *(gpio + 13) &= (1<<(g))

//#define GPIO_SET_P_I(g)    *(gpio + 13) &= (1<<(g))		//Set p

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

static void __iomem *gpio = NULL;

#define HEARTBEAT_PERIOD	100000000 /* 100 ms */
#define HEARTBEAT_PERIOD2	50000000 /* 5 ms */

static rtdm_task_t 			heartbeat_task;
static rtdm_task_t 			heartbeat_task2;
static rtdm_irq_t 			intr_rtdm_desc;

static RT_INTR				intr_desc;

static int end = 0;

char *gpio_mem, *gpio_map;

struct resource * mem;

static void leds_set(int state)
{
	static int ledstate = 0;
	if(ledstate)
	{
		//Set led on
		iowrite32(1<<25, gpio+(7*4));
		iowrite32(1<<23, gpio+(10*4));
		//rtdm_printk("MEM 0x%08X ", ioread32(gpio + (7*4)));
		//rtdm_printk("MEM 0x%08X\n", ioread32(gpio + (13*4)));
		ledstate = 0;
	}
	else
	{
		//Set led off
		iowrite32(1<<25, gpio+(10*4));
		iowrite32(1<<23, gpio+(7*4));
		//rtdm_printk("MEM 0x%08X ", ioread32(gpio + (10*4)));
		//rtdm_printk("MEM 0x%08X\n", ioread32(gpio + (13*4)));
		ledstate = 1;
	}


}

static void leds_set2(int state)
{
	static int ledstate = 0;
	if(ledstate)
	{
		//Set led on
		iowrite32(1<<18, gpio+(7*4));
		iowrite32(1<<7, gpio+(10*4));
		//rtdm_printk("MEM 0x%08X ", ioread32(gpio + (7*4)));
		//rtdm_printk("MEM 0x%08X\n", ioread32(gpio + (13*4)));
		ledstate = 0;
	}
	else
	{
		//Set led off
		iowrite32(1<<18, gpio+(10*4));
		iowrite32(1<<7, gpio+(7*4));
		//rtdm_printk("MEM 0x%08X ", ioread32(gpio + (10*4)));
		//rtdm_printk("MEM 0x%08X\n", ioread32(gpio + (13*4)));
		ledstate = 1;
	}
}

static int leds_init(void)
{
	int 	value;
	rtdm_printk("KEMS: Init module... \n");


	//mem = request_mem_region(GPIO_BASE, 4096, "mygpio");

	  gpio =
	  ioremap_nocache((GPIO_BASE), 0x05cc);
	  if(!gpio)
	    {
	      rtdm_printk("KEMS: GPIO mapping failed\n");
	      return 0;
	    }

	  //We need to set port to output

	  rtdm_printk("KEMS: Set port pin25 to input\n");
	  //Set pin off
	  value = ioread32(gpio+(2*4));
	  value = value & ~(7<<15);		//Mask out our port
	  iowrite32(value, gpio+(2*4));	//Set port to input
	  //Set pin to output
	  rtdm_printk("KEMS: Set pin 25 to output\n");
	  //Set pin on
	  value = ioread32(gpio+(2*4));
	  value = value | (1<<15);		//Mask out our port
	  iowrite32(value, gpio+(2*4));	//Set port to input

	//  iowrite32(((1<<15)), gpio+0x16+(2*4));
	//  rtdm_printk("MEM 0x%08X\n", ioread32(gpio+(0x02*4)));

//	  rtdm_printk("KEMS: Set port FS0 to input\n");
	//  rtdm_printk("MEM 0x%08X\n", ioread32(gpio+0x16));

	  rtdm_printk("KEMS: Set port pin7 to input\n");
	  //Set pin off
	  value = ioread32(gpio);
	  value = value & ~(7<<21);		//Mask out our port
	  iowrite32(value, gpio);	//Set port to input
	  rtdm_printk("KEMS: Set port pin7 to output\n");
	  //Set pin on
	  value = ioread32(gpio);
	  value = value | (1<<21);		//Mask out our port
	  iowrite32(value, gpio);	//Set port to input

	  rtdm_printk("KEMS: Set port pin18 to input\n");
	  //Set pin off
	  value = ioread32(gpio+(1*4));
	  value = value & ~(7<<24);		//Mask out our port
	  iowrite32(value, gpio+(1*4));	//Set port to input
	  rtdm_printk("KEMS: Set port pin18 to output\n");
	  //Set pin on
	  value = ioread32(gpio+(1*4));
	  value = value | (1<<24);		//Mask out our port
	  iowrite32(value, gpio+(1*4));	//Set port to input

	  rtdm_printk("KEMS: Set port pin23 to input\n");
	  //Set pin off
	  value = ioread32(gpio+(2*4));
	  value = value & ~(7<<9);		//Mask out our port
	  iowrite32(value, gpio+(2*4));	//Set port to input
	  rtdm_printk("KEMS: Set port pin23 to output\n");
	  //Set pin on
	  value = ioread32(gpio+(2*4));
	  value = value | (1<<9);		//Mask out our port
	  iowrite32(value, gpio+(2*4));	//Set port to input

	 // rtdm_printk("MEM 0x%08X\n", ioread32(gpio+0x16));

	  //Set pin to output
	//  rtdm_printk("KEMS: Set pin 24 to output\n");
	 // iowrite32((1<<), gpio+0x16+(2*4));
	//  rtdm_printk("MEM 0x%08X\n", ioread32(gpio+(0x02*4)));


	return 0;
}

static void leds_cleanup(void)
{
//	int i;
//
//	for (i = GPIO_PF6; i <= GPIO_PF11; i++)
//		gpio_free(i);

	//void iowrite32(u32 value, void *addr);

	rtdm_printk("KEMS: Deinit module... \n");
}

//static inline void leds_set(int state) { }
//static inline int leds_init(void) { return 0; }
//static inline void leds_cleanup(void) { }

/* Generic part: A simple periodic RTDM kernel space task */
void heartbeat(void *cookie)
{
	int state = 0;

	while (!end) {
		rtdm_task_wait_period();

		leds_set(state++);
	}
}

void heartbeat2(void *cookie)
{
	int state = 0;

	while (!end) {
		rtdm_task_wait_period();

		leds_set2(state++);
	}
}

int gpio_interrupt_rtdm(rtdm_irq_t *irq_handle)
{
	rtdm_printk("Interrupt RTDM\n\r");

   return RTDM_IRQ_HANDLED;
}

//static int isr_code(xnintr_t *intr)
static int gpio_interrupt(xnintr_t *intr)
{
	//rtdm_printk("Interrupt\n\r");

	rt_printf("Interrupt\n\r");

	return RTDM_IRQ_HANDLED;
}

int __init init_heartbeat(void)
{
	int err;

	err = leds_init();
	if (err)
		return err;

	rtdm_task_init(&heartbeat_task, "heartbeat", heartbeat, NULL,
			      98, HEARTBEAT_PERIOD);

	rtdm_task_init(&heartbeat_task2, "heartbeat2", heartbeat2, NULL,
			      99, HEARTBEAT_PERIOD2);

	int rt_intr_create ( RT_INTR  intr, const char  name, unsigned irq, rt_isr_t isr,
	rt_iack_t iack, int mode )

	err = rt_intr_create(&intr_desc, "GPIO-int", 52, gpio_interrupt , NULL, RT_INTR_PROPAGATE);
	if(err < 0)
	{
		rtdm_printk("Error creating interrupt \n");
	}


//	err = rtdm_irq_request(&intr_rtdm_desc, 52, gpio_interrupt_rtdm, 0, "GPIO-int-rtdm", 0 );
//	if(err < 0)
//	{
//		rtdm_printk("Error creating interrupt \n");
//	}


	return 0;
}

void __exit cleanup_heartbeat(void)
{
	end = 1;
	rtdm_task_join_nrt(&heartbeat_task, 100);
	leds_cleanup();
}

module_init(init_heartbeat);
module_exit(cleanup_heartbeat);
