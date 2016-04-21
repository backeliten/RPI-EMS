/*
 * 	This is a demo of how to use a RPi as a realtime system support for GPIO
 */

#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/math64.h>

#include <native/timer.h>
#include <native/queue.h>

#include <rtdm/rtdm_driver.h>

#define QUEUE_SUPPORT

MODULE_LICENSE("GPL");

///*To setup memory access*/
//#define PAGE_SIZE (4*1024)
//#define BLOCK_SIZE (4*1024)

#include "../header/RPi-KEMS.h"
#include "../header/fuelcontrol.h"
#include "../header/wheeldecoder.h"
#include "../header/communication.h"
#include "../header/spi.h"

/*Defines for PIN-conf*/
#define MAIN_TRIGGER	4 		//This is the pin for trigger input!

#define IGN_OUT_1 		23
#define IGN_OUT_2		24

#define INJ_OUT_1		17
#define INJ_OUT_2		25

static void __iomem *gpio = NULL;
static void __iomem *spi = NULL;

#define HEARTBEAT_PERIOD	100000000 /* 100 ms */
#define HEARTBEAT_PERIOD2	50000000 /* 50 ms */

#define MAIN_SLEEP 			10000000 /* 0.1 sec */ //100Hz
#define COMM_SLEEP			500000000 /* 0.2 sec */
#define BACK_SLEEP			1000000000 /* 0.3 sec */

static RTIME time = 0;
//static RTIME old_time = 0;

/*We include the c-file, until we figured a way to compile to one module*/
#include "wheeldecoder.c"
#include "fuelcontrol.c"
#include "ignitioncontrol.c"
#include "maintrigger.c"

//#include "spi.c"
//#include "mcp3208.c"

static void SPI_init(void);
int8_t	SPI_read_and_write(uint8_t cs, uint8_t size, uint8_t *in_array, uint8_t *out_array);

//#include "fuelcalc.c"		//Kernel does not support float support, therefore we cannot compile this one

//FIXME: Do calc in user rt app and send via rtp?

/*Function declaration*/

//static int gpio_interrupt(rtdm_irq_t *irq_handle);
//static int trigger_init(void);

static rtdm_task_t			main_task;
static rtdm_task_t			comm_task;
static rtdm_task_t			back_task;

RT_QUEUE 			q_desc;

char *gpio_mem, *gpio_map;
char end = 0;

struct resource * mem;

static ku_comm_s 		*ku_comm;
static ku_comm_s 		ku_comm_backup;

/*Threads*/
static void main_loop(void *cookie)
{
//	static int toggle = 0;
//	int ret = 0;
//	int inj0_fire_var = 0;
//	int inj1_fire_var = 0;
//
//	int ign0_fire = 0;
//	int ign1_fire = 0;

	//Do main stuff here, read inputs and set non critical outputs

	while (!end) {
		rtdm_task_wait_period();

//		if(toggle == 0)
//		{
//			gpio_set_value(INJ_OUT_1, 0);
//			//gpio_set_value(INJ_OUT_2, 1);
//			toggle = 1;
//		}
//		else
//		{
//			gpio_set_value(INJ_OUT_1, 1);
//			//gpio_set_value(INJ_OUT_2, 0);
//			toggle = 0;
//		}
		//Set alarm timer for inj0
//		int rtdm_timer_start ( rtdm_timer_t  timer, nanosecs_abs_t expiry, nanosecs_rel_t
//		interval, enum rtdm_timer_mode mode )

//		if(inj0_fire_var)
//		{
//			inj0_fire(3000);
//			inj0_fire_var = 0;
//		}
//
//		if(inj1_fire_var)
//		{
//			inj1_fire(3000);
//			inj1_fire_var = 0;
//		}
//
//		if(ign0_fire)
//		{
//			ret = rtdm_timer_start(&ign0_timer, 3500000, 0, RTDM_TIMERMODE_RELATIVE);  //3m500u000n
//			if(ret != 0)
//			{
//				//FIXME Handle error
//			}
//			else
//			{
//				//iowrite32(1<<22, gpio+(7*4));		//Set output on
//				gpio_set_value(22, 1);					//Pin 22, value 1
//			}
//			ign0_fire = 0;
//		}
//		if(ign1_fire)
//		{
//			ret = rtdm_timer_start(&ign1_timer, 3500000, 0, RTDM_TIMERMODE_RELATIVE);  //3m500u000n
//			if(ret != 0)
//			{
//				//FIXME Handle error
//			}
//			else
//			{
//				//iowrite32(1<<23, gpio+(7*4)); 		//Set output on
//				gpio_set_value(23, 1);					//Pin 22, value 1
//			}
//			ign1_fire = 0;
//		}
		//leds_set(state++);
	}
}

static void comm_loop(void *cookie)
{
	int ret;
//	char array[10] = "Message";
//	int ret = 0;
	//FIXME: Keep communication to non realtime task, easy QaA protocol, could use same as Tunerstudio?
	while (!end) {
		rtdm_task_wait_period();

//		rtdm_printk("KEMS: RPM: %5u RESYNC: %3u TRIGGER STATUS: ", trigger.rpm, trigger.trigger_err_pos);
//
//		if(trigger.sync == SYNC)
//		{
//			rtdm_printk("SYNC\n");
//		}
//		else
//		{
//			rtdm_printk("NOT SYNC\n");
//		}
		ku_comm->rpm = trigger.rpm;
		ku_comm->sync = trigger.sync;

//		ret = rt_queue_send(&q_desc, ku_comm, sizeof(ku_comm_s), Q_NORMAL);
//		if(ret < 0 )
//		{
//			rtdm_printk("KEMS: Error sending to queue\n");
//		}
#ifdef QUEUE_SUPPORT
		ret = rt_queue_write(&q_desc,ku_comm, sizeof(ku_comm_s), Q_NORMAL);
		if(ret < 0 )
		{
			rtdm_printk("KEMS: Error sending to queue\n");
		}
#endif

		//leds_set(state++);
	}
}

static void back_loop(void *cookie)
{
	uint8_t data_in = {0};
	uint8_t data_out = {0};
	int ret;

	//FIXME: Background tasks, diagnose, and other plausiblity checks
	while (!end) {
		rtdm_task_wait_period();
		if(trigger.sync == SYNC)
		{
			if((rt_timer_read() - time) > 2000000000)
			{
				rtdm_printk("LOST TRIGGER, resetting! \n");
				trigger.sync = SYNC_FIRST_TOOTH;		//Clear sync flag
				trigger.rpm = 0;		//Clear rpm value;
			}
		}

//		//Try to send some SPI..
//		rtdm_printk("Send some SPI data...");
//		ret = SPI_read_and_write(0, 3, &data_in, &data_out);
//		if(ret < 0)
//		{
//			rtdm_printk("FAILED\n");
//		}
//		else
//		{
//			rtdm_printk("SUCCESS\n");
//		}

		//leds_set(state++);
	}
}

/*Init stuff*/
static int IO_init(void)
{
	//Setup all the ports

	inj0_init();
	inj0_timer_init();
	inj1_init();
	inj1_timer_init();

	ign0_init();
	ign0_timer_init();
	ign1_init();
	ign1_timer_init();

	trigger_init();

	SPI_init();

	 return 0;
}
static void IO_deinit(void)
{
	trigger_deinit();

	inj0_deinit();
	inj0_timer_deinit();
	inj1_deinit();
	inj1_timer_deinit();

	ign0_deinit();
	ign0_timer_deinit();
	ign1_deinit();
	ign1_timer_deinit();

}

static int DEV_init(void)
{
	//Init SPI
	//mcp3208_init();
	return 0;
}
static void DEV_deinit(void)
{
	//mcp3208_deinit();
	//Deinit SPI
}

static int Queue_init(void)
{
	int ret;
	//void *buffert;
	//ku_comm_s 	ku_comm;
#ifdef QUEUE_SUPPORT
	ret = rt_queue_create(&q_desc, "KEMS-RTDATA", 2048, 1, Q_SHARED);
	if(ret < 0)
	{
		return -1;
	}
#endif

	ku_comm = &ku_comm_backup;

//	buffert = rt_queue_alloc(&q_desc, sizeof(ku_comm_s));
//	if(buffert != NULL)
//	{
//		ku_comm = (ku_comm_s*)buffert;
//	}
//	else
//	{
//		//Use internal buffert, will make it to not crash
//		ku_comm = &ku_comm_backup;
//
//	}
//	static int pipe_fd;
//    char devname[32], buf[16];

//    pipe_fd = open("/dev/rtp0", O_RDWR);
//
//    if (pipe_fd < 0)
//            fail();
//
//    /* Wait for the prompt string "Hello"... */
//    read(pipe_fd, buf, sizeof(buf));
//
//    /* Then send the reply string "World": */
//    write(pipe_fd, "World", sizeof("World"));
//
//    /* ... */

	return 0;
}

static int KEMS_init(void)
{
	int 	ret;
	rtdm_printk("KEMS: Init module... \n");

	//mem = request_mem_region(GPIO_BASE, 4096, "mygpio");

	  gpio = ioremap_nocache((GPIO_BASE), 0x05cc);
	  if(!gpio)
	  {
		  rtdm_printk("KEMS: GPIO mapping failed\n");
		  return 0;
	  }

	  spi = ioremap_nocache((SPI_BASE), 0x020);		//We need size of 0x20
	  if(!spi)
	  {
		  rtdm_printk("KEMS: SPI mapping failed\n");
		  return 0;
	  }

	  /*
	   * Setup I/O
	   */
	  ret = IO_init();
	  if(ret != 0)
	  {
		  return -1;
	  }

	  /*
	   * Setup Devices
	   */
	  ret = DEV_init();
	  if(ret != 0)
	  {
		  return -1;
	  }
	  //Initiate wheeldecoder
	  wh_init();

	  /* Setup queue to userprocess*/
	  Queue_init();

	  /*
	   * Start all threads for KEMS
	   */
	  ret = rtdm_task_init(&main_task, "EMS-Main", main_loop, NULL, 90, MAIN_SLEEP);
	  if(ret != 0)
	  {
		  return -1;
	  }
	  ret = rtdm_task_init(&comm_task, "EMS-Comm", comm_loop, NULL, 60, COMM_SLEEP);
	  if(ret != 0)
	  {
		  return -1;
	  }
	  ret = rtdm_task_init(&back_task, "EMS-Back", back_loop, NULL, 50, BACK_SLEEP);
	  if(ret != 0)
	  {
		  return -1;
	  }

	return 0;
}

static void bcm2708_init_pinmode(void)
{
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

	int pin;
	u32 *gpio = ioremap(0x20200000, SZ_16K);

	/* SPI is on GPIO 7..11 */
	for (pin = 7; pin <= 11; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
	}

	iounmap(gpio);

#undef INP_GPIO
#undef SET_GPIO_ALT
}

static void SPI_init(void)
{
	//int pin;
	int i;
	int value;

	bcm2708_init_pinmode();

//#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
//#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

//	/* SPI is on GPIO 8..11 */
//	for (pin = 8; pin <= 11; pin++) {
//		INP_GPIO(pin);		/* set mode to GPIO input first */
//		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
//	}

    /*SPI FUNCTION HERE*/
  //  *(spi+SPI_CS) = 0x00;		//Shutdown SPI controller
    writel(0x00, spi+SPI_CS);

//    //Set clock divider to 150 -> Gives 1Mhz on SPI bus (master clock of 150Mhz)
//    *(spi+SPI_CLK) = 150;		//Gives around 1.8Mhz on SPI
    writel(150, spi+SPI_CLK);
//
//    //Clear fifo buffert		//Do we need this??  Could we check the flag if buffert is empty.
    for(i=0;i<10;i++)
    {
    	value = readl(spi+SPI_FIFO);
    	//value = *(spi+SPI_FIFO);
    }

	value = readl(spi+SPI_CS);
	rtdm_printk("SPI_CS read 0x%08X \n", value);

//#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
//#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

}

//static void bcm2708_init_pinmode(void)
//{
//#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
//#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
//
//	int pin;
//	u32 *gpio = ioremap(0x20200000, SZ_16K);
//
//	/* SPI is on GPIO 7..11 */
//	for (pin = 7; pin <= 11; pin++) {
//		INP_GPIO(pin);		/* set mode to GPIO input first */
//		SET_GPIO_ALT(pin, 0);	/* set mode to ALT 0 */
//	}
//
//	iounmap(gpio);
//
//#undef INP_GPIO
//#undef SET_GPIO_ALT
//}

int8_t	SPI_read_and_write(uint8_t cs, uint8_t size, uint8_t *in_array, uint8_t *out_array)
{
	int i = 0;
	int out_value = 0;
	int in_value = 0;
	int value = 0;

	if(size == 0)
	{
		return -1;		//Cannot read array which is zero
	}
	if((cs == 0) | (cs == 1))		//CS0
	{
		//Set CS, CPOL, CPHA
		//Set TA = 1
		//*(spi+SPI_CS) = (cs & 0x01) |  SPI_CS_TA;		//Shutdown SPI controller
		 writel(((cs & 0x01) |  SPI_CS_TA), spi+SPI_CS);

	//	for(i=0;i<size;i++)
	//	{
			//Write FIFO
			//*(spi+SPI_FIFO) = 0xFF;
			writel(in_value, spi+SPI_FIFO);

			//Wait for DONEflag
			//while((readl(spi+SPI_CS) & SPI_CS_DONE) != 1);
//			value = readl(spi+SPI_CS);
//			rtdm_printk("SPI_CS read 0x%08X ", value);
			//Read FIFO
//			out_value = (readl(spi+SPI_FIFO));
//			rtdm_printk("SPI_FIFO read 0x%08X ", out_value);

	//	}

		//Set TA = 0
		 writel(0x00, spi+SPI_CS);
		//*(spi+SPI_CS) = 0x00;		//Shutdown SPI controller
	}
	else
	{
		return -2;		//Does not support other CS and 0 and 1
	}

	return 0;
}

//int8_t	SPI_read_and_write(uint8_t cs, uint8_t size, uint8_t *in_array, uint8_t *out_array)		//in_array is from Master to slave, out_array is from slave to master
//{
////	int i = 0;
////
////	if(size != 0)
////	{
////		return -1;		//Cannot read array which is zero
////	}
////	if((cs == 0) | (cs == 1))		//CS0
////	{
////		//Set CS, CPOL, CPHA
////		//Set TA = 1
////		*(spi+SPI_CS) = (cs & 0x01) |  SPI_CS_TA;		//Shutdown SPI controller
////
////		for(i=0;i<size;i++)
////		{
////			//Write FIFO
////			*(spi+SPI_FIFO) = in_array[i];
////			//Wait for DONEflag
////			while((spi+SPI_CS & SPI_CS_DONE) != 1);
////			//Read FIFO
////			out_array[i] = *(spi+SPI_FIFO);
////		}
////
////		//Set TA = 0
////		*(spi+SPI_CS) = 0x00;		//Shutdown SPI controller
////	}
////	else
////	{
////		return -2;		//Does not support other CS and 0 and 1
////	}
//
//	return 0;
//}

static void KEMS_cleanup(void)
{
	//FIXME: Set default values to GPIO, so we dont end up in a disaster
	IO_deinit();

	DEV_deinit();

	rtdm_printk("KEMS: Deinit module... \n");

	rtdm_task_join_nrt(&main_task, 100);
	rtdm_task_join_nrt(&comm_task, 100);
	rtdm_task_join_nrt(&back_task, 100);

	rtdm_task_destroy(&main_task);
	rtdm_task_destroy(&comm_task);
	rtdm_task_destroy(&back_task);
}

/*Module init and deinit*/
int __init init_KEMS(void)
{
	int err;

	err = KEMS_init();
	if (err)
		return err;

	return 0;
}

void __exit cleanup_KEMS(void)
{
	end = 1;

	KEMS_cleanup();
}

module_init(init_KEMS);
module_exit(cleanup_KEMS);
