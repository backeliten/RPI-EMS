/*
 * cutnpaste.c
 *
 *  Created on: Mar 20, 2013
 *      Author: jonas
 */


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


// rtdm_printk("MEM 0x%08X\n", ioread32(gpio+0x16));

 //Set pin to output
//  rtdm_printk("KEMS: Set pin 24 to output\n");
// iowrite32((1<<), gpio+0x16+(2*4));
//  rtdm_printk("MEM 0x%08X\n", ioread32(gpio+(0x02*4)));



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



//	static int toggle = 0;

//	static nanosecs_abs_t  	current_time;
//	static nanosecs_abs_t	prev_time;
//	static nanosecs_abs_t	diff_time;
//	unsigned int nr_interrupts = 0;
//	int err;

//	int toggle = 0;
//	RTIME		curr_time;
//	RTIME		prev_time;
//	RTIME		diff_time;

//	unsigned int temp;

//	current_time =  rtdm_clock_read_monotonic();
//	diff_time = (current_time - prev_time);
//	prev_time = current_time;

//	time  = rt_timer_read();
	//rtdm_printk("I %10llu \n", (time-old_time) );
//	set_next_tooth(time);
	//rt_printf("Diff:%10llu\n\r", (detect_time-set_time));
//	old_time = time;

//	if(toggle == 0)
//	{
//		gpio_set_value(INJ_OUT_2, 1);
//		toggle = 1;
//	}
//	else
//	{
//		gpio_set_value(INJ_OUT_2, 0);
//		toggle = 0;
//	}

//    err = rt_intr_enable(&gpio_intr_desc);		//This should enable the interrupt
//    if(err < 0)
//    {
//    	rt_printf("Could not enable interrupt err %d \n\r", err);
//    }

//    while(1)
//    {
       /* Wait for the next interrupt */
//       nr_interrupts = rt_intr_wait(&gpio_intr_desc,rt_timer_ns2ticks(500000000));
//
//      // rt_printf("passed interrupt \n\r");
//
//       if(nr_interrupts == -ETIMEDOUT)
//       {
//    	   trigger.sync = SYNC_FIRST_TOOTH;
//    	   trigger.count = 0;
//    	   speed = 0;
//    	  // trigger.trigger_err_pos;
//    	   rtdm_printk("GOT TIMEOUT\n\r");
//       }else
//       {

//    	   curr_time = rt_timer_read();
//    	   diff_time = curr_time - prev_time;
//    	   prev_time = curr_time;
//
////    	   rt_printf("GOT INTERRUPT\n\r");
////    	   rt_printf("D: 0x%08X\n\r", *(gpio + 17));
////    	   *(gpio + 17) = (1<<4);
////    	   rt_printf("0: 0x%08X\n\r", *(intr+0x80+0x00));
////    	   rt_printf("1: 0x%08X\n\r", *(intr+0x80+0x01));
////    	   rt_printf("2: 0x%08X\n\r", *(intr+0x80+0x02));
////     	   rt_printf("3: 0x%08X\n\r", *(intr+0x80+0x03));
////     	   rt_printf("4: 0x%08X\n\r", *(intr+0x80+0x04));
////     	   rt_printf("5: 0x%08X\n\r", *(intr+0x80+0x05));
////     	   rt_printf("6: 0x%08X\n\r", *(intr+0x80+0x06));
////    	   rt_printf("C: %u\n\r", trigger.count);
////    	   trigger.count++;
//    	  // rt_printf("3: 0x%08X\n\r", *(intr+0x80+0x03));
//    	  // rt_printf("4: 0x%08X\n\r", *(intr+0x80+0x04));
//
//    	   if((diff_time > 1100000) || (diff_time < 800000))
//    	   {
//    		   rev++;
//    		   //rt_printk("M %u ", rev);
//    		   rtdm_printk("OVERRUN %10llu \n\r", diff_time);
//
////        	   if(toggle == 0)
////        	   {
////        		   GPIO_CLR = 1<<7;
////        		   toggle = 1;
////        	   }
////        	   else
////        	   {
////        		   GPIO_SET = 1<<7;
////        		   toggle = 0;
////        	   }
//
//    	   }

    	   //rt_printf("|D: %10llu\n\r",trigger.triggertooth_diff[0]);

     //  }
       	   //rt_intr_enable(&gpio_intr_desc);		//This should enable the interrupt
//       err = rt_intr_enable(&gpio_intr_desc);		//This should enable the interrupt
//       if(err < 0)
//       {
//       	rt_printf("Could not enable interrupt err %d \n\r", err);
//       }
  //  }
