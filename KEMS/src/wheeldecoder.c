/*
 * wheeldecoder.c
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */

/*Variable declaration*/
static volatile triggerwheel_s	trigger;
static volatile uint8_t 	downcount = 4;

unsigned int 	diff_time = 0;
unsigned int	diff_time_nom = 1;
unsigned int 	rev = 0;
unsigned int 	diff = 0;

unsigned int	speed = 0;
unsigned int	speed_time;
unsigned int	prev_speed_time;

//#define UINT64 long long unsigned int
#define UINT32 long unsigned int

void wh_init(void)
{
	trigger.sync = SYNC_FIRST_TOOTH;
	trigger.count = 0;
	trigger.trigger_err_pos = 0;
}

//int8_t set_next_tooth_2(RTIME time)
//{
//
//	unsigned int internal_time;
//
//	static int i = 0;
//	static RTIME average[10];
//	static RTIME average_value;
//
//	internal_time = (uint32_t)div_u64(time, 1000);
//
//	if(i > 9)
//	{
//		i = 0;
//	}
//	else
//	{
//		average[i] = internal_time;
//	}
//
//
//}


int8_t set_next_tooth(RTIME time)
{
	 	// trigger.triggertooth[trigger.count] =  (uint32_t)(time);

		trigger.triggertooth[trigger.count] = (uint32_t)div_u64(time, 1000);

		//trigger.triggertooth[trigger.count] =  (uint32_t)(time / 1000);

	 	// rtdm_printk("INTERRUPT: Current time %16llu | diff: %16llu ", time, (read_time-prev_time));
		if(trigger.count == 0)
		{
		 trigger.triggertooth_diff[trigger.count] = trigger.triggertooth[trigger.count] - trigger.triggertooth[57];
		}
		else
		{
		 trigger.triggertooth_diff[trigger.count] = trigger.triggertooth[trigger.count] - trigger.triggertooth[trigger.count-1];
		}

//		if(trigger.sync == SYNC)
//		{
//			rtdm_printk("SYNC | ");
//		}
//		else
//		{
//			rtdm_printk("NO_SYNC | ");
//		}

		//rtdm_printk("TRIGGER: Count: %02u | Diff:  %10u | Speed %u \n", trigger.count,  trigger.triggertooth_diff[trigger.count], speed);

		//if((trigger.count % 4) == 0 && trigger.count > 4)
		if((trigger.count == 10) | (trigger.count == 25) | (trigger.count == 40) | (trigger.count == 55))
		{
		 speed_time = trigger.triggertooth[trigger.count];
		 speed = (10000000 / (speed_time - prev_speed_time))*6;
		 trigger.rpm = speed / 4;
		 prev_speed_time = speed_time;
		 rtdm_printk("S: %u %u \n", trigger.rpm, trigger.count );
		}

		if(trigger.count == 0)
		{
		 diff_time = ((unsigned int)trigger.triggertooth_diff[trigger.count]);
		 diff_time_nom = (unsigned int)(trigger.triggertooth_diff[57]/100);
		 if(diff_time_nom == 0)
		 {
			 diff_time_nom = 1;
		 }
		 diff = diff_time / diff_time_nom;
		}
		else
		{
		 diff_time = ((unsigned int)trigger.triggertooth_diff[trigger.count]);
		 diff_time_nom = (unsigned int)(trigger.triggertooth_diff[trigger.count-1]/100);
		 if(diff_time_nom == 0)
		 {
			 diff_time_nom = 1;
		 }
		 diff = diff_time / diff_time_nom;
		}

		//rtdm_printk(" T:%2u D:%3u \n",trigger.count, diff++);
		if(rev > 1)
		{
		   if(trigger.count == 0 && trigger.sync == SYNC)
		   {
			   if((diff < 200) | (diff > 400))
			   {
				   //We have missed the missing tooth, we need to resync
				 trigger.count = 0;
				 trigger.sync = SYNC_FIRST_TOOTH;
				 trigger.trigger_err_pos++;
				 rev = 0;
				 rtdm_printk("| OUT OF SYNC, MISS_TOOTH E_Count: %3u DIFF %u \n", trigger.trigger_err_pos, diff);

			   }
			   //Check if we have missing tooth
		   }

		 if(trigger.count != 0 && trigger.count != 1 && trigger.count != 2 && trigger.sync == SYNC)
		 {
			if((diff < 20) | (diff > 180))
			{
				 trigger.count = 0;
				 trigger.sync = SYNC_FIRST_TOOTH;
				 trigger.trigger_err_pos++;
				 rev = 0;
				 rtdm_printk("| OUT OF SYNC, ToT E_Count: %3u DIFF: %u \n", trigger.trigger_err_pos, diff);
			}
		 }
		}

		if(downcount > 0)
		{
		   downcount--;
		}
		else
		{
			if(trigger.sync == SYNC_FIRST_TOOTH)
			{
				//rtdm_printk("| NOTSYNC ");
				if(diff > 200)
				{
					rtdm_printk("| FOUND MISSING, resync table \n");
				 trigger.count = 0;
				 trigger.sync = SYNC;
				 rev = 0;
				}

			}
		}
		// rt_printf("| diff %u \n\r", diff);

//		 if((trigger.sync == SYNC)&&(trigger.count == 12))		//Ignition 1
//		 {
//			 //set_ign0_time(3000000); 	//3ms
//		 }
//		 if((trigger.sync == SYNC)&&(trigger.count == 42))		//Ignition 2
//		 {
//			 //set_ign1_time(3000000); 	//3ms
//		 }
//		 if((trigger.sync == SYNC)&&(trigger.count == 20))		//Injector 1
//		 {
//			 //sset_inj0_time(8000000); 	//8ms
//		 }
//		 if((trigger.sync == SYNC)&&(trigger.count == 50))		//Injector 2
//		 {
//			 //set_inj1_time(8000000); 	//8ms
//		 }

		trigger.count++;
		if(trigger.count == 58)
		{
		 trigger.count = 0;
		 rev++;
		}


	return trigger.sync;
}

uint16_t get_current_tooth(void)
{
	return trigger.count;
}


