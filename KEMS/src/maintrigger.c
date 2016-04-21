/*
 * maintrigger.c
 *
 *  Created on: 7 jun 2013
 *      Author: bkj1mll
 */

static rtdm_irq_t			irq_main_trigger;

static int gpio_interrupt(rtdm_irq_t *irq_handle)
{
	RTIME 				temp_time;
	static SRTIME		curr_time;
	static SRTIME		prev_time;
	static SRTIME		diff_time;
	static int			freq;

	temp_time = rt_timer_tsc();

	curr_time = rt_timer_tsc2ns(temp_time);
	diff_time = curr_time - prev_time;
	prev_time = curr_time;

	//Get frequency
	freq = (uint32_t)div_u64(100000000000, diff_time);

	//rtdm_printk("F: %u \n", freq);

	set_next_tooth(curr_time);

	//freq = 100000000000 / diff_time;  //Diff_time is in ns, freq = times*1000

    return RTDM_IRQ_HANDLED;
}

static int trigger_init(void)
{
	int err;

	/*Aquire Trigger input*/
	int numero_interruption = gpio_to_irq(MAIN_TRIGGER);
	if ((err = gpio_request(MAIN_TRIGGER, THIS_MODULE->name)) != 0)
	{
		return err;
	}
	if ((err = gpio_direction_input(MAIN_TRIGGER)) != 0)
	{
		gpio_free(MAIN_TRIGGER);
		return err;
	}

	/*Setup interrupt for TRIGGER*/
	irq_set_irq_type(numero_interruption,  IRQF_TRIGGER_RISING);
	if ((err = rtdm_irq_request(&irq_main_trigger,
	                 numero_interruption, gpio_interrupt,
	                 RTDM_IRQTYPE_EDGE,
	                 THIS_MODULE->name, NULL)) != 0)
	{
		gpio_free(MAIN_TRIGGER);
		return err;
	}

	return 0;
}
static void trigger_deinit(void)
{
	rtdm_irq_free(&irq_main_trigger);

	gpio_free(MAIN_TRIGGER);
}
