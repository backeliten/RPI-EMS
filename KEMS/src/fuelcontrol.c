/*
 * fuelcontrol.c
 *
 *  Created on: 24 mar 2013
 *      Author: bkj1mll
 */

static rtdm_timer_t inj0_timer;
static rtdm_timer_t inj1_timer;

void inj0_fire(int timer_us)
{
	int ret;
	ret = rtdm_timer_start(&inj0_timer, (timer_us*1000), 0, RTDM_TIMERMODE_RELATIVE);
	if(ret != 0)
	{
		//FIXME Handle error
	}
	else
	{
		gpio_set_value(INJ_OUT_1, 1);
	}
}
void inj1_fire(int timer_us)
{
	int ret;
	ret = rtdm_timer_start(&inj1_timer, (timer_us*1000), 0, RTDM_TIMERMODE_RELATIVE);
	if(ret != 0)
	{
		//FIXME Handle error
	}
	else
	{
		gpio_set_value(INJ_OUT_2, 1);
	}
}
static int inj0_init(void)
{
	int err;

	/*Aquire Inj 1 output*/
	if ((err = gpio_request(INJ_OUT_1, THIS_MODULE->name)) != 0) {
		return err;
	}
	if ((err = gpio_direction_output(INJ_OUT_1, 1)) != 0)
	{
		gpio_free(INJ_OUT_1);
		return err;
	}

	return 0;
}
static int inj1_init(void)
{
	int err;

	/*Aquire Inj 2 output*/
	if ((err = gpio_request(INJ_OUT_2, THIS_MODULE->name)) != 0) {
		return err;
	}
	if ((err = gpio_direction_output(INJ_OUT_2, 1)) != 0)
	{
		gpio_free(INJ_OUT_2);
		return err;
	}

	return 0;
}
static void inj0_int(rtdm_timer_t *timer)
{
	/*Set output off*/
	gpio_set_value(INJ_OUT_1, 0);
}
static void inj1_int(rtdm_timer_t *timer)
{
	/*Set output off*/
	gpio_set_value(INJ_OUT_2, 0);
}
static int inj0_timer_init(void)
{
	int ret;
	  ret = rtdm_timer_init(&inj0_timer, inj0_int, "EMS-Inj0");
	  if(ret != 0)
	  {
		  return -1;
	  }
	  return 0;
}
static int inj1_timer_init(void)
{
	int ret;
	  ret = rtdm_timer_init(&inj1_timer, inj1_int, "EMS-Inj1");
	  if(ret != 0)
	  {
		  return -1;
	  }
	  return 0;
}
static void inj0_deinit(void)
{
	gpio_set_value(INJ_OUT_1, 0);
	gpio_free(INJ_OUT_1);
}
static void inj1_deinit(void)
{
	gpio_set_value(INJ_OUT_2, 0);
	gpio_free(INJ_OUT_2);
}
static void inj0_timer_deinit(void)
{
	rtdm_timer_destroy(&inj0_timer);
}
static void inj1_timer_deinit(void)
{
	rtdm_timer_destroy(&inj1_timer);
}
