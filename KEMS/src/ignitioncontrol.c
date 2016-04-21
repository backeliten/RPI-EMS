/*
 * ignitioncalc.c
 *
 *  Created on: 20 feb 2013
 *      Author: bkj1mll
 */

static rtdm_timer_t ign0_timer;
static rtdm_timer_t ign1_timer;

void ign0_fire(double angle)
{
//	int ret;
//	ret = rtdm_timer_start(&inj0_timer, (timer_us*1000), 0, RTDM_TIMERMODE_RELATIVE);
//	if(ret != 0)
//	{
//		//FIXME Handle error
//	}
//	else
//	{
//		gpio_set_value(INJ_OUT_1, 1);
//	}
}
void ign1_fire(double angle)
{
//	int ret;
//	ret = rtdm_timer_start(&inj1_timer, (timer_us*1000), 0, RTDM_TIMERMODE_RELATIVE);
//	if(ret != 0)
//	{
//		//FIXME Handle error
//	}
//	else
//	{
//		gpio_set_value(INJ_OUT_2, 1);
//	}
}
static int ign0_init(void)
{
	int err;

	/*Aquire Ign 1 output*/
	if ((err = gpio_request(IGN_OUT_1, THIS_MODULE->name)) != 0) {
		return err;
	}
	if ((err = gpio_direction_output(IGN_OUT_1, 1)) != 0)
	{
		gpio_free(IGN_OUT_1);
		return err;
	}

	return 0;
}
static int ign1_init(void)
{
	int err;

	/*Aquire Ign 2 output*/
	if ((err = gpio_request(IGN_OUT_2, THIS_MODULE->name)) != 0) {
		return err;
	}
	if ((err = gpio_direction_output(IGN_OUT_2, 1)) != 0)
	{
		gpio_free(IGN_OUT_2);
		return err;
	}

	return 0;
}
static void ign0_int(rtdm_timer_t *timer)
{
	gpio_set_value(IGN_OUT_1, 0);
}
static void ign1_int(rtdm_timer_t *timer)
{
	gpio_set_value(IGN_OUT_2, 0);
}
static int ign0_timer_init(void)
{
	int ret;
	  ret = rtdm_timer_init(&ign0_timer, ign0_int, "EMS-Ign0");
	  if(ret != 0)
	  {
		  return -1;
	  }
	  return 0;
}
static int ign1_timer_init(void)
{
	int ret;
	  ret = rtdm_timer_init(&ign1_timer, ign1_int, "EMS-Ign1");
	  if(ret != 0)
	  {
		  return -1;
	  }
	  return 0;
}
static void ign0_deinit(void)
{
	gpio_set_value(IGN_OUT_1, 0);
	gpio_free(IGN_OUT_1);
}
static void ign1_deinit(void)
{
	gpio_set_value(IGN_OUT_2, 0);
	gpio_free(IGN_OUT_2);
}
static void ign0_timer_deinit(void)
{
	rtdm_timer_destroy(&ign0_timer);
}
static void ign1_timer_deinit(void)
{
	rtdm_timer_destroy(&ign1_timer);
}
