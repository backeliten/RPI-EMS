/*
 * RPi-KEMS.h
 *
 *  Created on: Mar 20, 2013
 *      Author: jonas
 */

#ifndef RPI_KEMS_H_
#define RPI_KEMS_H_

/*Memory base adress to reach Perpherial*/
#define BCM2708_PERI_BASE        0x20000000

#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) 	/* GPIO controller */
#define INTR_BASE				 (BCM2708_PERI_BASE + 0x00B000)		//0x7E00B000		/*Interrupt Controller*/
#define SPI_BASE				 (BCM2708_PERI_BASE + 0x204000)		/*SPI Controller*/

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

#endif /* RPI_KEMS_H_ */
