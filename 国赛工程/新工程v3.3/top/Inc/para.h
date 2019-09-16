#ifndef __PARA_H
#define __PARA_H

#include "mytype.h"


//led
#define LED0STATES_PARA_DEFAULT \
{\
	GPIOI,\
	GPIO_PIN_0,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED1STATES_PARA_DEFAULT \
{\
	GPIOH,\
	GPIO_PIN_12,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED2STATES_PARA_DEFAULT \
{\
	GPIOH,\
	GPIO_PIN_11,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED3STATES_PARA_DEFAULT \
{\
	GPIOH,\
	GPIO_PIN_10,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED4STATES_PARA_DEFAULT \
{\
	GPIOD,\
	GPIO_PIN_15,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED5STATES_PARA_DEFAULT \
{\
	GPIOD,\
	GPIO_PIN_14,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED6STATES_PARA_DEFAULT \
{\
	GPIOD,\
	GPIO_PIN_13,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED7STATES_PARA_DEFAULT \
{\
	GPIOD,\
	GPIO_PIN_12,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\


#define LED8STATES_PARA_DEFAULT \
{\
	GPIOE,\
	GPIO_PIN_11,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\

#define LED9STATES_PARA_DEFAULT \
{\
	GPIOF,\
	GPIO_PIN_14,\
	&LedOn,\
	&LedOff,\
	&LedToggle,\
}\
 





#endif
