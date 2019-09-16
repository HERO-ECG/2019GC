#ifndef __IDLE_INTERRUPT_H__
#define __IDLE_INTERRUPT_H__

#include "usart.h"

#define DBUS_MAX_LEN 50
#define DBUS_BUFLEN  18
extern uint8_t  dbus_buf[DBUS_BUFLEN];

extern void dbus_uart_init(void);
extern void uart_receive_handler(UART_HandleTypeDef *huart);

#endif
