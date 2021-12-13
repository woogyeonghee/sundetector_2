#ifndef _DFROBOT_WIFI_IOT_H
#define _DFROBOT_WIFI_IOT_H
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "DFRobot_queue.h"
#include "Uart_ring_buffer.h"


void Usart_SendString(USART_TypeDef*USARTx, char *str);

void mqtt(const char* url, const char *port, const char *iotid, const char *iotpwd , const char *topic);

void loop(void);

void publish(const char *topic,const char *masag);

void splitString(void);

void connectWifi(const char *ssid , const char *pwd);

#endif
