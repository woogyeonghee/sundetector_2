#ifndef _DFROBOT_WIFI_IOT_H
#define _DFROBOT_WIFI_IOT_H
#include "usart.h"
#include "system.h"
#include "SysTick.h"


void mqtt(const char* url, const char *port, const char *iotid, const char *iotpwd , const char *topic);

void loop(void);

void publish(const char *topic,const char *masag);

void IFTTTSendMasage(const char *data1, const char *data2, const char *data3);

void thingSpeakSendMasage(const char *data1, const char *data2);

void splitString(void);

void connectWifi(const char *ssid , const char *pwd);

void configIFTTT(const char *event , const char *key);

void configThingSpeak(const char *key);

#endif
