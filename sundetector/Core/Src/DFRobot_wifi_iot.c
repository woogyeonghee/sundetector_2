#include "DFRobot_wifi_iot.h"
#include <string.h>

char *splitdata[5];

char*iftttEvent;
char*iftttkey;
char* thingSpeakkey;

void mqtt(const char* url, const char *port, const char *iotid, const char *iotpwd , const char *topic)
{
	u8 state = 1;
	u8 i;
	
	char _mqtt[50];
	char _topic[50];
	strcat(_mqtt,"|4|1|1|");
	strcat(_mqtt,url);
	strcat(_mqtt,"|");
	strcat(_mqtt,port);
	strcat(_mqtt,"|");
	strcat(_mqtt,iotid);
	strcat(_mqtt,"|");
	strcat(_mqtt,iotpwd);
	strcat(_mqtt,"|\\r");
	printf("%s\n",_mqtt);
	Usart_SendStriong(USART3, _mqtt);
	memset(_mqtt,'\0',50);
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("1",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
				}else{
					printf(".");
				}
			}
		}else{
			printf(".");
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		delay_ms(100);
	}
	printf("MQTT Connect SUCCESS\n");
	state=1;
	strcat(_topic,"|4|1|2|");
	strcat(_topic,topic);
	strcat(_topic,"|\r");
	printf("%s\n",_topic);
	Usart_SendStriong(USART3, _topic);
	memset(_topic,'\0',50);
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("2",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
				}else if(strcmp("1",splitdata[4]) == 0){
					printf("Subscription limit reached");
				}else if(strcmp("2",splitdata[4]) != 0){
					printf("Subscription failed");
				}else{
					printf(".");
				}
		}else{
			printf(".");
			}
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		delay_ms(100);
	}
	printf("Subscribe Topic SUCCESS\n");
}

void publish(const char *topic,const char *masag)
{
	u8 state = 1;
	u8 i,j=0;
	char _data[50];
	strcat(_data,"|4|1|3|");
	strcat(_data,topic);
	strcat(_data,"|");
	strcat(_data,masag);
	strcat(_data,"|\r");
	printf("%s\n",_data);
	Usart_SendStriong(USART3, _data);
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("3",splitdata[2]) == 0){
				if(strcmp("1",splitdata[3]) == 0){
					state=0;
					printf("Successfully sent\n");
				}else{
					printf("Failed to send message\n");
				}
			}
		}else if(j == (u8)100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		delay_ms(100);
	}
	memset(_data,'\0',50);
}


void loop(void)
{
	u8 state = 1;
	u8 i,j=0;
	while(state == 1){
		splitString();
		if(strcmp("4",splitdata[0]) == 0){
			if(strcmp("5",splitdata[2]) == 0){
				state=0;
				printf("topic:%s\n",splitdata[3]);
				printf("massage:%s\n",splitdata[4]);
			}
		}else if(strcmp("3",splitdata[0]) == 0){
			if(strcmp("200",splitdata[1]) == 0){
				state=0;
				printf("Sent successfully\n");
				printf("massage:%s\n",splitdata[2]);
			}else if(strcmp("-1",splitdata[1]) == 0){
				printf("-1");
			}else if(strcmp("1",splitdata[1]) == 0){
				printf("1");
			}
		}else if(j == (u8)100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		delay_ms(100);
	}
}

void IFTTTSendMasage(const char *data1, const char *data2, const char *data3)
{
	char _http[200] = "";
	strcat(_http,"|3|2|http://maker.ifttt.com/trigger/");
	strcat(_http,iftttEvent);
	strcat(_http,"/with/key/");
	strcat(_http,iftttkey);
	strcat(_http,",");
	strcat(_http,"{\"value1\":\"");
	strcat(_http,data1);
	strcat(_http,"\",\"value2\":\"");
	strcat(_http,data2);
	strcat(_http,"\",\"value3\":\"");
	strcat(_http,data3);
	strcat(_http,"\" }\r|\r");
	printf("%s\n",_http);
	Usart_SendStriong(USART3, _http);
	memset(_http,'\0',200);
}


void thingSpeakSendMasage(const char *data1, const char *data2)
{
	char _http[200] = "";
	
	strcat(_http,"|3|1|http://api.thingspeak.com/update?api_key=");
	strcat(_http,thingSpeakkey);
	strcat(_http,"&field1=");
	strcat(_http,data1);
	strcat(_http,"&field2=");
	strcat(_http,data2);
	strcat(_http,"\r\0|\r");
	Usart_SendStriong(USART3, _http);
	memset(_http,'\0',200);
}

void splitString(void){
	u8 count = 0;
	static struct sQueueData *p = NULL;
	p = cuappDequeue();
	if(p != NULL){
		splitdata[count] = strtok((char*)p->data, "|");
		while(splitdata[count])
		{
			splitdata[++count] = strtok(NULL, "|");
		}
	}
	free(p);
}

void connectWifi(const char *ssid , const char *pwd){
char _wifi[100] = "";
	u8 state = 1;
	u8 i;
	
	strcat(_wifi,"|2|1|");
	strcat(_wifi,ssid);
	strcat(_wifi,",");
	strcat(_wifi,pwd);
	strcat(_wifi,"|\r");
	printf("%s\n",_wifi);
	Usart_SendStriong(USART3, _wifi);
	memset(_wifi,'\0',50);
	while(state == 1){
		splitString();
		if(strcmp("2",splitdata[0]) == 0){
			if(strcmp("3",splitdata[1]) == 0){
				state=0;
			}else{
				printf(".");
			}
		}else{
		printf(".");
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		delay_ms(100);
	}
	printf("Wifi Connect SUCCESS\n");
}

void configIFTTT(const char *event , const char *key){
	iftttEvent = (char*)event;
	iftttkey   = (char*)key;
}
void configThingSpeak(const char *key){
	thingSpeakkey = (char*)key;
}
