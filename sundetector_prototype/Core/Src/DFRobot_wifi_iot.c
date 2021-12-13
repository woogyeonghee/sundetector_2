#include "DFRobot_wifi_iot.h"
#include <string.h>

char *splitdata[5];

void Usart_SendString1(USART_TypeDef* USARTx, char *str)
{
	while(*str)
	{
		while(!LL_USART_IsActiveFlag_TXE(USARTx));//判断是否可以发送
		LL_USART_TransmitData9(USARTx, *str);
		while(LL_USART_IsActiveFlag_TC(USARTx));
		str++;
	}
}



void mqtt(const char* url, const char *port, const char *iotid, const char *iotpwd , const char *topic)
{
	uint8_t state = 1;
	uint8_t i;
	
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
	strcat(_mqtt,"|\r");
	printf("%s\n",_mqtt);
	Usart_SendString1(USART3, _mqtt);
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
		HAL_Delay(100);
	}
	printf("MQTT Connect SUCCESS\n");

	state=1;
	strcat(_topic,"|4|1|2|");
	strcat(_topic,topic);
	strcat(_topic,"|\r");
	printf("%s\n",_topic);
	Usart_SendString1(USART3, _topic);
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
		HAL_Delay(100);
	}
	printf("Subscribe Topic SUCCESS\n");

}

void publish(const char *topic,const char *masag)
{
	uint8_t state = 1;
	uint8_t i,j=0;
	char _data[100];
	memset(_data,'\0',100);
	printf("\n");
	HAL_Delay(100);
	strcat(_data,"|4|1|3|");
	strcat(_data,topic);
	strcat(_data,"|");
	strcat(_data,masag);
	strcat(_data,"|\r");
	printf("%s\n",_data);
	Usart_SendString1(USART3, _data);
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
		}else if(j == 100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		HAL_Delay(100);
	}
	memset(_data,'\0',100);
}


void splitString(void){
	uint8_t count = 0;
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

void loop(void)
{
	uint8_t state = 1;
	uint8_t i,j=0;
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
		}else if(j == (uint8_t)100){
			printf("time out");
			state = 0;
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		j++;
		HAL_Delay(100);
	}
}




void connectWifi(const char *ssid , const char *pwd)
{
	char _wifi[100] = "";
	uint8_t state = 1;
	uint8_t i;
	
	strcat(_wifi,"|2|1|");
	strcat(_wifi,ssid);
	strcat(_wifi,",");
	strcat(_wifi,pwd);
	strcat(_wifi,"|\r");
	printf("%s\n",_wifi);
	Usart_SendString1(USART3, _wifi);
	memset(_wifi,'\0',50);

	while(state == 1){
		splitString();
		//printf("%c,%c,%c,%c,%c\n",*(splitdata[0]),*(splitdata[1]),*(splitdata[2]),*(splitdata[3]),*(splitdata[4]));
		if(strcmp("2",splitdata[0]) == 0){
			if(strcmp("3",splitdata[1]) == 0){
				state=0;
			}else{
				printf(".");
			}
		}else{
		printf(".");
		HAL_Delay(100);
		}
		for(i=0; i<5; i++){
			splitdata[i] = "\0";
		}
		HAL_Delay(100);
	}
	printf("Wifi Connect SUCCESS\n");
}

