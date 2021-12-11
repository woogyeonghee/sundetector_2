# 프로젝트명

sundetector

# 프로젝트 팀 구성원 

- 우경희 (개인 프로젝트)

# 프로젝트 개요

- stm32 cortex-m4를 활용한 solar detector 라는 주제로 프로젝트를 진행 하였습니다

- 4개의 조도센서와 2개의 서보모터를 활용하여 가장 빛을 따라 갈 수 있도록 구현 하였습니다

- uart - wifi 를 통해 데이터를 전송할 예정입니다. (진행중)

- 수집한 데이터의 원활한 처리를 위해 MQTT의 프로토콜을 사용 예정입니다. (진행예정)

- MQTT - nodejs - mongodb 데이터를 저장할 예정입니다. (진행예정)

- 추가적으로 전압센서 모듈을 통해 배터리 충전율의 데이터를 logging 할 예정입니다.(진행예정)

# 개발 환경

- 언어 : C/C++, javascript, json

- os : window

- library : HAL && LL, DFrobot 제공 wifi 관련 라이브러리

- ide : stm32cubeide, vscode


# HW 구성

- PC

- NUCLEO l476RG

- OBLOQ IOT module

- sensor & actuator
  
  - control servo * 2
    
  - cds sensor *4

  - solar pad

# SW 구성

- project main func ( in STM32 )

  - board setting

  - wifi setting

  - MQTT setting

  - Cal_val

  - UART ring buffer

  - sendmsg per 1 sec


- data communication sub func

  - MQTT

  - node js

- data collection

  -  mongoDB


# 프로젝트 작업 과정

- 프로젝트 주제, 기능, 파트리스트 정하기 ( 1주 )

- main 기능 구현하기 ( 1주 )

- 통신 관련 기능 구현하기 (예정)

- 데이터 베이스 저장 (예정)

- 전압 측정-> 배터리 충전율 구현 (예정)

# 외관 

![KakaoTalk_20211211_185343909](https://user-images.githubusercontent.com/88933098/145672292-3a9da356-3d2d-4da4-a3b3-1b70a48dd9e2.jpg)

# 동작영상

### light를 detect 하는 기능 관련 영상 (이미지 클릭후 영상으로 이동)
<br/>

[![Video Label](https://user-images.githubusercontent.com/88933098/145672468-0b19eb3a-15f9-4b47-9024-c06487d4637f.png)](https://youtu.be/SaPicVdNIJQ)

### 데이터 수집 영상 (예정)
<br/>




# FLOWCHART(main func)



# board setting

- adc_dma 방식으로


# 코드 리뷰 및 문제 해결 (main func)

## Cal_val


- 문제점 : 동일한 빛을 받을때의 센서값 불정확

- 해결법 : 오차 값이 가장 큰 센서 값 및 종류를 선택후 계산

![cds1](https://user-images.githubusercontent.com/88933098/145672606-58faa529-00c3-43f9-8169-7f0d4c72b46f.png)
~~~
	동일한 빛 센서값 1,000 (cds3,4)
	cds1
	700보다 클때 : 700*a+b=1000, 700*a+b=4095
	y=x*0.9+409.5;
	700보다 작을때 : 700*a = 1000
	y=x*1.5;
~~~

![cds2](https://user-images.githubusercontent.com/88933098/145672633-3cdfd881-be52-453e-b9da-f939cb599655.png)

~~~
	동일한 빛 센서값 1,000 (cds3,4)
	cds2
	1300보다 클때 : 1300*a+b=1000, 1300*a+b=4095
	y=x*1.1-409;
	1300보다 작을때 : 1300*a = 1000
	y=x*0.8;
~~~
~~~
//adc_cds1
uint16_t Sensor_cal2(uint16_t x){
	if(x>700)
	{
		return x*0.9+409.5;
	}
	else
	{
		return x*1.5;
	}
}
//adc_cds2
uint16_t Sensor_cal1(uint16_t x){
	if(x>1300)
	{
		return x*1.1-409;
	}
	else
	{
		return x*0.8;
	}
}
 ~~~
 
 ## UART ring buffer

- 문제점 : DFRobot 에서 사용한 몇몇 함수들에 implicit declaration of function error 발생

- 원인 : 사용한 함수들이 standard peripheral library ( spl ) 로 이루어져 있음 현재 st사에서 더이상 지원 안함

- 해결 방법
  - spl2ll-converter 사용 ( 관련내용 하단 url 참조 )

  - https://woogyeonghee.github.io/firmware/2020-01-01-stm32_spl2ll.html

## sendmsg per 1 sec

- 문제점 : while문 속 코드들의 처리 속도는 빨라야 한 반면에 data logging 은 1초에 1번씩만 보내도록 처리해야함

- 해결방법
  - timer interupt callback 함수를 구현


~~~
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	gTimerCnt++;
	//same as HAL_Delay(1000) == 1 sec
	if(gTimerCnt == 1000)
	{
		gTimerCnt = 0;
		printf ("Lf_Rt_pos=%4d, Tp_Dn_pos=%4d, ", Lf_Rt_pos,Tp_Dn_pos);
		printf ("CDS1=%4d, CDS2=%4d, ", uwADCxConvertedValue[0],uwADCxConvertedValue[1]);
		printf ("CDS3=%4d, CDS4=%4d\n", uwADCxConvertedValue[2],uwADCxConvertedValue[3]);

	}
}
~~~

