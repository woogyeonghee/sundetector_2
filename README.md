# 프로젝트명

sundetector

# 프로젝트 팀 구성원 

- 우경희 (개인 프로젝트)

# 프로젝트 개요

- stm32 cortex-m4를 활용한 solar detector 라는 주제로 프로젝트를 진행 하였습니다. 태양광 추적 시스템으로 상하, 좌우 의 pos 값과 cds 센서(평균값)을 aws 서버(브로커 역할)을 통해 데이터를 저장 시킬 수 있는 시스템을 구현하였습니다.


# 개발 환경

- 언어 : C/C++, javascript, json

- os : window

- library : HAL && LL, DFrobot 제공 wifi 관련 라이브러리

- ide : stm32cubeide, vscode

- famework : nodejs

- 통신 프로토콜 : mqtt

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


- data communication & sub func

  - MQTT
  
  - aws

  - node js

- data collection

  -  mongoDB


# 프로젝트 작업 과정

- 프로젝트 주제, 기능, 파트리스트 정하기 ( 1주 )

- main 기능 구현하기 ( 1주 )

- 통신 관련 기능 구현하기 (1 주)

- 데이터 베이스 저장 ( 3일 )

# 외관 

![KakaoTalk_20211211_185343909](https://user-images.githubusercontent.com/88933098/145672292-3a9da356-3d2d-4da4-a3b3-1b70a48dd9e2.jpg)

# 동작영상

### light를 detect 하는 기능 관련 영상 (이미지 클릭후 영상으로 이동)
<br/>

[![Video Label](https://user-images.githubusercontent.com/88933098/145672468-0b19eb3a-15f9-4b47-9024-c06487d4637f.png)](https://youtu.be/SaPicVdNIJQ)

### 데이터 수집 영상
<br/>

[![Video Label](https://user-images.githubusercontent.com/88933098/146226442-611b362b-6959-4efa-a561-e1bfdfdc4839.png)](https://youtu.be/AV951ft0zV4)



# FLOWCHART(main func)

![firmware](https://user-images.githubusercontent.com/88933098/145672908-c09fea04-3f48-4a24-ab5a-a697f307eb43.png)

# board setting

- adc_dma 방식으로


# 코드 리뷰 및 문제 해결 (main func)

## Cal_val


- 문제점 : 동일한 빛을 받을때의 센서값 불정확

- 해결법 : 오차 값이 가장 큰 센서 값 및 종류를 선택후 계산

![cds1](https://user-images.githubusercontent.com/88933098/145672606-58faa529-00c3-43f9-8169-7f0d4c72b46f.png)
~~~
	동일한 빛 센서값 1,000 (cds3,4)
	     - cds1 : 700을 읽음
	
	1. 700보다 클때 : 700*a+b=1000, 700*a+b=4095
	                 y=x*0.9+409.5;
	2. 700보다 작을때 : 700*a = 1000
	                 y=x*1.5;
~~~

![cds2](https://user-images.githubusercontent.com/88933098/145672633-3cdfd881-be52-453e-b9da-f939cb599655.png)

~~~
	동일한 빛 센서값 1,000 (cds3,4)
	    - cds2 : 1300을 읽음
	1. 1300보다 클때 : 1300*a+b=1000, 1300*a+b=4095
	                   y=x*1.1-409;
	2. 1300보다 작을때 : 1300*a = 1000
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
 
 ## USART IRQHandler

- 문제점 : DFRobot 에서 사용한 몇몇 함수들에 implicit declaration of function error 발생

- 원인 : 사용한 함수들이 standard peripheral library ( spl ) 로 이루어져 있음 현재 st사에서 더이상 지원 안함

- 해결 방법

![spl2ll](https://user-images.githubusercontent.com/88933098/145715190-9de035ad-fdcf-4ada-96ee-0dfdc161b194.JPG)



  - spl2ll-converter 사용 ( 관련내용 하단 url 참조 )
  - https://woogyeonghee.github.io/firmware/2020-01-01-stm32_spl2ll.html


# FLOWCHART (sub func)

![DSD](https://user-images.githubusercontent.com/88933098/146223727-c343e079-7d1f-4245-9587-137d57b12319.png)

## WIFI MODULE 

-  모듈 내에서 자체적으로 MQTT 기능이 있어 해당 메뉴얼을 참조하여 구현 하였습니다.
-  링크 : https://iot.dfrobot.com/docs/

## AWS

-  어느 지역이든 와이파이만 연결 되었을 때 실행 가능 할 수 있게 끔 구현하기 위해서 상시 작동되는 서버의 IP, 도메인 주소가 필요 하여 aws를 선택하였습니다..

## MQTT 

- 프로젝트 진행 중에 MQTT 포트를 열어줘야 하여서 MQTT 인바운드 규칙에 1883-1884 포트를 열어주었습니다.
- WIFI 모듈에서 제공 하는 메뉴얼에 따르면 MQTT connect를 위해선 username 및 passwd 가 필요하여 cmd로 테스트 진행시 아래와 같은 명령어 옵션을 사용하였습니다

~~~
//sub
mosquitto_sub -h 3.38.154.143 -t test -u woo -P 12345678

//pub
mosquitto_pub -h 3.38.154.143 -t test -u woo -P 12345678 -m "hello world"
~~~

## NODEJS

- 이역시 구독을 하기 위해 mqtt connect 옵션에 username 과 passwd를 추가 하였습니다.
- string 파일을 json형태로 변환 하였습니다.
- mongoose 라는 패키지를 통해 mongodb와 연동 할 수 있게 하였습니다.

## mongodb

- nodejs에서 데이터를 받도록 하였습니다.

