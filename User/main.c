#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "LED.h"
#include "motor.h"
#include "path_planner.h"

uint8_t KeyNum;			//定义用于接收按键键码的变量

u8 dir=1;
u16 x=2250,y=2250;

/*
	1
	
2 		 4

	3
*/
#define ABS(x) ((x) < 0 ? -(x) : (x))
u8 n=0;

void judge(void){
	u8 judge1=0,judge2=0,judge3=0,judge4=0;
	while(1){
		if(judge1==0){
			Serial_SendByte(0x01);
			Serial_SendByte(0x3A);
			Serial_SendByte(0x6B);
			if(Serial_GetRxFlag()){
				if(Serial_RxPacket[1]==0x03){//到达
					judge1=1;
				}
			}
		}
		Delay_ms(5);
		if(judge2==0){
			Serial_SendByte(0x02);
			Serial_SendByte(0x3A);
			Serial_SendByte(0x6B);
			if(Serial_GetRxFlag()){
				if(Serial_RxPacket[1]==0x03){//到达
					judge2=1;
				}
			}
		}
		Delay_ms(5);
		if(judge3==0){
			Serial_SendByte(0x03);
			Serial_SendByte(0x3A);
			Serial_SendByte(0x6B);
			if(Serial_GetRxFlag()){
				if(Serial_RxPacket[1]==0x03){//到达
					judge3=1;
				}
			}
		}
		Delay_ms(5);
		if(judge4==0){
			Serial_SendByte(0x04);
			Serial_SendByte(0x3A);
			Serial_SendByte(0x6B);
			if(Serial_GetRxFlag()){
				if(Serial_RxPacket[1]==0x03){//到达
					judge4=1;
				}
			}
		}
		Delay_ms(5);
		if(judge1==1&&judge2==1&&judge3==1&&judge4==1){
			return;
		}
		OLED_ShowNum(2,1,n++,5);
	}
}
void translation(u16 x0,u16 y0,u16 x1,u16 y1){//0起点 1终点
	if(x1==x0&&y1>y0){//向上
		if(dir==2){//朝左
			shun_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=1;
		}
		if(dir==4){//朝右
			ni_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=1;
		}
		if(dir==3){
			houtui_(ABS(y1-y0));
		}else{
			qianjin_(ABS(y1-y0));
		}
	}else if(x1==x0&&y1<y0){//向下
		if(dir==2){//朝左
			ni_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=3;
		}
		if(dir==4){//朝右
			shun_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=3;
		}
		if(dir==3){
			qianjin_(ABS(y1-y0));
		}else{
//			OLED_ShowChar(4,1,'3');
			houtui_(ABS(y1-y0));
//			OLED_ShowChar(4,2,'3');
		}
	}else if(x1<x0&&y0==y1){//向左
		if(dir==1){//朝上
			ni_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=2;
		}else if(dir==3){//朝下
			shun_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=2;
		}
		if(dir==2){
			qianjin_(ABS(x1-x0));
		}else{
			houtui_(ABS(x1-x0));
		}
	}else if(x1>x0&&y0==y1){//向右
		if(dir==1){//朝上
			shun_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=4;
		}
		if(dir==3){//朝下
			ni_yaw_(90);
			Delay_ms(1000);
			judge();
			dir=4;
		}
		if(dir==4){
			qianjin_(ABS(x1-x0));
		}else{
			OLED_ShowChar(1,1,'c');
			houtui_(ABS(x1-x0));
		}
	}
	judge();
}
int main(void)
{
	LED_Init();//PA1,2
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Key_Init();			//按键初始化
	Serial_Init();		//串口初始化
	
	Delay_ms(1000);
//	shun_yaw_(90);
//	OLED_ShowChar(1,1,'z');
	translation(2250,2250,1200,2250);
	Delay_ms(3000);
	translation(2250,2250,2250,1200);
	while(1){
	
	}
	u16 xpath[16];
	u16 ypath[16];
	path_plan(2250,2250,1200,150,16,xpath,ypath);
	for(int i=0;i<key_point_num;i++){
		OLED_ShowNum(i+1,1,xpath[i],4);
		OLED_ShowNum(i+1,6,ypath[i],4);
	}
//	OLED_ShowNum(1,1,key_point_num,2);
	while(1){}
	
	OLED_ShowChar(1,1,'z');
	Delay_ms(1000);
	
	qianjin_(1200);
	Delay_ms(3000);
	houtui_(1200);
	while(1){}
	
	
	u16 x0=150,y0=2250;
		
	

	
	
	while(1){}
	//标定：最早麦轮10000脉冲79cm，12658脉冲100cm
//	qianjin_(600);
//	Delay_ms(4000);
//	houtui_(600);
//	Delay_ms(4000);
	shun_yaw_(90);
	Delay_ms(2000);
	shun_yaw_(90);
	Delay_ms(2000);
	shun_yaw_(90);
	Delay_ms(2000);
	shun_yaw_(90);
	Delay_ms(2000);
//	zuo_yaw_(90);
//	Delay_ms(4000);
//	zuo_(100);
//	Delay_ms(4000);
//	you_(100);
//	Delay_ms(3000);
	
	while(1){
		
	}
	
	while(1){
		qianjin_(80);
		Delay_ms(3000);
		zuo_(80);
		Delay_ms(3000);
		houtui_(80);
		Delay_ms(3000);
		you_(80);
		Delay_ms(3000);
	}

	/*显示静态字符串*/
	OLED_ShowString(1, 1, "TxPacket");
	OLED_ShowString(3, 1, "RxPacket");
	
	/*设置发送数据包数组的初始值，用于测试*/
	Serial_TxPacket[0] = 0x01;
	Serial_TxPacket[1] = 0x02;
	Serial_TxPacket[2] = 0x03;
	Serial_TxPacket[3] = 0x04;
	
	while (1)
	{
		KeyNum = Key_GetNum();			//获取按键键码
		if (KeyNum == 1)				//按键1按下
		{
			Serial_TxPacket[0] ++;		//测试数据自增
			Serial_TxPacket[1] ++;
			Serial_TxPacket[2] ++;
			Serial_TxPacket[3] ++;
			
			Serial_SendPacket();		//串口发送数据包Serial_TxPacket
			
			OLED_ShowHexNum(2, 1, Serial_TxPacket[0], 2);	//显示发送的数据包
			OLED_ShowHexNum(2, 4, Serial_TxPacket[1], 2);
			OLED_ShowHexNum(2, 7, Serial_TxPacket[2], 2);
			OLED_ShowHexNum(2, 10, Serial_TxPacket[3], 2);
		}
		
		if (Serial_GetRxFlag() == 1)	//如果接收到数据包
		{
			OLED_ShowHexNum(4, 1, Serial_RxPacket[0], 2);	//显示接收的数据包
			OLED_ShowHexNum(4, 4, Serial_RxPacket[1], 2);
			OLED_ShowHexNum(4, 7, Serial_RxPacket[2], 2);
			OLED_ShowHexNum(4, 10, Serial_RxPacket[3], 2);
		}
	}
}
