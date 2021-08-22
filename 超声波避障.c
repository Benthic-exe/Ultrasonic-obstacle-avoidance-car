#include<reg52.h>
typedef unsigned int u16;
typedef char u8;

sbit Trig = P1^6;				//超声波发射引脚
sbit Echo = P1^7;				//超声波接受引脚
sbit led=P2^0;					//led指示灯
sbit PWM  = P2^1;					//为舵机转向提供脉冲
sbit motorDriver_1 =P2^2; 			//分别接l298n的四个引脚
sbit motorDriver_2 =P2^3; 
sbit motorDriver_3 =P2^4;  
sbit motorDriver_4 =P2^5; 
u16 timer1; 
u16 s;	
u16 time; 
u16 sleft,sright,count=0,value=0;



//小车停止
void stop(){	
	motorDriver_1=0;
	motorDriver_2=0;
	motorDriver_3=0;
	motorDriver_4=0;
}
//小车左转
void turnleft(){		
	motorDriver_1=1;
	motorDriver_2=0;
	motorDriver_3=0;
	motorDriver_4=0;
}
//小车右转						
void turnright(){
	motorDriver_1=0;
	motorDriver_2=0;
	motorDriver_3=1;
	motorDriver_4=0;
}
//小车前进
void turnup(){			  
	motorDriver_1=1;
	motorDriver_2=0;
	motorDriver_3=1;
	motorDriver_4=0;
}
//小车退后
void turndown(){		 
	motorDriver_1=0;
	motorDriver_2=1;
	motorDriver_3=0;
	motorDriver_4=1;
}

//延时函数1，用于启动超声波
void delay1(u16 i)
{
	while(i--);	
}

	
//延时函数2
void delay(u16 i)
{
	u16 j=0;
		for(;i>0;i--)
			{
					for(;j<10;j++);
			}
}

void Timer1Init()						//定时器0的初始化函数，启用计数模式
{
	TMOD|=0X10;
	TH1 = 0xfe; 
	TL1 = 0x33;
	ET1=1;
	EA=1;
	TR1=1;	
}




u16 getdistance()					//超声波获取距离函数

{
		TMOD=0x01;
		TH0=0;
		TL0=0;
		TR0=0;
		

		Trig=1;
		delay1(2);
		Trig=0; 
		
		while(!Echo);
		TR0=1;
		while(Echo);
		TR0=0;
		time=TH0*256+TL0;
		s=(time*1.7)/100;
		TH0=0;
		TL0=0;
		if(s<=16)			//设定障碍距离是16cm，距离小于16则函数返回值为1表示前						//方有障碍物，否则返回值为0
		{
		return 1;
		}
		else
		{
		return 0;
		}

}





void main()						//主函数
{
	stop();
	Timer1Init();
	led=1;
	turnUp();
	
		
	while(1) 
		{
			
		  do{
			turnup();
			}while(getdistance==0);				//在没有障碍物时执行循环不断前进
			
			
			
			Timer1Init();
		if(getdistance()==1)						//遇到障碍物，判断有障碍物的方向并进											//行转向
			{	
				stop();
				delay(20);
				Timer1Init();
				
				
				value=1;				
				delay(5000);	
				TR1=0;
				sleft=getdistance();				//判断左边有没有障碍物
				TR1=1;
				delay(5000);	
				Timer1Init();
				value=3;
				delay(5000);


				
				value=5;		
				delay(5000);	                        
				TR1=0;
				sright=getdistance();			//判断右边是否有障碍物
				TR1=1;	
				delay(5000);
				
				
				Timer1Init();
				value=3;
				delay(5000);
			
			
			if(sleft==0&&sright==1)          //根据障碍物情况进行转向
                                        //以下所有的delay函数用于调节转向的幅度
			{
				
			led=0;
			turndown();
			delay(5000);
			turnleft();
			delay(30000);
			}
			else if(sleft==1&&sright==0)
			{
			led=1;
			turndown();
			delay(5000);                          
			turnright();
			delay(30000);			
			}
			if(sleft==1&&sright==1)					//若是左右都有障碍物，默认先退后												//然后左转
			{
			led=0;
			turndown();
			delay(20000);
			turnleft();
			delay(30000);
			}
		  else if(sleft==0&&sright==0)				//左右都没有障碍物则默认右转
			{
			turnright();
			delay(30000);
			}
		}
		if(getdistance==0)
		{Timer1Init();
				value=3;
				delay(5000);
		} 
			turnup();
			}	
}
	
void Time1(void) interrupt 3   			//舵机脉冲中断
{	
  count++;
	TH1 = 0xfe; 						//每次进入中断的时间为0.5毫秒，进入40次为一										//个周期
	TL1 = 0x33;
	
	if(count<=value)
	{
	PWM=1;
	}
	
	else
	{
	PWM=0;
	}
	
	if(count==40)
	{
		count=0;
	}
	
}
