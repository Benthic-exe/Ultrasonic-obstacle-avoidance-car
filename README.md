# Ultrasonic obstacle avoidance car
## 超声波避障小车

### 1. SG90舵机

#### 接线方法：

红色	VCC
棕色	GND
橙色	信号线

#### 原理

> 调整信号线的PWM，实现各个角度转动
>
> 高电平t占整个周期T（20ms）的时间
> 舵机旋转的角度
> 0.5ms ---------> 0度
> 1ms ---------> 45度
> 1.5ms ---------> 90度
> 2ms ---------> 135度
> 2.5ms ---------> 180度

#### 参考代码

```c
#include <REG52.H>

typedef unsigned char uchar;   //0~255
typedef unsigned int uint;     //0~65535

uchar Turn_angle;
sbit PWM = P3^5;  //信号线
uchar kk = 0;

//延迟1ms
void delay_ms(uint c)   //误差 0us
{
    uint a,b;
    for(;c>0;c--)
        for(b=102;b>0;b--)
            for(a=3;a>0;a--);
}

void Time0_Init()          //定时器初始化
{
        TMOD = 0x01;           //定时器0工作在方式1
        EA = 1;
        ET0 = 1;
        TH0  = 0xfe;
        TL0  = 0x33;                   //11.0592MZ晶振，0.5ms
        TR0=1;                 //定时器开始
}

void main()
{
    Time0_Init();
   //PWM = 0;

    while (1)
    {
        kk = 0;
        Turn_angle = 1; //归零   1--0 2--45 3--90 4--135 5--180
        delay_ms(1000);
        kk = 0;
        Turn_angle = 5; //旋转
        delay_ms(1000);
    }
}

void timer0() interrupt 1
{
    TR0 = 0;  //关闭计时
    TH0  = 0xfe;
    TL0  = 0x33;                   //11.0592MZ晶振，0.5ms

    if(kk<=Turn_angle)   //1---0度  2---45度 3---90 4--- 135度  5---180度
    {
        PWM = 1;
    }
    else
    {
        PWM = 0;
    }
    if(kk==40)   //一个周期20ms
    {
        kk = 0;
    }
    kk ++;
    TR0 = 1;  //开启计时
}

```



#### 问题总结

1.测试程序时要测试每一个角度，可能会出现错误
2.错误一般出现在定时器上，PWM的占空比出错

### 2. HC-SR04

#### 1.模块简介

HC-SR04超声波模块常用于机器人避障、物体测距、液位检测、公共安防、停车场检测等场所。HC-SR04超声波模块主要是由两个通用的压电陶瓷超声传感器，并加外围信号处理电路构成的。如图：

![20191027150845140](C:\Users\Benthic\OneDrive\图片\20191027150845140.png)

两个压电陶瓷超声传感器，一个用于发出超声波信号，一个用于接收反射回来的超声波信号。由于发出信号和接收信号都比较微弱，所以需要通过外围信号放大器提高发出信号的功率，和将反射回来信号进行放大，以能更稳定地将信号传输给单片机。模块整体电路如图：

![20191027150941793](C:\Users\Benthic\OneDrive\图片\20191027150941793.png)

#### 2.模块参数

##### （1）模块主要电气参数

​	使用电压：DC—5V
​	静态电流：小于2mA
​	电平输出：高5V
​	电平输出：底0V
​	感应角度：不大于15度
​	探测距离：2cm-450cm
​	高精度 可达0.2cm

##### （2）模块引脚

​	超声波模块有4个引脚，分别为Vcc、 Trig（控制端）、 Echo（接收端）、 GND；其中VCC、GND接上5V电源， Trig（控制端）控制发出的超声波信号，Echo（接收端）接收反射回来的超声波信号。模块如图：

![20191027151225210](C:\Users\Benthic\OneDrive\图片\20191027151225210.jpg)

与单片机的连接如图：

![20191027151145133](C:\Users\Benthic\OneDrive\图片\20191027151145133.png)

控制原理：通过Trig引脚发一个 10US 以上的高电平,就可以在Echo接收口等待高电平输出；一有输出就可以开定时器计时,当此口变为低电平时就可以读定时器的值,此时就为此次测距的时间,方可算出距离.如此不断的周期测,就可以达到你移动测量的值了。

#### 3.控制程序

##### （1）工作流程

a.单片机引脚触发Trig测距，给至少 10us 的高电平信号;
b.模块自动发送 8 个 40khz 的方波，自动检测是否有信号返回；
c.有信号返回，通过 IO 输出一高电平，并单片机定时器计算高电平持续的时间;
d.超声波从发射到返回的时间．
计算公式：测试距离=(高电平时间*声速(340M/S))/2;

整个控制时序如图：

![20191027151401643](C:\Users\Benthic\OneDrive\图片\20191027151401643.png)

##### （2）驱动程序

###### 1.单片机引脚触发Trig测距，给至少 10us 的高电平信号;

```c
void  StartModule() 		         //启动模块
{
	  TX=1;			                     //启动一次模块
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_();
	  _nop_(); 
	  _nop_(); 
	  _nop_(); 
	  _nop_();
	  TX=0;
}
```

###### 2.自动检测是否有信号返回,如果有，启动定时器；

```c
while(!RX);		//当RX为零时等待
TR0=1;			    //开启计数
while(RX);			//当RX为1计数并等待
TR0=0;				//关闭计数
Conut();			//计算

```

###### 3.定时器计算高电平持续的时间，并计算出距离；

```c
 void Conut(void)
{
	 time=TH0*256+TL0;
	 TH0=0;
	 TL0=0;	
	 S=(time*1.7)/100;     //算出来是CM
}
```

### 3. 超声波避障小车

![20200403201322229](C:\Users\Benthic\OneDrive\图片\20200403201322229.png)

```c
#include <at89x51.h> 
#include <intrins.h
#define  TX  P1_3
#define  RX  P1_2
#define Forward_L_DATA  180 //当前进不能走直线时，调节这两个参数，理想是100，100，最大时256，最小是0.
#define Forward_R_DATA  180
sbit L293D_IN1=P0^0; 
sbit L293D_IN2=P0^1;
sbit L293D_IN3=P0^2;
sbit L293D_IN4=P0^3;
sbit L293D_EN1=P0^4;
sbit L293D_EN2=P0^5;
void Delay400Ms(void);//延时400毫秒函数
unsigned char disbuff[4]={0,0,0,0};//用于分别存放距离的值0.1mm,mm,cm,m
void Count(void);//距离计算函数
unsigned int  time=0;//用于存放定时器的时间值
unsigned long S=0;//用于存放距离的值
bit  flag =0;//量程溢出标志位
bit  turn_right_flag;
void Delay1ms(unsigned int i) 
{ 
unsigned char j,k; 
do{ 
  j = 10; 
  do{ 
   k = 50; 
   do{ 
    _nop_(); 
   }while(--k);     
  }while(--j); 
}while(--i); 
} 
void Delay10us(unsigned char i) 
{ 
   unsigned char j; 
do{ 
  j = 10; 
  do{ 
   _nop_(); 
   }while(--j); 
}while(--i); 
}
void Forward()//前进
{
  L293D_IN1=1; 
  L293D_IN2=0;
  L293D_IN3=1;
  L293D_IN4=0;
}
void Stop(void)//刹车
{
  L293D_IN1=0; 
  L293D_IN2=0;
  L293D_IN3=0;
  L293D_IN4=0;
}
void Turn_Retreat()//后
{
 L293D_IN1=0; 
 L293D_IN2=1;
 L293D_IN3=0;
 L293D_IN4=1;
}
void Turn_left()//左
{
 L293D_IN1=0; 
 L293D_IN2=1;
 L293D_IN3=1;
 L293D_IN4=0;
}
void Conut(void)//计算距离
 {
  time=TH1*256+TL1;
  TH1=0;
  TL1=0;
  S=time*2;
  S=S*0.17;
  if(S<=300)
  {
  if(turn_right_flag!=1)
  {
      Stop();
      Delay1ms(5);
  }
  turn_right_flag=1;
  P1_7=0;
  P2_0=0;
  P0_6=0;
  Delay1ms(10);
  P1_7=1;
  P2_0=1;
  P0_6=1;
  Delay1ms(5);
  Turn_left();
  Delay1ms(10);
 }
 else
 {
 turn_right_flag=0;
 Forward();
 }
 if((S>=5000)||flag==1)//超出测量范围
 {
 flag=0;
 }
 else
  {
   disbuff[0]=S%10;
   disbuff[1]=S/10%10;
   disbuff[2]=S/100%10;
   disbuff[3]=S/1000;
  }
 }
 void zd0() interrupt 3//T0中断用来计数器溢出，超过测距范围
 {
 flag=1;
 RX=0;
 }
 void Timer_Count(void)
 {
 TR1=1;//开启计数
 while(RX);//当RX为1计数并等待
 TR1=0;//关闭计数
 Conut();//计算
 }
 void  StartModule()//启动模块
 {
 TX=1;//启动一次模块
 Delay10us(2);
 TX=0;
 }
 void main(void)
 {
 unsigned char i;
 unsigned int a;
 Delay1ms(400);
 Delay1ms(5);
 TMOD=TMOD|0x10;
    EA=1;
    TH1=0;
    TL1=0;          
    ET1=1;
    turn_right_flag=0;
 B:  for(i=0;i<50;i++)//判断k3是否按下
 {
 Delay1ms(1);
 if(P3_2!=0 )
 goto B;
 }
while(1)
   {
  RX=1;
     StartModule();
        for(a=951;a>0;a--)
     {
     
        if(RX==1)
     {
           Timer_Count();
     }
      }
    }
}
```

