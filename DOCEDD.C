/* ----------------Design by Tian Xiangyi with C Code---------------- */
//	室内环境质量检测装置设计
//  STC12C5A16S2作为主控芯片，时钟11.0592MHz
//  LCD1602显示检测结果。				   
/* ------------------------------------------------------------------ */ 
#include "stc12c5a16s2.h"
#include <intrins.h> 
#include <math.h>

typedef unsigned char BYTE;
typedef bit BOOL;
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;

#define   ADC_POWER     0x80               //ADC power control bit
#define   ADC_FLAG      0x10               //ADC complete flag
#define   ADC_START    0x08               //ADC start control bit
#define   ADC_SPEEDLL   0x00               //540 clocks
#define   ADC_SPEEDL    0x20               //360 clocks
#define   ADC_SPEEDH   0x40               //180 clocks
#define   ADC_SPEEDHH   0x60               //90 clocks

uint i,cdisplay,ddisplay;
uchar flag,flag_r,check,StrData;
double c0,c1,c2,c3,c4,d;
uchar show_dat[4]={"0""0""0""0"};
uchar read_dat[5]={"0""0""0""0""0"};
uchar conv_dat[2]={"0""0"};
uint count30s=0; 
ulong totalsampletime = 0;
ulong lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0; 
sbit DHT11 = P2^0;
sbit LCD_RS = P2^4;             
sbit LCD_RW = P2^5;
sbit LCD_EP = P2^6;
sbit LED = P1^5;
sbit beep = P2^1;
/*函数功能: 函数声明*/
void initiate_RS232 (void);         
void Send_Byte(uchar one_byte);     
void Init_StcADC(uchar channel);
unsigned int ReadADCResult(uchar channel);
void Init_DHTRead();
void JiaQuan_Show();
void PM_Show();
void Read_Data();
void WenShi_Show();
void Dis_collection();
/*函数功能: 延时1ms子程序*/
void Delay1ms(uint ms)		 // 延时1ms子程序
{ 	while(ms--)
	{
		for(i = 0; i< 250; i++)
		{
			_nop_();			_nop_();			_nop_();			_nop_();
		}
	}
}
/*函数功能: 延时1us子程序*/  
void Delay5us(uint us)			// 延时1us子程序
{
	while(us--)
	{
		_nop_();		_nop_();		_nop_();		_nop_();		_nop_();
	}  
}	 
/*函数功能: LCD忙碌检测*/
BOOL lcd_bz()
{                          // 测试LCD忙碌状态
	BOOL result;
	LCD_RS = 0;
	LCD_RW = 1;
	LCD_EP = 1;
	_nop_();	_nop_();	_nop_();	_nop_();
	result = (BOOL)(P0 & 0x80);
	LCD_EP = 0;
	return result; 
}
/*函数功能: LCD写命令*/
void lcd_wcmd(uchar cmd)
{                          // 写入指令数据到LCD
	while(lcd_bz());
	LCD_RS = 0;
	LCD_RW = 0;
	LCD_EP = 0;
	_nop_();	_nop_(); 
	P0 = cmd;
	_nop_();	_nop_();	_nop_();	_nop_();
	LCD_EP = 1;
	_nop_();	_nop_();	_nop_();	_nop_();
	LCD_EP = 0;  
}

/*函数功能: LCD显示位置设置*/
void lcd_pos(uchar pos)				  //设定显示位置
{                         
	lcd_wcmd(pos | 0x80);
}
/*函数功能: LCD写数据*/
void lcd_wdat(uchar dat) 
{                          //写入字符显示数据到LCD
	while(lcd_bz());
	LCD_RS = 1;
	LCD_RW = 0;
	LCD_EP = 0;
	P0 = dat;
	_nop_();	_nop_();	_nop_();	_nop_();
	LCD_EP = 1;
	_nop_();	_nop_();	_nop_();	_nop_();
	LCD_EP = 0; 
}
/*函数功能: LCD初始化*/
void initiate_lcd()
{                        //LCD初始化设定
	lcd_wcmd(0x38);          //16*2显示，5*7点阵，8位数据
	Delay1ms(1);
	lcd_wcmd(0x0c);          //显示开，关光标
	Delay1ms(1);
	lcd_wcmd(0x06);          //移动光标
	Delay1ms(1);
	lcd_wcmd(0x01);          //清除LCD的显示内容
	Delay1ms(1);  
}
/*函数功能: 初始化显示*/
void initiate_Show()
{
	lcd_pos(0x80); lcd_wdat('S');		//湿度显示初始化	
	lcd_pos(0x81);	lcd_wdat(':');	
	lcd_pos(0x84);	lcd_wdat('%');
	lcd_pos(0x85);	lcd_wdat('R');
	lcd_pos(0x86);	lcd_wdat('H');

	lcd_pos(0x88); lcd_wdat('P');		//PM2.5显示初始化
	lcd_pos(0x89);	lcd_wdat('M');
	lcd_pos(0x8A);	lcd_wdat(':'); 
	lcd_pos(0x8e);	lcd_wdat(0xe4);
	lcd_pos(0x8f);	lcd_wdat('g');

	lcd_pos(0x40); lcd_wdat('W');			//温度显示初始化
	lcd_pos(0x41);	lcd_wdat(':'); 
	lcd_pos(0x44);	lcd_wdat(0xdf);
	lcd_pos(0x45);	lcd_wdat('C'); 

	lcd_pos(0x48); lcd_wdat('J');			//甲醛显示初始化
	lcd_pos(0x49);	lcd_wdat(':');
	lcd_pos(0x4e);	lcd_wdat('m');
	lcd_pos(0x4f);	lcd_wdat('g');
}
/*函数功能: 甲醛浓度显示*/
void JiaQuan_Show()
{
	int ad_result=0;  
	ad_result=ReadADCResult(1);
	c0 = ((double)ad_result*(4.42/1024));
	c1 = fabs(c0-2.50);
	c2 = c1*1.528-(pow(c1,2)*0.125)-2.631;
	c3 = pow(10,c2);	
	c4 = c3*1235.22;					//等价于 c4 = c3*30/22.4*273/296*1000;
	cdisplay = (int)c4;
	 
	lcd_pos(0x4a);	lcd_wdat('.');
	lcd_pos(0x4b);	lcd_wdat(cdisplay/100+0x30);
	lcd_pos(0x4c);	lcd_wdat((cdisplay%100)/10+0x30);	                   
	lcd_pos(0x4d);  	lcd_wdat(cdisplay%10+0x30);	
	Delay1ms(10);
	   
}  
/*函数功能: PM25浓度显示*/
void PM_Show()
{
	concentration*=100;
	ddisplay = (int) concentration;
	lcd_pos(0x8b);	lcd_wdat(ddisplay/100+0x30);
	lcd_pos(0x8c);	lcd_wdat((ddisplay%100)/10+0x30);            
	lcd_pos(0x8d);  	lcd_wdat(ddisplay%10+0x30);
	Delay1ms(10);	   
}
/*函数功能: PM离散采样函数*/
void Dis_collection()
{
	int PM_result=0;  
	PM_result=ReadADCResult(0);
	if(PM_result<320)
	{
		lowpulseoccupancy++; 					   	
	}
	totalsampletime++;
}  																		 
/*函数功能: ADC初始化*/
void Init_StcADC(uchar channel)
{
	P1ASF = 0x01 << channel;                 //Set P_channel as analog input port
	ADC_RES = 0;                           //Clear previous result
	ADC_RESL = 0;
	ADC_CONTR &= !ADC_FLAG;                //Clear ADC interrupt flag  不能位寻址
	ADC_CONTR = ADC_POWER | ADC_SPEEDHH | ADC_START | channel; 
	 //EADC = 1;                  //Set ADC interrupt
	Delay1ms(50);                         //ADC power-on delay and Start A/D conversion
	ADC_CONTR &= !ADC_FLAG;                //Clear ADC interrupt flag  不能位寻址
	ADC_CONTR = ADC_POWER | ADC_SPEEDHH | ADC_START | channel; 
	Delay1ms(5);                //第一个数据不准，要延时一段时间再测
}
/*函数功能: 读AD的电压值*/
unsigned int ReadADCResult(uchar channel)
{
	unsigned int adl = 0;
	unsigned int adh = 0;
	if((ADC_CONTR & ADC_FLAG) == ADC_FLAG)
	{
		ADC_CONTR &= !ADC_FLAG;               //Clear ADC interrupt flag  不能位寻址
		adh = ADC_RES;               //Get ADC high 8-bit result
		adl = ADC_RESL;              //Get ADC low  2-bit result
		adh = adh << 2;
		adh = adh + adl;            //Get ADC all 10-bit result
		ADC_CONTR = ADC_POWER | ADC_SPEEDHH | ADC_START | channel;
	}
		return(adh);
}
/*函数功能: 温湿度读取函数*/
void Read_Data()			//单次取值		D1是数据的存储空间
{
	uint i;
    for(i=0;i<8;i++)	   
	{	
   	     flag=2; 
   	   	 while((!DHT11)&&flag++);	//2.043ms P10是1的时候出来
		 Delay5us(4);  				//45us（28-70）
		 flag_r=0;
     	 if(DHT11)			//如果低电平（50微秒低电平数据的标志）
			flag_r=1;
		 StrData<<=1;		
   	     StrData|=flag_r; 	    
		 flag=2;			
	 	 while((DHT11)&&flag++);	
	  	}	   
}
/*函数功能: 温湿度初始化及数据采集*/
void Init_DHTRead()
{
	Delay1ms(80);
	DHT11=0;				//开始信号
	Delay1ms(6);			//大于18ms  18.925ms
	DHT11=1;
	flag=2;				//开始信号结束
	while((DHT11)&&flag++);	//主机拉高20us结束
	if(!DHT11)			//有低电平响应可读取DHT11的响应信号p10=0出
	{
		Delay5us(18);  //90us		
		flag=2;	 
	 	while((DHT11)&&flag++);	//低电平出	
		Read_Data();		read_dat[0]=StrData;
		Read_Data();		read_dat[1]=StrData;
		Read_Data();		read_dat[2]=StrData;
		Read_Data();		read_dat[3]=StrData;
		Read_Data();		read_dat[4]=StrData;   
		DHT11=1;			//注意DHT11温度 湿度的小数部分为0
	}
	check=read_dat[0]+read_dat[1]+read_dat[2]+read_dat[3];
	if(check=read_dat[4])
	{
		show_dat[0]=read_dat[0];
		show_dat[1]=read_dat[1];
		show_dat[2]=read_dat[2];
		show_dat[3]=read_dat[3];
	}
}
/*函数功能: 数据转换成单字符*/
void Conv_SChar(int k)      
{
	conv_dat[0]=show_dat[k]/10;
	conv_dat[1]=show_dat[k]%10;
}
/*函数功能: 显示温湿度数值*/
void WenShi_Show()		
{
	uint i;
	lcd_pos(0x82);	 //第一行显示湿度/
	Delay1ms(1);
		Conv_SChar(0);
	for(i=0;i<2;i++)
	{
		lcd_wdat('0'+conv_dat[i]);
		Delay1ms(1);	
	}
	lcd_pos(0x42);  //第二行显示温度/			
	Conv_SChar(2);
	for(i=0;i<2;i++)
	{
		lcd_wdat('0'+conv_dat[i]);
		Delay1ms(1);
	} 	
}
/*函数功能: 定时器及外部中断初始化*/
void initiate_Timer()		
{ 
	TMOD=0x19;
	TH1=(65536-45872)/256;
	TL1=(65536-45872)%256;
	EA=1;	ET1=1;	EX0=1;	IT0=1;	TR1=1; 
}
/*函数功能: 主程序*/ 
main()
{
	CLK_DIV=0x02;			//时钟分频寄存器设定
	initiate_lcd();         //初始化LCD控制的初始化
	initiate_Show();        //对LCD显示结果进行初始化
	Delay1ms(200);			//延时使显示稳定
	initiate_Show();
	Init_StcADC(0);         //对ADC0的初始化
	Init_StcADC(1);         //对ADC0的初始化             	 	
	initiate_Timer();       //定时器及外部中断的初始化
	while(1)                //循环扫描
	{ 
		Init_DHTRead();						
		WenShi_Show();
		JiaQuan_Show();
		Dis_collection();
		Delay1ms(50);			
	}
}
/*函数功能: 外部中断处理程序*/
void theTimer0() interrupt 0			 
{
	LED=0;
	beep=0;	
	Delay1ms(3000);
	LED=1;
	beep=1;
			  
}	
/*函数功能: 定时器中断处理程序*/	 
void theTimer1() interrupt 3			 //50ms进入一次中断
{
	TR1=0;
	TH1=(65536-45872)/256;
	TL1=(65536-45872)%256;
	count30s++;
	if(count30s==150)					
	{
		count30s=0;
		ratio = lowpulseoccupancy/totalsampletime;  // 采样低电平比率 0=>100
		concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // 浓度与低电平比率相关曲线方程
		PM_Show();
		lowpulseoccupancy=0;
		totalsampletime=0;
	}
	TR1=1;			  
} 
