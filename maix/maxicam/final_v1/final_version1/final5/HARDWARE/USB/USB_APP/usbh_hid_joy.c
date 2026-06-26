#include "usbh_hid_joy.h"


static void  JOY_Init (void);
static void  JOY_Decode(uint8_t *data);
extern HID_Machine_TypeDef HID_Machine;
PS2_TypeDef ps2_data;

u8 ps2_type = 0;

HID_cb_TypeDef HID_JOY_cb = 
{
  JOY_Init,
  JOY_Decode,
};

static void  JOY_Init ( void )
{
	ps2_data.KeyState = 0x0;
//	printf("手柄初始化\r\n");
}

//标志位设置函数
void ps2_set_bit(uint16_t* state,u8 state_bit,u8 bit)
{
	if(state_bit==1) //指定的位(bit)设置为1,其他位不变
	{
		*state |= (1U<<bit);
	}
	else //指定的位(bit)设置为0,其他位不变
	{
		*state &= ~(1U<<bit);
	}
}

#define debug_ps2_data 0

static void  JOY_Decode(uint8_t *data)
{
	
	
	u8 tmp_bool = 0;
	
	//有线ps2手柄数据解包
	if(ps2_type==Normal_PS2)
	{
		#if debug_ps2_data
		printf("有线ps2: ");
		for(u8 i=0;i<HID_Machine.length;i++)
		{
			printf("%d\t",data[i]);
		}
		printf("\r\n");
		#endif
		
		usb_ps2_ready = 1;
		
		ps2_data.LX = data[3];
		ps2_data.LY = data[4];
		ps2_data.RX = data[1];
		ps2_data.RY = data[2];
		
		tmp_bool = (data[6]>>4)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,0); //seltec key 选择按键
		
		tmp_bool = (data[6]>>6)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,1); //左摇杆按键
		
		tmp_bool = (data[6]>>7)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,2); //右摇杆按键
		
		tmp_bool = (data[6]>>5)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,3); //start
		
		tmp_bool = data[5]&0x0F;//取出低4位
		if(tmp_bool==0x0F)//没有任何按键按下
		{
			ps2_set_bit(&ps2_data.KeyState,0,4); //↑
			ps2_set_bit(&ps2_data.KeyState,0,5); //→
			ps2_set_bit(&ps2_data.KeyState,0,6); //↓
			ps2_set_bit(&ps2_data.KeyState,0,7); //←
		}
		else if( (tmp_bool&0x01)==0 )
		{	
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑
					ps2_set_bit(&ps2_data.KeyState,1,4); //↑
					break;
				case 0x01://→
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x02://↓
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					break;
				case 0x03://←
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				default:
					break;
			}
		}
		else if( (tmp_bool&0x01)==1 ) //首位为1,代表存在左盘2个按键按下的情况
		{
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑→
					ps2_set_bit(&ps2_data.KeyState,1,4);//↑
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x01://↓→
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x02://↓←
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				case 0x03://↑←
					ps2_set_bit(&ps2_data.KeyState,1,4); //↑
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				default:
					break;
			}
		}
		
		tmp_bool = (data[6]>>2)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,8); //左扳机2号
		
		tmp_bool = (data[6]>>3)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,9); //右扳机2号
		
		tmp_bool = (data[6]>>0)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,10); //左扳机1号
		
		tmp_bool = (data[6]>>1)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,11); //右扳机1号
		
		
		tmp_bool = (data[5]>>4)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,12); //一号,绿色GREEN
		
		tmp_bool = (data[5]>>5)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,13); //二号,红色RED
	
		tmp_bool = (data[5]>>6)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,14); //三号,蓝牙BLUE
		
		tmp_bool = (data[5]>>7)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,15); //四号,粉色PINK
	}
	
	//无线ps2手柄数据解包
	else if( ps2_type==_2_4G_Android_PS2 )
	{
		#if debug_ps2_data
		printf("无线ps2安卓模式: ");
		for(u8 i=0;i<HID_Machine.length;i++)
		{
			printf("%d\t",data[i]);
		}
		printf("\r\n");
		#endif
		
		usb_ps2_ready = 1;
		
		if( data[6]==0&&data[7]==0 ) data[6]=128;
		ps2_data.LX = data[6];
		
		if( data[8]==0&&data[9]==0  ) data[8]=128;
		ps2_data.LY = 255 -data[8];
		
		if( data[10]==0&&data[11]==0  ) data[10]=128;
		ps2_data.RX = data[10];
		
		if( data[12]==0&&data[12]==0  ) data[12]=128;
		ps2_data.RY = 255 - data[12];
		
		//data[2]
		//Rm    Lm    select   start    →      ←       ↓        ↑
		//0		0		0		0		0		0		0		0
		tmp_bool = (data[2]>>0)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,4); //↑
		
		tmp_bool = (data[2]>>3)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,5); //→
		
		tmp_bool = (data[2]>>1)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,6); //↓
		
		tmp_bool = (data[2]>>2)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,7); //←
		
		tmp_bool = (data[2]>>5)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,0); //seltec key 选择按键	
		
		tmp_bool = (data[2]>>4)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,3); //start key 选择按键
		
		tmp_bool = (data[2]>>6)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,1); //左摇杆按键	
		
		tmp_bool = (data[2]>>7)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,2); //右摇杆按键
		
		tmp_bool = (data[3]>>0)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,10); //左扳机1号
		
		tmp_bool = (data[3]>>1)&0x01;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,11); //右扳机1号
		
		if(data[4]==0xff) tmp_bool=1;
		else tmp_bool=0;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,8); //左扳机2号
		
		if(data[5]==0xff) tmp_bool=1;
		else tmp_bool=0;
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,9); //右扳机2号
		
		tmp_bool = (data[3]>>4)&0x01;//BLUE
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,14);
		
		tmp_bool = (data[3]>>5)&0x01;//RED
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,13);
		
		tmp_bool = (data[3]>>6)&0x01;//PINK
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,15);
		
		tmp_bool = (data[3]>>7)&0x01;//GREEN
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,12);
	}
	
	else if( ps2_type==_2_4G_PC_PS2 ) //无线ps2手柄pc模式
	{
		#if debug_ps2_data
		printf("无线ps2 PC模式: ");
		for(u8 i=0;i<HID_Machine.length;i++)
		{
			printf("%d\t",data[i]);
		}
		printf("\r\n");
		#endif
		
		usb_ps2_ready=1;
		
		ps2_data.LX = data[3];
		ps2_data.LY = data[4];
		ps2_data.RX = data[5];
		ps2_data.RY = data[6];
		
		tmp_bool = (data[1]>>0)&0x01;//select
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,0); //seltec key 选择按键	
		
		tmp_bool = (data[1]>>1)&0x01;//start
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,3); //start key 选择按键
		
		tmp_bool = (data[1]>>2)&0x01;//Lm
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,1); //左摇杆按键	
		
		tmp_bool = (data[1]>>3)&0x01;//Rm
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,2); //右摇杆按键
		
		tmp_bool = (data[0]>>4)&0x01;//L1
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,10); //左扳机1号
		
		tmp_bool = (data[0]>>5)&0x01;//R1
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,11); //右扳机1号
		
		tmp_bool = (data[0]>>6)&0x01;//L2
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,8); //左扳机2号
		
		tmp_bool = (data[0]>>7)&0x01;//R2
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,9); //左扳机2号
		
		tmp_bool = (data[0]>>0)&0x01;//GREEN
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,12);
		
		tmp_bool = (data[0]>>1)&0x01;//RED
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,13);
		
		tmp_bool = (data[0]>>2)&0x01;//BLUE
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,14);
		
		tmp_bool = (data[0]>>3)&0x01;//PINK
		ps2_set_bit(&ps2_data.KeyState,tmp_bool,15);
		
		tmp_bool = data[2]&0x0F;//取出低4位
		if(tmp_bool==0x0F)//没有任何按键按下
		{
			ps2_set_bit(&ps2_data.KeyState,0,4); //↑
			ps2_set_bit(&ps2_data.KeyState,0,5); //→
			ps2_set_bit(&ps2_data.KeyState,0,6); //↓
			ps2_set_bit(&ps2_data.KeyState,0,7); //←
		}
		else if( (tmp_bool&0x01)==0 )
		{	
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑
					ps2_set_bit(&ps2_data.KeyState,1,4); //↑
					break;
				case 0x01://→
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x02://↓
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					break;
				case 0x03://←
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				default:
					break;
			}
		}
		else if( (tmp_bool&0x01)==1 ) //首位为1,代表存在左盘2个按键按下的情况
		{
			switch ((tmp_bool>>1)&0x03)
			{
				case 0x00://↑→
					ps2_set_bit(&ps2_data.KeyState,1,4);//↑
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x01://↓→
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					ps2_set_bit(&ps2_data.KeyState,1,5); //→
					break;
				case 0x02://↓←
					ps2_set_bit(&ps2_data.KeyState,1,6); //↓
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				case 0x03://↑←
					ps2_set_bit(&ps2_data.KeyState,1,4); //↑
					ps2_set_bit(&ps2_data.KeyState,1,7); //←
					break;
				default:
					break;
			}
		}

	}
	else
	{
		//未知设备
		usb_ps2_ready = 0;
	}
		

	

	
}
