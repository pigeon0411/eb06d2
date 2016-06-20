#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#include "includes.h"
#include "osd_menu.h"
#include "key_ctl.h"


#define	LONG_KEY_DELAY			40


#define	PD_ZOOM_FOCUS_STOP		0
#define	PD_ZOOM_TELE_CMD		1
#define	PD_ZOOM_WIDE_CMD		2
#define	PD_FOCUS_FAR_CMD		3
#define	PD_FOCUS_NEAR_CMD		4



void pelcod_call_pre_packet_send(u8 val);



#define	KEY_PORT1		GPIOA
#define	KEY_PORT2		GPIOB

void key_pin_init(void)
{
	GPIO_InitTypeDef GPIOD_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
		
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOD_InitStructure); 

	
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIOD_InitStructure); 


}




extern rt_sem_t	uart1_sem;

rt_err_t rs485_recieve_check(u8 val)
{

	
	if(rt_sem_take(uart1_sem, 30) == RT_EOK)
    {
		if (command_analysis()) 
		{
            switch(command_byte)
		    {
			 	case 0x11://call preset point

					if(Rocket_fir_data == val)
						return RT_EOK;
					break;

             	default:
				break;
	   	    }

		}
	}
	return RT_ERROR;

}


u8 cmd_buff[7];

rt_sem_t rs485_return_sem;

extern rt_err_t rs485_send_data(u8* data,u16 len);

void pelcod_call_pre_packet_send(u8 val)
{
	u8 cnt;
	cmd_buff[0] = 0xff;
	cmd_buff[1] = 0xff;
	cmd_buff[2] = 0;
	cmd_buff[3] = 0x07;
	cmd_buff[4] = 0;
	cmd_buff[5] = val;
	
	cmd_buff[6] = cmd_buff[1] + cmd_buff[2] + cmd_buff[3] + cmd_buff[4] + cmd_buff[5];
	rs485_send_data(cmd_buff,7);

//	cnt=3;
//	while(cnt--)
//	{
//		if(RT_EOK == rs485_recieve_check(val))
//			break;
//		else
//			rs485_send_data(cmd_buff,7);
//	}
}


void pelcod_set_pre_packet_send(u8 val)
{
	u8 cnt;
	cmd_buff[0] = 0xff;
	cmd_buff[1] = 0xff;
	cmd_buff[2] = 0;
	cmd_buff[3] = 0x03;
	cmd_buff[4] = 0;
	cmd_buff[5] = val;
	
	cmd_buff[6] = cmd_buff[1] + cmd_buff[2] + cmd_buff[3] + cmd_buff[4] + cmd_buff[5];
	rs485_send_data(cmd_buff,7);

//	cnt=3;
//	while(cnt--)
//	{
//		if(RT_EOK == rs485_recieve_check(val))
//			break;
//		else
//			rs485_send_data(cmd_buff,7);
//	}
}

//val: 0,open; 1,close
void pelcod_open_close_packet_send(u8 val)
{
	u8 cnt;
	cmd_buff[0] = 0xff;
	cmd_buff[1] = 0xff;
	if(val)//close
		cmd_buff[2] = 0x04;
	else
		cmd_buff[2] = 0x02;
	cmd_buff[3] = 0;
	cmd_buff[4] = 0;
	cmd_buff[5] = 0;
	
	cmd_buff[6] = cmd_buff[1] + cmd_buff[2] + cmd_buff[3] + cmd_buff[4] + cmd_buff[5];
	rs485_send_data(cmd_buff,7);

//	cnt=3;
//	while(cnt--)
//	{
//		if(RT_EOK == rs485_recieve_check(val))
//			break;
//		else
//			rs485_send_data(cmd_buff,7);
//	}
}

//cmd,0,stop; 1,tele,2wide; 3,far,4,near
void pelcod_zf_packet_send(u8 cmd,u8 zfspeed)
{
	u8 cnt;
	
	u8 cmd_buff_private[7];
	cmd_buff_private[0] = 0xff;
	cmd_buff_private[1] = domeNo;

	
	switch(cmd)
	{
	case 1:
		cmd_buff_private[3] = 0x20;
		cmd_buff_private[2] = 0;
		break;
	case 2:
		cmd_buff_private[3] = 0x40;
		cmd_buff_private[2] = 0;
		break;
	case 3:
		cmd_buff_private[3] = 0x00;
		cmd_buff_private[2] = 0x01;//
		break;
	case 4:
		cmd_buff_private[3] = 0x80;
		cmd_buff_private[2] = 0;
		break;
	case 0:
		cmd_buff_private[3] = 0x00;
		cmd_buff_private[2] = 0;
		break;
	}
	
	cmd_buff_private[4] = 0;
	cmd_buff_private[5] = 0;
	
	cmd_buff_private[6] = cmd_buff_private[1] + cmd_buff_private[2] + cmd_buff_private[3] + cmd_buff_private[4] + cmd_buff_private[5];
	rs485_send_data(cmd_buff_private,7);
}



u16 KeyPre = 0;//  上次按键的状态
u16 KeyCurrent = 0;//  本次按键的状态
u16 KeyStbLast = 0;// 经滤波后的上次按键的状态
u16 KeyStb = 0;//  经滤波后的本次按键的状态
u16 KeyAvlUp = 0;// 按键抬起有效结果寄存器
u16 KeyAvlDown = 0;//  按键按下有效结果寄存器

#define	KEY_DOWN_STATE		0

u16 key_val_table[] = {KEY_LEFT_PIN,KEY_RIGHT_PIN,KEY_UP_PIN,KEY_DOWN_PIN,KEY_SET_PIN};




#define		KEY_DEFAULT		0X1F00


u16 key_pre = 0;
u16 key_pre2 = 0;



#define	KEY_PORT1		GPIOA
#define	KEY_PORT2		GPIOB
#define	KEY_PORT3		GPIOC


enum key_type
{
	KEY_NONE,
		KEY_FOCUS_PLUS, 
		
		KEY_ZOOM_SUB,	

	
	KEY_ZOOM_PLUS,	
		KEY_FOCUS_SUB,	
	
	KEY_MODE,	
};

#define	key_to_long(val)	(val|0x9000)
#define	key_to_release(val)	(val|0x8000)


u32 key_merge(void)
{
	u32 data = 0;

	u32 key_tmp;
	
	data = GPIO_ReadInputData(KEY_PORT3);	
		
	key_tmp = (data>>6)&0x000f;//0-1


	return key_tmp;
}

#define	KEY_NUMS_MAX	4


u32 press_continue_cnt = 0;
u8 continue_motor_flag = 0;
u16 motor_steps_cnt = 0;
u8 press_long_flag = 0;

u8 press_long_flag2 = 0;
u32 press_continue_cnt2 = 0;
u8 continue_motor_flag2 = 0;

u8 key_combine_state = 0;



static u16 key_check_1(void)
{
	u16 i;
	u32 key_tmp;
	static u32 long_press_cnt = 0;// 50ms
	
	key_tmp = key_merge();
	for(i=0;i<KEY_NUMS_MAX;i++)
	{
		if(((key_tmp>>i)&0x0001)==0)
		{
			rt_thread_delay(LONG_KEY_DELAY);

			key_tmp = key_merge();

			if(((key_tmp>>i)&0x0001)==0)
			{
				if(key_pre == i+1)
				{

					if(key_pre2 != KEY_NONE)
					{
						if(long_press_cnt>30)
						{
							if(press_long_flag)
								return 0;
							
							press_long_flag = 1;
							long_press_cnt=0;
							key_pre = 0;

							
							return ((i+1)|0x9000);
						}
						else
						{
							long_press_cnt++;

						}
					}
					else
					{
						key_pre = i+1;
						return (key_pre);	

					}

				}
				key_pre = i+1;
				//return (i+1);
				break;
			}
		}
	}


	if((key_pre && key_pre!=(i+1))||(key_pre && i>=KEY_NUMS_MAX))
	{
		i = key_pre|0x8000;
		key_pre = 0;

		if(press_continue_cnt>10)
			i = 0;

		if(press_long_flag)
		{
			press_long_flag = 0;
			i =0;
		}
		press_continue_cnt = 0;
		motor_steps_cnt = 0;
			
		continue_motor_flag = 0;
		
		return i;

	}
	return 0;
}	


static u16 key_check_2(void)
{
	u16 i=0;
	u32 key_tmp;
	static u32 long_press_cnt = 0;// 50ms
	
	key_tmp = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
	

	if(((key_tmp)&0x0001)==0)
	{
		rt_thread_delay(40);

		key_tmp = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);

		i=0;
		if(((key_tmp)&0x0001)==0)
		{
			if(key_pre2 == KEY_MODE)
			{
				if(long_press_cnt>40)
				{
					if(press_long_flag2)
						return 0;
					
					press_long_flag2 = 1;
					long_press_cnt=0;
					key_pre2 = 0;

					
					return (KEY_MODE|0x9000);
				}
				
				long_press_cnt++;

			}
			key_pre2 = KEY_MODE;
			return (KEY_MODE);
		}
	}


	if(key_pre2==KEY_MODE)
	{
		i = key_pre2|0x8000;
		key_pre2 = 0;

		if(press_continue_cnt2>10)
			i = 0;

		if(press_long_flag2)
		{
			press_long_flag2 = 0;
			i =0;
		}
		press_continue_cnt = 0;
			
		continue_motor_flag2 = 0;
		
		return i;

	}
	return 0;
}



//返回0为无按键，返回非0值，则为对应的按键号
static u16 key_ctl_check(void)
{
	key_check_2();

	
	return (key_check_1());
	
}



void key_analyze_mode(u16 mode_val,u16 val)
{

	if(mode_val == key_to_release(KEY_MODE) ||(mode_val == KEY_MODE)||(mode_val == key_to_long(KEY_MODE)))
	{
		
		switch(val)
		{
		case key_to_release(KEY_FOCUS_PLUS):
			
			pelcod_call_pre_packet_send(203);
		
			break;
		case key_to_release(KEY_FOCUS_SUB):
			
			pelcod_call_pre_packet_send(204);
		
			break;
		case key_to_release(KEY_ZOOM_PLUS):
			pelcod_call_pre_packet_send(201);
		
			break;

			
		case key_to_release(KEY_ZOOM_SUB):
			pelcod_call_pre_packet_send(202);
		
			break;
		case key_to_long(KEY_FOCUS_PLUS):
			
			pelcod_call_pre_packet_send(207);
		
			break;
		case key_to_long(KEY_FOCUS_SUB):
			
			pelcod_call_pre_packet_send(208);
		
			break;
		case key_to_long(KEY_ZOOM_PLUS):
			
			pelcod_call_pre_packet_send(205);
		
			break;
		case key_to_long(KEY_ZOOM_SUB):
			pelcod_call_pre_packet_send(206);
		
			break;
							
		default:
			break;
		}

		return;
	}
	
	
}


void key_analyze_non_mode(u16 mode_val,u16 val)
{

	
	if(mode_val == 0)
	{
		if(val > 0 && val < KEY_MODE)
			{
				switch(val)
				{
				case (KEY_FOCUS_PLUS):
					pelcod_zf_packet_send(PD_FOCUS_FAR_CMD,0);
					break;
				case (KEY_FOCUS_SUB):
					pelcod_zf_packet_send(PD_FOCUS_NEAR_CMD,0);
					break;
		
				case (KEY_ZOOM_PLUS):
					pelcod_zf_packet_send(PD_ZOOM_TELE_CMD,0);
					break;
				case (KEY_ZOOM_SUB):
					pelcod_zf_packet_send(PD_ZOOM_WIDE_CMD,0);
					break;
				default:
					break;
				}
		
				return;
		
			}
		
			switch(val)
			{
			case key_to_release(KEY_FOCUS_PLUS):
			case key_to_release(KEY_FOCUS_SUB):
			case key_to_release(KEY_ZOOM_PLUS):
			case key_to_release(KEY_ZOOM_SUB):
				pelcod_zf_packet_send(PD_ZOOM_FOCUS_STOP,0);
		
				break;	
				
			default:
				break;
			}

	}
}


static u16 key_ctl_check_all(void)
{
	u16 key_mode_tmp;
	u16 key_tmp ;
	static u16 key_tmp_bak =0xff;


	key_mode_tmp = key_check_2();
	key_tmp = key_check_1();

	if(key_mode_tmp || press_long_flag2)
	{
		if(key_tmp)
		{
			if(press_long_flag2)
			{
				key_mode_tmp = key_to_long(KEY_MODE);

			}
			key_analyze_mode(key_mode_tmp,key_tmp);
			
		}

		key_combine_state = 1;
	}
	else
	{

		if(key_tmp)
		{
			if(key_combine_state)
			{	
				if(key_tmp_bak != key_tmp && key_tmp_bak!=0xff)
				{
					key_combine_state = 0;

				}
				
			}
			else
			{
				key_analyze_non_mode(key_mode_tmp,key_tmp);
			}

			key_tmp_bak = key_tmp;

		}
	}
	return 0;
	
}



void rt_key_thread_entry(void* parameter)
{

	u16 k;

	key_pin_init();
	

    while(1)
	{

        key_ctl_check_all();


		
		
		rt_thread_delay(4);
    }
}




int rt_key_ctl_init(void)
{

	
    rt_thread_t init_thread;

    init_thread = rt_thread_create("key",
                                   rt_key_thread_entry, RT_NULL,
                                   4096, 10, 5);
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

