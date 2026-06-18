/*
 * 程序清单：这是一个 底层硬件驱动板程序
 *
 *
 * 程序功能：芯片驱动程序。
*/

#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include "drvBoard.h"
#include "car_ble_ctrl.h"
#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "Board"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

/***************** 属性定义 ********************************/
uint8_t current_work_mode = 0;         // 0=正常模式, 1=单机模式
/* 建图模式标志，RT_TRUE=建图模式，RT_FALSE=导航模式（默认）*/
rt_bool_t slam_mode = RT_FALSE;

eWorkMode current_mode = MODE_CAR;  // 默认小车模式

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */
static rt_uint8_t pulseDuty2;
static struct rt_semaphore rx_sem;    /* 用于接收消息的信号量 */
Device_StateTypeDef device_state = eState_initialize;	/*板载系统状态初始化*/

static rt_int32_t m_dwLastKeyCode = KEY_NULL;	// 最后一次按键编码值
static rt_bool_t m_flgKeyDown = RT_FALSE;		// 已经有按键按下事件发生
static rt_int32_t g_KeyMsgDown = 0;	//当前被按下的按键消息
static rt_bool_t KeyUnlock=RT_FALSE;   // false=锁 ture=解锁
static rt_timer_t unlock_timer;		// 解锁后定时器

static rt_tick_t keyTick = 0;

struct rt_mailbox mbKeyMsg;	// 邮箱相关变量   用于动作指令  传递按键值给BLE发送到中控屏
static rt_uint32_t mbKeyPool[1];   // 存放1份邮件

static rt_tick_t switch_press_start = 0;      // KEY_SWITCH 按下时刻
static rt_bool_t switch_long_pressed = RT_FALSE; // 长按是否已触发

/* 长按检测阈值（单位：毫秒）*/
#define LONG_PRESS_MS_UNLOCK  200   // KEY_LOCK 长按解锁阈值 1秒
#define LONG_PRESS_MS_SWITCH  5000   // KEY_SWITCH 长按切换单机/正常模式
#define LONG_PRESS_MS_SEWAGE  5500   // KEY_SEWAGE 长按排污（5.5秒）
#define UNLOCK_TIME_S					120			// 键盘锁解锁时间 单位秒
/* 点动按键重复发送间隔（毫秒）*/
#define MOTION_REPEAT_INTERVAL_MS  50

/* 点动按键重复发送定时器 */
static rt_timer_t motion_repeat_timer;
static uint8_t current_motion_key = 0;      // 当前按下的点动按键命令码
static rt_bool_t motion_key_pressed = RT_FALSE;


#define RT_KEY_THREAD_STACK_SIZE     4096
#define RT_KEY_THREAD_PRIORITY       15
static rt_thread_t KEYThread;

#define ADC_DEV_NAME        "adc1"
static rt_adc_device_t adc_dev;

// 关机检测相关变量
static rt_tick_t shutdown_press_start = 0;
static rt_bool_t shutdown_triggered = RT_FALSE;  // 防止重复触发
#define SHUTDOWN_HOLD_MS   1500   // 长按1.5秒关机

extern rt_bool_t ble_connected;

typedef struct
{
	rt_int32_t key;	// 物理键值
	E_KeyMessage msg;	// 对应的按键消息
}S_KeyMapping;

// 物理键值与按键消息关系映射表
// 小车模式按键映射表 (物理键值 -> 小车消息枚举)
static const S_KeyMapping C_carKeyMapping[] = 
{
    {KEY_NULL, KEY_RELEASE},   // 按键释放时发送		
		{0, KEY_SLAM},
    {1, KEY_STOPWORK},

    {2, KEY_FORWORD},
    {3, KEY_BACKWORD},
    {4, KEY_LEFT},
    {5, KEY_RIGHT},

    {6, KEY_ROME_A},
    {7, KEY_ROME_B},

    {8, KEY_HOME},
    {9, KEY_FREE},
		
    {10, KEY_SWITCH},
    {11, KEY_LOCK},
};

// 马桶模式按键映射表 (物理键值 -> 马桶消息枚举)
static const S_KeyMapping C_wcKeyMapping[] = 
{	
		{KEY_NULL, KEY_RELEASE},
		{0, KEY_FLUSH},            // 冲洗马桶
		{1, KEY_STOP},            // 停止动作
		
		{2, KEY_BIANMEN_KAI},      // 马桶盖开
		{3, KEY_BIANMEN_GUAN},     // 马桶盖关
		{4, KEY_CLEAN_REAR},       // 臀部清洗
		{5, KEY_CLEAN_FEMALE},     // 女性清洗
		
		{6, KEY_CLEAN_MODE},       // 模式切换
		{7, KEY_CLEAN_STRENGTH},   // 力度调节
		{8, KEY_GANZAO},           // 暖风烘干
		{9, KEY_SEWAGE},           // 排污泵（长按触发）
		
		{10, KEY_SWITCH},           // 模式切换（仍有效）
		{11, KEY_LOCK},            // 解锁
};

static void stop_motion_repeat(void);
static void reset_unlock_timer(void);

void setPuty(rt_uint8_t ch,rt_int8_t percent)
{
	/* 设置PWM 脉冲宽度占空比 0-100*/	
	if(percent < 0 || percent > 100){
		return;
	}
	pulseDuty2 = percent;
	rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_2, PWM_TIMER, (percent * PWM_DUTY));		
}
/* PWM 初始化*/
void initPWM(void)
{
	LOG_D("init PWM2");
	/* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        LOG_D("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
    }
	/* 设置PWM 脉冲宽度占空比 0-100*/	
	/* 设置PWM周期和脉冲宽度默认值 */
		pulseDuty2 = 0;
	rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL_2, PWM_TIMER, (pulseDuty2 * PWM_DUTY));
/* 关闭设备通道 */
	setPuty(0,0);
	rt_pwm_disable(pwm_dev,PWM_DEV_CHANNEL_2);								
}



static void LED_OFF_ALL(void)
{
	rt_pin_write(LED1, PIN_HIGH);
	rt_pin_write(LED2, PIN_HIGH);
	rt_pin_write(LED3, PIN_HIGH);
	rt_pin_write(LED4, PIN_HIGH);
	rt_pin_write(LED5, PIN_HIGH);
	rt_pin_write(LED6, PIN_HIGH);
	rt_pin_write(LED7, PIN_HIGH);
	rt_pin_write(LED8, PIN_HIGH);
	rt_pin_write(LED9, PIN_HIGH);
	rt_pin_write(LED10, PIN_HIGH);
	rt_pin_write(LED11, PIN_HIGH);	
	rt_pin_write(LED12, PIN_HIGH);	
}

static void LED_Ctrl(rt_uint8_t on,rt_int32_t ch)
{
//	rt_kprintf("ch=%d,on=%d\r\n",ch,on);
	switch(ch)
	{
			case 0: 
			if(on){
				rt_pin_write(LED1, PIN_LOW);
			}else{
				rt_pin_write(LED1, PIN_HIGH);	
			}	
			break;		
		case 1: 
			if(on){
				rt_pin_write(LED2, PIN_LOW);
			}else{
				rt_pin_write(LED2, PIN_HIGH);	
			}	
			break;	
		case 2: 
			if(on){
				rt_pin_write(LED3, PIN_LOW);
			}else{
				rt_pin_write(LED3, PIN_HIGH);	
			}	
			break;		
		case 3: 
			if(on){
				rt_pin_write(LED4, PIN_LOW);
			}else{
				rt_pin_write(LED4, PIN_HIGH);	
			}	
			break;		
		case 4: 
			if(on){
				rt_pin_write(LED5, PIN_LOW);
			}else{
				rt_pin_write(LED5, PIN_HIGH);	
			}	
			break;
		case 5: 
			if(on){
				rt_pin_write(LED6, PIN_LOW);
			}else{
				rt_pin_write(LED6, PIN_HIGH);	
			}	
			break;
		case 6: 
			if(on){
				rt_pin_write(LED7, PIN_LOW);
			}else{
				rt_pin_write(LED7, PIN_HIGH);	
			}	
			break;
		case 7: 
			if(on){
				rt_pin_write(LED8, PIN_LOW);
			}else{
				rt_pin_write(LED8, PIN_HIGH);	
			}	
			break;
		case 8: 
			if(on){
				rt_pin_write(LED9, PIN_LOW);
			}else{
				rt_pin_write(LED9, PIN_HIGH);	
			}	
			break;
		case 9: 
			if(on){
				rt_pin_write(LED10, PIN_LOW);
			}else{
				rt_pin_write(LED10, PIN_HIGH);	
			}	
			break;
		case 10: 
			if(on){
				rt_pin_write(LED11, PIN_LOW);
			}else{
				rt_pin_write(LED11, PIN_HIGH);	
			}	
			break;
		case 11: 
			if(on){
				rt_pin_write(LED12, PIN_LOW);
			}else{
				rt_pin_write(LED12, PIN_HIGH);	
			}		
			break;
		default:break;
	}
}

static void ALARM_BUZZER(rt_uint8_t on)
{
	if(on){
		rt_pin_write(ALARM, PIN_LOW);
	}else{
		rt_pin_write(ALARM, PIN_HIGH);	
	}
}

void Alarm(rt_uint8_t status)
{
	if(status){
		ALARM_BUZZER(1);
		rt_thread_delay(RT_TICK_PER_SECOND*50/1000);
		ALARM_BUZZER(0);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		ALARM_BUZZER(1);
		rt_thread_delay(RT_TICK_PER_SECOND*50/1000);
		ALARM_BUZZER(0);	
	}else{
		ALARM_BUZZER(1);
		rt_thread_delay(RT_TICK_PER_SECOND*30/1000);
		ALARM_BUZZER(0);
	}
}

static void KEY_CTRL_LIE_IN(void)
{
	rt_pin_mode(LIE0, PIN_MODE_INPUT);
	rt_pin_mode(LIE1, PIN_MODE_INPUT);	
	rt_pin_mode(LIE2, PIN_MODE_INPUT);	
	rt_pin_mode(LIE3, PIN_MODE_INPUT);
}
static void KEY_CTRL_LIE_OUT(void)
{
	rt_pin_mode(LIE0, PIN_MODE_OUTPUT);
	rt_pin_mode(LIE1, PIN_MODE_OUTPUT);	
	rt_pin_mode(LIE2, PIN_MODE_OUTPUT);	
	rt_pin_mode(LIE3, PIN_MODE_OUTPUT);
}
static void KEY_LIE_CLR(void)
{
	rt_pin_write(LIE0, PIN_LOW);	
	rt_pin_write(LIE1, PIN_LOW);	
	rt_pin_write(LIE2, PIN_LOW);	
	rt_pin_write(LIE3, PIN_LOW);	
}
static void KEY_LIE_SET(void)
{
	rt_pin_write(LIE0, PIN_HIGH);	
	rt_pin_write(LIE1, PIN_HIGH);	
	rt_pin_write(LIE2, PIN_HIGH);	
	rt_pin_write(LIE3, PIN_HIGH);	
}
static rt_uint8_t GET_HANG(void)
{
	rt_uint8_t rowData=0;
	if(0 == rt_pin_read(HANG0)){
		rowData |= 1<<0;
	}
	if(0 == rt_pin_read(HANG1)){
		rowData |= 1<<1;
	}
	if(0 == rt_pin_read(HANG2)){
		rowData |= 1<<2;
	}
	return rowData;
}

static void KEY_CTRL_HANG_IN(void)
{
	rt_pin_mode(HANG0, PIN_MODE_INPUT);
	rt_pin_mode(HANG1, PIN_MODE_INPUT);	
	rt_pin_mode(HANG2, PIN_MODE_INPUT);		
}
static void KEY_CTRL_HANG_OUT(void)
{
	rt_pin_mode(HANG0, PIN_MODE_OUTPUT);
	rt_pin_mode(HANG1, PIN_MODE_OUTPUT);	
	rt_pin_mode(HANG2, PIN_MODE_OUTPUT);	
}
static void KEY_HANG_CLR(void)
{
	rt_pin_write(HANG0, PIN_LOW);	
	rt_pin_write(HANG1, PIN_LOW);	
	rt_pin_write(HANG2, PIN_LOW);		
}
static void KEY_HANG_SET(void)
{
	rt_pin_write(HANG0, PIN_HIGH);	
	rt_pin_write(HANG1, PIN_HIGH);	
	rt_pin_write(HANG2, PIN_HIGH);	
}
static rt_uint8_t GET_LIE(void)
{
	rt_uint8_t colData=0;
	if(0 == rt_pin_read(LIE0)){
		colData |= 1<<0;
	}
	if(0 == rt_pin_read(LIE1)){
		colData |= 1<<1;
	}
	if(0 == rt_pin_read(LIE2)){
		colData |= 1<<2;
	}
	if(0 == rt_pin_read(LIE3)){
		colData |= 1<<3;
	}
	return colData;
}


//扫描行列按键 返回第几行第几列被按下 注意：只能一个被按下才识别为有效
static void KeyCheck(rt_uint8_t* row,rt_uint8_t* col)
{
	rt_uint8_t rowData,colData;
	rt_uint32_t i;
	rt_uint8_t r=0,c=0;	
	
	KEY_CTRL_LIE_OUT();
	KEY_LIE_CLR();	
	rt_thread_delay(1);	// 1ms最小单位延时
	rowData =  GET_HANG();
//	rt_kprintf("rowData = %x\r\n",rowData);
	//	KEY_LIE_SET();
	KEY_CTRL_LIE_IN();
	if(0==rowData)   // 没有按键按下
	{
		*row = 0;
		*col = 0;
		return;
	}
	for(i=0;i<HANG_MAX;i++)
	{
		if (rowData&(1ul<<i))
		{
			if(0 != r)	// 保证只能有一次
			{
				*row = 0;
				*col = 0;
				return;
			}
			r |= 1ul<<i;
		}
	}
	
	KEY_CTRL_HANG_OUT();
	KEY_HANG_CLR();
	rt_thread_delay(1);
	colData = GET_LIE();
//	rt_kprintf("colData=%x\r\n",colData);
//	KEY_HANG_SET();
	KEY_CTRL_HANG_IN();
	for(i=0;i<LIE_MAX;i++)
	{
		if (colData&(1ul<<i))
		{
			if(0 != c)	// 保证只能有一次
			{
				*row = 0;
				*col = 0;
				return;
			}
			c |= 1ul<<i;
		}
	}
	
	*row = r;
	*col = c;	
}

// 多个按键读取
static void MulKeysCheck(rt_uint8_t* row,rt_uint8_t* col)
{
	rt_uint8_t rowData,colData;
	rt_uint32_t i;
	rt_uint8_t r=0,c=0;	
	
	KEY_CTRL_LIE_OUT();
	KEY_LIE_CLR();	
	rt_thread_delay(1);	// 1ms最小单位延时
	rowData =  GET_HANG();
//	rt_kprintf("rowData = %x\r\n",rowData);
	//	KEY_LIE_SET();
	KEY_CTRL_LIE_IN();
	if(0==rowData)   // 没有按键按下
	{
		*row = 0;
		*col = 0;
		return;
	}
	for(i=0;i<HANG_MAX;i++)
	{
		if (rowData&(1ul<<i))
		{
			r |= 1ul<<i;
		}
	}
	
	KEY_CTRL_HANG_OUT();
	KEY_HANG_CLR();
	rt_thread_delay(1);
	colData = GET_LIE();
//	rt_kprintf("colData=%x\r\n",colData);
//	KEY_HANG_SET();
	KEY_CTRL_HANG_IN();
	for(i=0;i<LIE_MAX;i++)
	{
		if (colData&(1ul<<i))
		{
			c |= 1ul<<i;
		}
	}
	
	*row = r;
	*col = c;	
}


// 根据行列返回按键值 (row,col) = row*列数+col
rt_int32_t GetKeyCode(rt_uint8_t row,rt_uint8_t col)
{
	rt_uint8_t r,c;
	if((0==col)||(0==row))
	{
		return KEY_NULL;
	}
	for (r=0;r<HANG_MAX;r++)
	{
		if (row&(1ul<<r))
		{
			break;
		}
	}
	for (c=0;c<LIE_MAX;c++)
	{
		if (col&(1ul<<c))
		{
			break;
		}
	}
	return (r * LIE_MAX) + c;
}
// 返回两个按键值
rt_int32_t* GetKeysCode(rt_uint8_t row,rt_uint8_t col)
{
	rt_uint8_t r,c,t=0;
	static rt_int32_t arr[2] = {0, 0};	
//	 rt_int32_t* arr = (rt_int32_t*)rt_malloc(3 * sizeof(rt_int32_t));
//	rt_memset(&arr,0,3);
	
	if((0==col)||(0==row))
	{
		return arr;
	}

		for (r=0;r<HANG_MAX;r++){
			if (row&(1ul<<r)){
				for (c=0;c<LIE_MAX;c++){
					if (col&(1ul<<c)){
						if(t>1){
							break;
						}
						arr[t] = (r * LIE_MAX) + c;
						rt_kprintf("getKey=%d\r\n",arr[t]);
						t++;
					}
				}
			}
		}			
	return arr;
}

// 物理键值转换为消息值，转换成功返回消息值，失败返回 KEY_NULL
rt_int32_t Key2Msg(rt_int32_t key)
{
    const S_KeyMapping *map;
    uint8_t map_size;
    
    if (current_mode == MODE_CAR) {
        map = C_carKeyMapping;
        map_size = sizeof(C_carKeyMapping) / sizeof(S_KeyMapping);
    } else {
        map = C_wcKeyMapping;
        map_size = sizeof(C_wcKeyMapping) / sizeof(S_KeyMapping);
    }
    
    for (uint8_t i = 0; i < map_size; i++) {
        if (map[i].key == key) {
            return map[i].msg;
        }
    }
    return KEY_NULL;  // 或 0
}


// 定时超时
static void unlock_timeout(void *parameter)
{
 // 锁定同时切换回小车模式
	if (current_mode != MODE_CAR) {
			current_mode = MODE_CAR;
			LED_Ctrl(0, 10);   // 熄灭模式指示灯（物理键10）
	}
	// 强制恢复建图模式为导航模式
    if (slam_mode) {
        slam_mode = RT_FALSE;
			  rt_mb_send(&mbKeyMsg, KEY_NAV_MODE);
        LED_Ctrl(0, 0);
    }
	// 停止点动重复发送
	if (motion_key_pressed) {
			if (motion_repeat_timer) {
					rt_timer_stop(motion_repeat_timer);
			}
			motion_key_pressed = RT_FALSE;
			current_motion_key = 0;
			/* 发送停止命令（KEY_RELEASE = 0）*/
			rt_mb_send(&mbKeyMsg, KEY_RELEASE);
	}
		LED_Ctrl(0,11);
		KeyUnlock = RT_FALSE;	
}

/* 判断是否为点动按键 */
static rt_bool_t is_motion_key(E_KeyMessage msg)
{
    return (msg == KEY_FORWORD || msg == KEY_BACKWORD ||
            msg == KEY_LEFT || msg == KEY_RIGHT);
}

/* 点动按键定时器回调：重复发送当前按键命令 */
static void motion_repeat_cb(void *param)
{
    if (motion_key_pressed && current_motion_key != 0) {
        rt_mb_send(&mbKeyMsg, current_motion_key);
			  // 每次发送点动命令时，重置解锁定时器，防止自动锁定
        if (KeyUnlock) {
            reset_unlock_timer();
        }
    }
}

/* 启动点动按键的重复发送定时器 */
static void start_motion_repeat(uint8_t cmd)
{
    if (motion_repeat_timer == RT_NULL) {
        motion_repeat_timer = rt_timer_create("motion_tmr",
                                               (void(*)(void*))motion_repeat_cb,
                                               RT_NULL,
                                               MOTION_REPEAT_INTERVAL_MS,
                                               RT_TIMER_FLAG_PERIODIC);
    }
    current_motion_key = cmd;
    motion_key_pressed = RT_TRUE;
    if (motion_repeat_timer) {
        rt_timer_stop(motion_repeat_timer);
        rt_timer_start(motion_repeat_timer);
    }
}

/* 停止点动按键的重复发送，并发送停止命令（0）*/
static void stop_motion_repeat(void)
{
    if (motion_repeat_timer) {
        rt_timer_stop(motion_repeat_timer);
    }
    motion_key_pressed = RT_FALSE;
    current_motion_key = 0;
    /* 发送停止命令（KEY_RELEASE = 0）*/
    // 连续发送三次停止命令，间隔 10ms，提高可靠性
    for (int i = 0; i < 1; i++) {
        rt_mb_send(&mbKeyMsg, KEY_RELEASE);
        rt_thread_mdelay(10);
    }
}

static void reset_unlock_timer(void)
{
    if (unlock_timer != RT_NULL  && KeyUnlock) {
        rt_timer_stop(unlock_timer);
        rt_timer_start(unlock_timer);
    }
}

// 解锁处理 handle_unlock
static void handle_unlock(void)
{
    if (KeyUnlock) {
        LED_Ctrl(0, 11);
        if (unlock_timer != RT_NULL) rt_timer_stop(unlock_timer);
				rt_kprintf("KEY LOCK,Forbidden\r\n");
			 // 切换回小车模式
        if (current_mode != MODE_CAR) {
            current_mode = MODE_CAR;
            LED_Ctrl(0, 10);
        }
        // 强制恢复建图模式为导航模式
        if (slam_mode) {
            slam_mode = RT_FALSE;
            // 可选：发送导航模式给下位机，但锁定后可能无效
            rt_mb_send(&mbKeyMsg, KEY_NAV_MODE);
            LED_Ctrl(0, 0);   // 熄灭 SLAM 按键 LED				
						rt_thread_mdelay(10);
        }
				KeyUnlock = RT_FALSE;
    } else {
        KeyUnlock = RT_TRUE;
				LED_Ctrl(1, 11);
        if (unlock_timer != RT_NULL) rt_timer_start(unlock_timer);
				rt_kprintf("KEY UNLOCK,Allow\r\n");
    }
    // 解锁时蜂鸣一声
    Alarm(0);
}

// KEY_SWITCH 长按处理（切换单机/正常模式）
static void handle_switch_long_press(void)
{
    if (current_work_mode == 0) {
        current_work_mode = 1;
        rt_mb_send(&mbKeyMsg, KEY_SINGLE_MODE);
// 闪烁前进、后退、左转、右转的 LED 三次
        for (int i = 0; i < 3; i++) {
            LED_Ctrl(1, 2);   // 前进 KEY_FORWORD 物理键2
            LED_Ctrl(1, 3);   // 后退 KEY_BACKWORD 物理键3
            LED_Ctrl(1, 4);   // 左转 KEY_LEFT 物理键4
            LED_Ctrl(1, 5);   // 右转 KEY_RIGHT 物理键5
            rt_thread_mdelay(100);
            LED_Ctrl(0, 2);
            LED_Ctrl(0, 3);
            LED_Ctrl(0, 4);
            LED_Ctrl(0, 5);
            rt_thread_mdelay(100);
        }
        LOG_D("Enter single mode (motion keys flash)");
    } else {
        current_work_mode = 0;
        rt_mb_send(&mbKeyMsg, KEY_NORMAL_MODE);
      // 闪烁 ROME_A、ROME_B、HOME、FREE 的 LED 三次
        for (int i = 0; i < 3; i++) {
            LED_Ctrl(1, 6);   // ROME_A 物理键6
            LED_Ctrl(1, 7);   // ROME_B 物理键7
            LED_Ctrl(1, 8);   // HOME 物理键8
            LED_Ctrl(1, 9);   // FREE 物理键9
            rt_thread_mdelay(100);
            LED_Ctrl(0, 6);
            LED_Ctrl(0, 7);
            LED_Ctrl(0, 8);
            LED_Ctrl(0, 9);
            rt_thread_mdelay(100);
        }
        LOG_D("Enter normal mode (anchor keys flash)");
    }
    Alarm(0);
}

// KEY_SWITCH 短按处理（切换小车/马桶模式）
static void handle_switch_short_press(void)
{
    if (current_mode == MODE_CAR) {
        current_mode = MODE_TOILET;
			  // 切换到马桶模式时，强制退出建图模式
        if (slam_mode) {
            slam_mode = RT_FALSE;
            LED_Ctrl(0, 0);                     // 熄灭 SLAM 按键 LED
            rt_mb_send(&mbKeyMsg, KEY_NAV_MODE); // 通知下位机进入导航模式
            LOG_D("Switch to toilet mode, forced exit mapping mode");
        }
        // LED 闪三下
        for (int i = 0; i < 3; i++) {
            LED_Ctrl(1, 10);
            rt_thread_mdelay(100);
            LED_Ctrl(0, 10);
            rt_thread_mdelay(100);
        }
				LED_Ctrl(1, 10);   // 点亮 LED11（物理键10）
				LOG_D("Switch to toilet mode");
    } else {
        current_mode = MODE_CAR;
        // LED 闪一下
        LED_Ctrl(1, 10);
        rt_thread_mdelay(100);
        LED_Ctrl(0, 10);
				LOG_D("Switch to car mode");
    }
    reset_unlock_timer();
}

/*按键处理函数：点动按键按下连续发送，普通短按，长按阀值触发*/
void KeyScan(void)
{
    rt_uint8_t row, col;
    rt_int32_t key;
    static rt_tick_t press_tick = 0;          // 记录按键按下的起始时刻
    static rt_bool_t long_press_triggered = RT_FALSE; // 本次按下是否已触发过长按
    static E_KeyMessage last_processed_msg = KEY_RELEASE;
	 static rt_int32_t last_pressed_key = KEY_NULL;   // 记录按下的物理键值，用于关闭 LED
	
   /* 1. 扫描物理按键 */
    KeyCheck(&row, &col);
    key = GetKeyCode(row, col);
	
  /* 2. 无按键变化时，处理长按检测（按键持续按下）*/
    if (key == m_dwLastKeyCode) {
        if (m_flgKeyDown) {
            // 按键持续按下状态，检测长按
            rt_tick_t pressed_time = rt_tick_get() - press_tick;
            if (!long_press_triggered && pressed_time > 0) {
                E_KeyMessage msg = Key2Msg(key);
                // 根据不同的按键设置长按阈值
                if (msg == KEY_LOCK && pressed_time >= LONG_PRESS_MS_UNLOCK) {
                    long_press_triggered = RT_TRUE;
                    // 解锁逻辑（在解锁处理函数中执行）逻辑翻转一次
                    handle_unlock();										
                }
                else if (msg == KEY_SWITCH && pressed_time >= LONG_PRESS_MS_SWITCH) {
                    long_press_triggered = RT_TRUE;
                    handle_switch_long_press();
										if (KeyUnlock) reset_unlock_timer();   // 添加重置
                }
                else if (msg == KEY_SEWAGE && pressed_time >= LONG_PRESS_MS_SEWAGE) {
                    long_press_triggered = RT_TRUE;
                    // 发送排污指令（长按有效）
                    if (KeyUnlock) {
                        rt_mb_send(&mbKeyMsg, KEY_SEWAGE);
                        Alarm(0);
												reset_unlock_timer();
                    }										
                }
            }
        }
    }
 /* 3. 按键变化（按下或释放）*/
    else {
        // 按键释放事件
        if (m_flgKeyDown) {
            // 停止点动按键的重复发送
            if (is_motion_key(last_processed_msg)) {
                stop_motion_repeat();
            }
            // 如果没有触发长按，则作为短按处理（发送一次按键消息）
            if (!long_press_triggered && last_processed_msg != KEY_RELEASE) {
								if (last_processed_msg == KEY_SWITCH) {
                    // 短按切换小车/马桶模式
										if (KeyUnlock) {
											handle_switch_short_press();
										}
                } else if (last_processed_msg == KEY_SLAM) {
										// 不发送释放消息，也不熄灭 LED（LED 状态由模式决定）
								} else if (KeyUnlock || last_processed_msg == KEY_LOCK) {
                    if (last_processed_msg != KEY_LOCK) {
											  // 非点动按键：发送释放值 0
												rt_mb_send(&mbKeyMsg, KEY_RELEASE);
                    }
                }
            }
						// 关闭对应的 LED（按下时点亮过） 排除 KEY_SWITCH 和 KEY_SLAM 和 KEY_LOCK
            if (last_pressed_key != KEY_NULL && 
									last_processed_msg != KEY_SWITCH && 
										last_processed_msg != KEY_SLAM && 
												last_processed_msg != KEY_LOCK   ) {
                LED_Ctrl(0, last_pressed_key);
            }
            // 清除状态
            m_flgKeyDown = RT_FALSE;
            long_press_triggered = RT_FALSE;
            press_tick = 0;
            last_processed_msg = KEY_RELEASE;
						last_pressed_key = KEY_NULL;
        }
        
        // 新按键按下（边沿触发）
        if (key != KEY_NULL) {
            m_flgKeyDown = RT_TRUE;
            press_tick = rt_tick_get();
            long_press_triggered = RT_FALSE;
            last_pressed_key = key;
					
            E_KeyMessage msg = Key2Msg(key);
            last_processed_msg = msg;
            
						rt_kprintf("press=[%d,%d]\r\n", key,msg);
            // 解锁键单独处理（不依赖 KeyUnlock 标志）
            if (msg == KEY_LOCK) {
                // 不立即发送，等待释放或长按
            }
            // 模式切换键（短按切换小车/马桶模式）
            else if (msg == KEY_SWITCH) {
                // 短按切换模式（在释放时处理，但这里先记录）
                // 实际上模式切换应在短按时立即执行，不依赖释放？当前逻辑在释放时未触发长按时执行。
                // 为了明确，我们在释放时执行短按切换。
            }
						// 开启排污泵特殊处理，长按有效，避免误触发
						else if (msg == KEY_SEWAGE) {
								// 不立即发送，等待长按（长按检测在按键保持期间处理）
								LED_Ctrl(1, key);
						}
            // 点动按键：立即启动重复发送，并发送第一次命令
            else if (is_motion_key(msg)) {
                if (KeyUnlock) {
										start_motion_repeat((uint8_t)msg);
										rt_mb_send(&mbKeyMsg, msg);
										reset_unlock_timer();   // 重置自动锁定计时
                    LED_Ctrl(1, key);
                    Alarm(0);
                }
            }
						// 处理 KEY_SLAM：切换建图/导航模式
						else if (msg == KEY_SLAM) {
								// 发送消息给上位机
								if (KeyUnlock) {
										slam_mode = !slam_mode;   // 切换状态
										if (slam_mode) {
												rt_mb_send(&mbKeyMsg, KEY_MAPPING_MODE);   // 发送建图模式
												LED_Ctrl(1, key);
												LOG_D("Enter mapping mode");
										} else {
												rt_mb_send(&mbKeyMsg, KEY_NAV_MODE);       // 发送导航模式
												LED_Ctrl(0, key);
												LOG_D("Enter navigation mode");
										}									
//										rt_mb_send(&mbKeyMsg, KEY_SLAM);
										reset_unlock_timer();  
										Alarm(0);
								}
						}
            // 其他普通按键
            else {
                if (KeyUnlock) {										
                    rt_mb_send(&mbKeyMsg, msg);
										reset_unlock_timer();   // 重置自动锁定计时
                    LED_Ctrl(1, key);
                    Alarm(0);
                }
            }
        }
        // 按键完全释放（key == KEY_NULL）且之前有按下状态，已经在上面处理了释放事件
    }		
						
    /* 4. 更新上次按键值 */
    m_dwLastKeyCode = key;		
}

// 跑马灯1圈
void run_leds(void)
{
		rt_pin_write(LED1, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED1, PIN_HIGH);
		rt_pin_write(LED2, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED2, PIN_HIGH);
		rt_pin_write(LED4, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED4, PIN_HIGH);
		rt_pin_write(LED6, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED6, PIN_HIGH);
		rt_pin_write(LED8, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED8, PIN_HIGH);
		rt_pin_write(LED10, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED10, PIN_HIGH);
		rt_pin_write(LED12, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED12, PIN_HIGH);
		rt_pin_write(LED11, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED11, PIN_HIGH);
		rt_pin_write(LED9, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);	
		rt_pin_write(LED9, PIN_HIGH);		
		rt_pin_write(LED7, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);	
		rt_pin_write(LED7, PIN_HIGH);
		rt_pin_write(LED5, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);	
		rt_pin_write(LED5, PIN_HIGH);
		rt_pin_write(LED3, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND*100/1000);
		rt_pin_write(LED3, PIN_HIGH);	
}

/* 底层零部件位置初始化  */
void initComponent(void)
{
	rt_uint8_t i,cnt=0;	
	rt_uint8_t row,col;
	rt_int32_t* keys;
	rt_kprintf("initComponent\r\n");
	Alarm(1);
	rt_pin_write(LED0, PIN_HIGH);
	// 跑马灯跑2圈
	for(i=0;i<2;i++){	
		run_leds();
		// 断开蓝牙连接，清除蓝牙记忆
		MulKeysCheck(&row,&col);	
		keys = GetKeysCode(row,col);
		 rt_kprintf("[%d,%d]\r\n", keys[0],keys[1]);
		if(keys[0] == 10 && keys[1] == 11){
			cnt+=1;
		}else{
			cnt = 0;
		}
		if(cnt>1){
			rt_kprintf(" disconnect Bluetooth\r\n");
			ble_disc();// 断开并清除记忆
			Alarm(1);
			rt_thread_mdelay(500);
      Alarm(0);
			cnt = 0;
		}
	}		
	LED_OFF_ALL();
	rt_pin_write(LED0, PIN_LOW);		
}
/* gpio 初始化*/
static void initGPIO(void){
	rt_pin_mode(KEY_SHUTDOWN_PIN, PIN_MODE_INPUT_PULLUP);
	
	rt_pin_mode(LIE0, PIN_MODE_INPUT);
	rt_pin_mode(LIE1, PIN_MODE_INPUT);
	rt_pin_mode(LIE2, PIN_MODE_INPUT);
	rt_pin_mode(LIE3, PIN_MODE_INPUT);
	
	rt_pin_mode(HANG0, PIN_MODE_INPUT);
	rt_pin_mode(HANG1, PIN_MODE_INPUT);
	rt_pin_mode(HANG2, PIN_MODE_INPUT);
	
	rt_pin_mode(LED1, PIN_MODE_OUTPUT);
	rt_pin_write(LED1, PIN_HIGH);	
	rt_pin_mode(LED2, PIN_MODE_OUTPUT);
	rt_pin_write(LED2, PIN_HIGH);	
	rt_pin_mode(LED3, PIN_MODE_OUTPUT);
	rt_pin_write(LED3, PIN_HIGH);	
	rt_pin_mode(LED4, PIN_MODE_OUTPUT);
	rt_pin_write(LED4, PIN_HIGH);	

	rt_pin_mode(LED5, PIN_MODE_OUTPUT);
	rt_pin_write(LED5, PIN_HIGH);	
	rt_pin_mode(LED6, PIN_MODE_OUTPUT);
	rt_pin_write(LED6, PIN_HIGH);	
	rt_pin_mode(LED7, PIN_MODE_OUTPUT);
	rt_pin_write(LED7, PIN_HIGH);	
	rt_pin_mode(LED8, PIN_MODE_OUTPUT);
	rt_pin_write(LED8, PIN_HIGH);	

	rt_pin_mode(LED9, PIN_MODE_OUTPUT);
	rt_pin_write(LED9, PIN_HIGH);	
	rt_pin_mode(LED10, PIN_MODE_OUTPUT);
	rt_pin_write(LED10, PIN_HIGH);	
	rt_pin_mode(LED11, PIN_MODE_OUTPUT);
	rt_pin_write(LED11, PIN_HIGH);	
	rt_pin_mode(LED12, PIN_MODE_OUTPUT);
	rt_pin_write(LED12, PIN_HIGH);	

//	rt_pin_mode(LED13, PIN_MODE_OUTPUT);
//	rt_pin_write(LED13, PIN_HIGH);	
//	rt_pin_mode(LED14, PIN_MODE_OUTPUT);
//	rt_pin_write(LED14, PIN_HIGH);	
//	rt_pin_mode(LED15, PIN_MODE_OUTPUT);
//	rt_pin_write(LED15, PIN_HIGH);	
//	rt_pin_mode(LED16, PIN_MODE_OUTPUT);
//	rt_pin_write(LED16, PIN_HIGH);

	rt_pin_mode(ALARM, PIN_MODE_OUTPUT);
	rt_pin_write(ALARM, PIN_HIGH);


	rt_pin_mode(LED0, PIN_MODE_OUTPUT);
	rt_pin_write(LED0, PIN_LOW);	
}

uint32_t read_vdda_mv(void)
{
	    if (adc_dev == RT_NULL) return 0;
    uint32_t vrefint_data = 0;
    for (int i = 0; i < 10; i++) {
        vrefint_data += rt_adc_read(adc_dev, 17);  // 通道 17
    }
    vrefint_data /= 10;  // 取平均值以减少误差

    /* VDDA(mV) = VREFINT_CAL(mV) * 4096 / VREFINT_DATA */
    return (VREFINT_CAL_MV * 4096) / vrefint_data;
}

uint16_t read_battery_mv(void)
{
    if (adc_dev == RT_NULL) return 0;
	/* 1. 读取实际的 VDDA */
    uint32_t vdda_mv = read_vdda_mv();
	
    /* 2. 读取 PC1 通道（通道 11）的 ADC 原始值 */
    uint32_t channel_data = rt_adc_read(adc_dev, 11);
    /* 3. 根据当前的 VDDA 换算 */
    uint32_t pc1_mv = (channel_data * vdda_mv) / 4096;

    /* 4. 还原实际电池电压（两个10k电阻分压，比例2） */
    return (uint16_t)(pc1_mv * 2);
}


void adc_init(void)
{
    /* 查找ADC设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL) {
        LOG_E("ADC device not found");
        return;
    }
    /* 使能通道11 (PC1) */
    rt_adc_enable(adc_dev, 11);
}

static void key_scan_thread_entry(void *param)
{
		int cnt=0;
		initComponent();
	
    while (1) {
        KeyScan();           // 执行按键扫描			
			
			  // 检测独立关机按键 PC15
        rt_bool_t key_pressed = (rt_pin_read(KEY_SHUTDOWN_PIN) == PIN_LOW);
				if (key_pressed) {
							if (shutdown_press_start == 0) {
									// 刚按下，记录起始时间
									shutdown_press_start = rt_tick_get();
									shutdown_triggered = RT_FALSE;
							} else if (!shutdown_triggered) {
									// 检查是否超过阈值（例如 1.5 秒）
									rt_tick_t elapsed = rt_tick_get() - shutdown_press_start;
									if (elapsed >= rt_tick_from_millisecond(SHUTDOWN_HOLD_MS)) {
											shutdown_triggered = RT_TRUE;
											rt_kprintf("Long press shutdown, power off now!\n");
											Alarm(1);
											rt_pin_write(PWR_HOLD_PIN, PIN_LOW);
											// 等待硬件断电（此后的代码不会执行）
											rt_thread_mdelay(1000); // 等待电路板电容电释放完
											rt_kprintf("?????\n");
									}
							}
					} else {
							// 按键释放，重置计时
							shutdown_press_start = 0;
							shutdown_triggered = RT_FALSE;
					}
			
				// 低电压检测
				cnt++;
				if(cnt >= 3000){		// 1min
						cnt = 0;
					  uint16_t mv = read_battery_mv();
						LOG_D("Battery voltage: %d mV", mv);
						if (mv < 3000) {  // 低于3.0V提醒    锂电池电压范围2.75-4.2V 标称3.7V
								// 闪烁红灯或发送低电量指令
							Alarm(1);
						}
				}
        rt_thread_mdelay(20); // 20ms 采样一次
    }
}

// 指示灯线程函数
static void ble_indicator_thread_entry(void *param)
{
    while (1) {
        if (!ble_connected) {
            // 蓝牙断开，持续跑马灯直到连接成功
            while (!ble_connected) {
                run_leds();  // 执行一圈跑马灯（内部有延时，不会忙等）
            }
            // 连接成功，熄灭所有 LED
            LED_OFF_ALL();
            rt_kprintf("Bluetooth reconnected, indicator off.\n");
        } else {
            // 已连接，休眠 2 秒后再次检查（避免频繁空转）
            rt_thread_mdelay(2000);
        }
    }
}

/***************** config ********************************/
/*配置板载IO资源*/
void configBoard(void)
{	   
	initGPIO();					/*!< 管脚 初始化 */
	adc_init();
//	initPWM();					/*!< 压电式蜂鸣器 PWM 初始化 */
  unlock_timer=rt_timer_create("unlock timer",unlock_timeout,RT_NULL, UNLOCK_TIME_S*RT_TICK_PER_SECOND,RT_TIMER_FLAG_ONE_SHOT);    /* 创建定时器  单次定时器 */
	// 初始化邮箱
	rt_mb_init(&mbKeyMsg,"key mb",mbKeyPool,sizeof(mbKeyPool)/sizeof(rt_uint32_t),RT_IPC_FLAG_FIFO);	
	
	motion_repeat_timer = rt_timer_create("motion_tmr", (void(*)(void*))motion_repeat_cb,RT_NULL,                                              
                                       MOTION_REPEAT_INTERVAL_MS,
                                       RT_TIMER_FLAG_PERIODIC);
	if (motion_repeat_timer == RT_NULL) {
			LOG_E("Failed to create motion repeat timer");
	}
	/* 创建线程 */
    KEYThread = rt_thread_create("KEY", key_scan_thread_entry, RT_NULL, RT_KEY_THREAD_STACK_SIZE, RT_KEY_THREAD_PRIORITY, 10);
   if (KEYThread == RT_NULL)
   {
       rt_kprintf("create KEYThread failed!\n");
   }else{
			rt_thread_startup(KEYThread);
		}
	 
		// 创建指示灯线程（优先级低于按键扫描线程）
		rt_thread_t ind_thread = rt_thread_create("ble_ind",
																							ble_indicator_thread_entry,
																							RT_NULL,
																							1024,
																							RT_KEY_THREAD_PRIORITY + 1,  // 比 KEY 线程低一级
																							10);
		if (ind_thread) rt_thread_startup(ind_thread);
}

// 中断服务函数
static void key_wakeup_isr(void *args)
{
    // 清除中断标志（RT-Thread 的 pin 驱动会自动处理，这里只需做必要标志）
    // 可选：设置一个全局标志，通知主循环已经唤醒
    // 由于我们只是要退出 WFI，不需要额外操作
}

// 恢复按键扫描 GPIO（与 initGPIO 中按键部分相同，但不影响其他外设）
static void restore_keyboard_gpio(void)
{
    // 重新初始化按键引脚为扫描模式（参考 initGPIO 中的设置）
    rt_pin_mode(LIE0, PIN_MODE_INPUT);
    rt_pin_mode(LIE1, PIN_MODE_INPUT);
    rt_pin_mode(LIE2, PIN_MODE_INPUT);
    rt_pin_mode(LIE3, PIN_MODE_INPUT);
    
    rt_pin_mode(HANG0, PIN_MODE_INPUT);
    rt_pin_mode(HANG1, PIN_MODE_INPUT);
    rt_pin_mode(HANG2, PIN_MODE_INPUT);
    
    // 关闭中断
    rt_pin_irq_enable(HANG0, PIN_IRQ_DISABLE);
    rt_pin_irq_enable(HANG1, PIN_IRQ_DISABLE);
    rt_pin_irq_enable(HANG2, PIN_IRQ_DISABLE);
}

void enter_low_power_with_key_wakeup(void)
{
    // 1. 将所有列线（LIE0~LIE3）设置为输出低电平
    rt_pin_mode(LIE0, PIN_MODE_OUTPUT);
    rt_pin_mode(LIE1, PIN_MODE_OUTPUT);
    rt_pin_mode(LIE2, PIN_MODE_OUTPUT);
    rt_pin_mode(LIE3, PIN_MODE_OUTPUT);
    rt_pin_write(LIE0, PIN_LOW);
    rt_pin_write(LIE1, PIN_LOW);
    rt_pin_write(LIE2, PIN_LOW);
    rt_pin_write(LIE3, PIN_LOW);
    
    // 2. 将行线配置为下降沿中断（HANG0~HANG2）
    // 注意：仅配置需要唤醒的行，但为了简单，将所有行都配置为中断
    rt_pin_mode(HANG0, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(HANG1, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(HANG2, PIN_MODE_INPUT_PULLUP);
    
    rt_pin_attach_irq(HANG0, PIN_IRQ_MODE_FALLING, key_wakeup_isr, RT_NULL);
    rt_pin_attach_irq(HANG1, PIN_IRQ_MODE_FALLING, key_wakeup_isr, RT_NULL);
    rt_pin_attach_irq(HANG2, PIN_IRQ_MODE_FALLING, key_wakeup_isr, RT_NULL);
    
    rt_pin_irq_enable(HANG0, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(HANG1, PIN_IRQ_ENABLE);
    rt_pin_irq_enable(HANG2, PIN_IRQ_ENABLE);

// 注意不要用rt_thread_delay(),会导致唤醒不了
    // 3. 进入 WFI 模式
    // 确保进入的是普通睡眠模式
		    // 临时关闭 SysTick 中断（避免每 1ms 唤醒一次）
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
//    // 临时关闭 UART2 接收中断（避免蓝牙模块的扫描数据唤醒）
    HAL_NVIC_DisableIRQ(USART2_IRQn);
		
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();  // 数据同步屏障，确保存储操作完成
    __WFI();

    // 4. 唤醒后清除中断挂起标志
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);   // PB4 (HANG0)
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);   // PB5 (HANG1)
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);   // PB6 (HANG2)

		// 5. 唤醒后恢复中断
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    // 6. 唤醒后，恢复按键扫描模式
    // 注意：不能直接调用 configBoard 因为它会重置所有外设，包括串口等。应该只恢复按键相关 GPIO。
    restore_keyboard_gpio();
		
}



