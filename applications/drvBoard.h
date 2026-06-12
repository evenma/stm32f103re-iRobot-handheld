/*
 * Copyright (c) 2006-2018,  qiangying technology intelligent bed designer group
 *
 *  
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-9-26      mayb     first version
 * 2022-5-17      ---			         
 */

#ifndef __DRV_BOARD_H__
#define __DRV_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>

//按键定义
typedef enum
{
  KEY_RELEASE=0,			 // 按键释放
/*小车按键*/	
	KEY_SLAM = 50,				//1 建图和导航切换， 默认导航， 建图时LED灯常亮
	KEY_STOPWORK ,				//2 停止动作，不区分小车运动和智能马桶动作，全部停		
	
	KEY_FORWORD,					//3 前进，点动，按着动作，放开停
	KEY_BACKWORD,         //4 后退，点动，按着动作，放开停
	KEY_LEFT,             //5 左转，点动，按着动作，放开停
	KEY_RIGHT,						//6 右转，点动，按着动作，放开停
	
	KEY_ROME_A,						//7 正常模式下，去房间A--卧室   建图时设锚点坐标
	KEY_ROME_B,						//8 正常模式下，离开房间A--去厕所，避免房间A有臭味    建图时设锚点坐标
	KEY_HOME,							//9 回充电座，正常模式的发送指令和 单机模式 复用同一个按键	 建图时设锚点坐标
	KEY_FREE,							//10 手动推车，打开和关闭

/*智能马桶按键*/
	KEY_FLUSH = 100	,			//1 冲洗马桶	
	KEY_STOP,							//2 停止动作
	
	KEY_BIANMEN_KAI,			//3 马桶盖打开，由于按键不够，复用单机模式下的左转
	KEY_BIANMEN_GUAN,			//4 马桶盖关闭，由于按键不够，复用单机模式下的左转
	KEY_CLEAN_REAR,				//5 清洁臀部 
	KEY_CLEAN_FEMALE,			//6 清洁女性
	
	KEY_CLEAN_MODE,				//7 清洁时固定模式和按摩模式切换
	KEY_CLEAN_STRENGTH,		//8 清洁力度调节	1-3档位 按一下切一次
	KEY_GANZAO,						//9 暖风烘干		
	KEY_SEWAGE,						//10 长按5-6秒才能启动污物泵，防止误操作导致污物冲出到房间里。

/*其他*/
	KEY_SWITCH = 200,			// KEY_SWITCH短按触发，切换按键功能，前十个按键控制小车(前10个LED闪一下)或者控制智能马桶(前10个LED闪三下)，
	KEY_LOCK,							// 长按1-2秒解锁	
	KEY_SINGLE_MODE = 0xF0,  // KEY_SWITCH长按触发 小车操作单机模式，下位机控制小车,长按5-6秒切换单机模式 默认正常模式 ，单机模式点亮LED灯(前进/后退/左转/右转/回充电座)
	KEY_NORMAL_MODE ,
	KEY_MAPPING_MODE= 0xF2,	 // 建图模式
	KEY_NAV_MODE,						 // 导航模式
	
}E_KeyMessage;

#define KEY_NULL	-1

#define	HANG_MAX	3 		//按键扫描行数
#define	LIE_MAX		4 		//按键扫描列数
	
#define LIE0	 GET_PIN(B, 3)    	// col0 L0  列0
#define LIE1	 GET_PIN(D, 2)    	// col0 L1  列1
#define LIE2	 GET_PIN(C, 12)    	// col0 L2  列2
#define LIE3	 GET_PIN(A, 15)    	// col0 L3  列3

#define	HANG0	 GET_PIN(B, 4)    	// row0 R0  行0	
#define	HANG1	 GET_PIN(B, 5)    	// row0 R1  行1	
#define	HANG2	 GET_PIN(B, 6)    	// row0 R2  行2	
#define	HANG3	 GET_PIN(B, 7)    	// row0 R3  行3	预留
#define	HANG4	 GET_PIN(B, 8)    	// row0 R4  行4	外接

#define LED1	 GET_PIN(A, 8)    	// 指示灯D1 按键SW1  	NET LED1
#define LED2	 GET_PIN(C, 9)    	// 指示灯D3 按键SW2   NET LED2
#define LED3	 GET_PIN(C, 8)    	// 指示灯D5 按键SW3   NET LED3
#define LED4	 GET_PIN(C, 7)    	// 指示灯D7 按键SW4   NET LED4
#define LED5	 GET_PIN(C, 2)    	// 指示灯D9 按键SW5   NET LED5
#define LED6	 GET_PIN(B, 9)    	// 指示灯D11 按键SW6  NET LED6
#define LED7	 GET_PIN(C, 3)    	// 指示灯D13 按键SW7  NET LED7
#define LED8	 GET_PIN(A, 5)    	// 指示灯D15 按键SW8  NET LED8
#define LED9	 GET_PIN(A, 6)    	// 指示灯D2 按键SW9		NET LED9
#define LED10	 GET_PIN(A, 7)    	// 指示灯D4 按键SW10  NET LED10
#define LED11	 GET_PIN(C, 4)    	// 指示灯D6 按键SW11  NET LED11
#define LED12	 GET_PIN(C, 5)    	// 指示灯D8 按键SW12  NET LED12

#define LED13	 GET_PIN(B, 0)    	// 指示灯D10 预留     NET LED13
#define LED14	 GET_PIN(B, 1)    	// 指示灯D12 预留     NET LED14
#define LED15	 GET_PIN(B, 10)    	// 指示灯D14 预留     NET LED15
#define LED16	 GET_PIN(B, 11)    	// 指示灯D16 预留     NET LED16

#define ALARM	 GET_PIN(A, 1)    	// 直流蜂鸣器 如果选用压电式蜂鸣器，可设置为PWM TIME2_CH2

// 锁键对应的行和列引脚
#define KEY_LOCK_ROW    HANG2   // PB6
#define KEY_LOCK_COL    LIE3    // PA15

#define PWM_DEV_NAME        "pwm2"  	/* PWM设备名称  time2 ch2 */
#define PWM_DEV_CHANNEL_2     2       	/* PWM通道 */
#define PWM_TIMER			500000UL    /* 2K 周期为500us，单位为纳秒ns */
#define PWM_DUTY    		5000UL		/* PWM_PERIOD/100;*/

#define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 */

#define LED0	 GET_PIN(A, 0)    	// 工作指示灯D21
//#define BLE_nRST    GET_PIN(A,4)
#define PWR_HOLD_PIN	 GET_PIN(C, 0)
#define KEY_SHUTDOWN_PIN	 GET_PIN(C, 15)

/* VREFINT 典型值（单位mV）*/
#define VREFINT_CAL_MV    1200

//extern rt_int32_t g_KeyMsgDown;	// 当前被按下的按键消息

void KeyTick(void);	// 按键扫描定时显示

typedef enum {
    MODE_CAR,
    MODE_TOILET
} eWorkMode;

extern eWorkMode current_mode;       

typedef enum {
    eState_initialize = 1, /**< State machine: Reset and initialize module */
    eState_normal = 2, /**< State machine: application running  */
    eState_fault = 3, /**< State machine: Drop in an unexpected error */
}Device_StateTypeDef;

extern Device_StateTypeDef device_state;		// 系统状态
/**
  * @brief  配置硬件资源
  * @param  none
  * @retval None
  */
void configBoard(void);
/**
  * @brief  零部件归位或清零或停止
  * @param  none
  * @retval None
  */
void initComponent(void);

/**
  * @brief  蜂鸣器
  * @param  status 0:正常蜂鸣  1:错误蜂鸣
  * @retval None
  */
void Alarm(rt_uint8_t status);

void enter_low_power_with_key_wakeup(void);

extern struct rt_mailbox mbKeyMsg;   // 邮件 按键触发后BLE发送给上位机
extern rt_bool_t slam_mode;
extern uint8_t current_work_mode;   // 0=正常模式, 1=单机模式

#ifdef __cplusplus
}
#endif

#endif /* __DRV_BOARD_H__ */

/************************** end of file ******************/

