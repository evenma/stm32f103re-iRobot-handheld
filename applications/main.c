/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "fal.h"
#include "drvBoard.h"
#include "car_ble_ctrl.h"

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "main"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif /* ULOG_USING_SYSLOG */

/**			thread priority
BLE_THREAD_PRIORITY   (RT_THREAD_PRIORITY_MAX / 3 + 4)		= 14
RT_LED_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX - 5) = 27
idle.c		(RT_THREAD_PRIORITY_MAX - 2)										= 30
tshell																										= 20
can
*/

#define APP_VERSION "1.0.0"


//#define RT_LED_THREAD_STACK_SIZE     512
//#define RT_LED_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX - 5)
//static struct rt_thread LEDThread;
//static char LEDthread_stack[RT_LED_THREAD_STACK_SIZE];

#define BLE_THREAD_STACK_SIZE     2048
#define BLE_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 3 + 4)
static struct rt_thread bleThread;
static char bleThread_stack[BLE_THREAD_STACK_SIZE];

/* defined the LED0 pin: PA0 */
#define LED0_PIN    GET_PIN(A, 0)

rt_uint8_t enableDebug = 1;


//static void LEDThread_Entry(void* parameter)
//{
//	while (1)
//    {
//        rt_pin_write(LED0_PIN, PIN_HIGH);
//        rt_thread_mdelay(500);
//        rt_pin_write(LED0_PIN, PIN_LOW);
//        rt_thread_mdelay(500);
//    }
//}
/* 开机时立即将 PWR_HOLD 拉高，维持电源 */
static void power_hold_init(void)
{
    rt_pin_mode(PWR_HOLD_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PWR_HOLD_PIN, PIN_HIGH);   // 高电平维持供电
}

int main(void)
{
		rt_err_t ret = RT_EOK;
	// 电源开机管脚
	power_hold_init();
	
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
/* fal文件系统分区 bootloader相关 用于OTA */
	fal_init();
	if(enableDebug){
	LOG_D("/****************************************************/");
	LOG_D("The current version of APP fireware is iHomeRobot-handheld-V%s",APP_VERSION);
	LOG_D("/****************************************************/\n");
	}

// 硬件初始化
	configBoard();	
/* LED thread for debug*/
//	ret = rt_thread_init(&LEDThread,
//		"LED",
//		LEDThread_Entry, RT_NULL,
//		&LEDthread_stack[0], sizeof(LEDthread_stack),
//		RT_LED_THREAD_PRIORITY, 10);

//	if (ret == RT_EOK){
//		rt_thread_startup(&LEDThread);
//    }else{
//        LOG_D("thread ACTION init failed!\n");	
//	}
		
	/* ble thread and service logic process*/
    ret = rt_thread_init(&bleThread,
        "car_ble",
        car_ble_thread_entry, RT_NULL,
        &bleThread_stack[0], sizeof(bleThread_stack),
        BLE_THREAD_PRIORITY, 10);

    if (ret == RT_EOK){
        rt_thread_startup(&bleThread);
		}else{
        LOG_D("thread BLE init failed!\n");	
		}
		
		
  return RT_EOK;
}

static int debugON(int argc, char *argv[])
{
	rt_uint8_t enable = 0;
	enable = (rt_uint8_t) atoi(argv[1]);
	if(enable){
		enableDebug = 1;
	}else{
		enableDebug = 0;
	}
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(debugON, debug On);


/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);
