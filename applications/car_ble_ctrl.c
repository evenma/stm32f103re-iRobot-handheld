/**
 ******************************************************************************
 * @file    car_ble_ctrl.c
 * @brief   Car remote control Bluetooth master mode implementation
 ******************************************************************************
 */

#include "car_ble_ctrl.h"
#include "dx2002_at.h"
#include "ATCmdParser.h"
#include "drvBoard.h"
#include <rtthread.h>
#include <string.h>
#include <stdio.h>

#ifndef ULOG_USING_SYSLOG
#define LOG_TAG              "car_ble"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#else
#include <syslog.h>
#endif

/* Debug macro */
#define BLE_DBG(fmt, ...)   LOG_D(fmt, ##__VA_ARGS__)

/* Connection retry interval (milliseconds) */
#define CONN_RETRY_INTERVAL 5000

/* State machine */
typedef enum {
    BLE_STATE_UNINIT,
    BLE_STATE_INIT,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_ERROR
} ble_state_t;

static ble_state_t ble_state = BLE_STATE_UNINIT;
static struct rt_mailbox *key_mb = &mbKeyMsg;  /* external mailbox from drvBoard */

static rt_timer_t idle_timer;
rt_bool_t ble_connected = RT_FALSE;   // 蓝牙连接状态
static volatile rt_bool_t is_sleeping = RT_FALSE;     // 是否已进入低功耗（蓝牙断开）
static volatile rt_bool_t need_disconnect = RT_FALSE;  

#define TARGET_SLAVE_NAME   "QYRobotS"   // 要连接的从机名称
//#define TARGET_SLAVE_NAME   "DX2002"
#define SCAN_INTERVAL       160          // 扫描间隔 (0.625ms units) 默认160=100ms
#define SCAN_WINDOW         80           // 扫描窗口 (0.625ms units) 默认80=50ms
/* Configuration - modify according to your car slave MAC address */
#define CAR_SLAVE_MAC       "112233445566"   /* 12 hex digits */
static rt_bool_t connecting = RT_FALSE;

/* Forward declarations */
static rt_err_t ble_set_master_mode(void);
static rt_err_t ble_connect_to_car(void);
static void ble_send_key(uint8_t key_msg);
static void ble_handle_connection(void);

/* 重置空闲定时器（每次按键时调用） */
static void reset_idle_timer(void)
{
    if (idle_timer) {
        rt_timer_stop(idle_timer);
        rt_timer_start(idle_timer);
    }
}
/* 空闲超时回调：断开蓝牙并进入低功耗 */
static void idle_timeout(void *param)
{
//     is_sleeping = RT_TRUE;
////		need_disconnect = RT_TRUE;   // 请求断开蓝牙
	    // 拉低电源保持引脚，实现断电
    rt_pin_write(PWR_HOLD_PIN, PIN_LOW);
    // 等待断电（此后的代码不会执行）
}

// 连接/断开事件回调
static void ble_conn_callback(void)
{
    ble_connected = RT_TRUE;
		connecting = RT_FALSE;

	    // 发送工作模式（正常/单机）
    if (current_work_mode == 1) {
        ble_send_key(KEY_SINGLE_MODE);
    } else {
        ble_send_key(KEY_NORMAL_MODE);
    }
    rt_thread_mdelay(10);  // 短暂延时，避免下位机处理不过来
    
    // 发送建图/导航模式
    if (slam_mode) {
        ble_send_key(KEY_MAPPING_MODE);
    } else {
        ble_send_key(KEY_NAV_MODE);
    }
		
    BLE_DBG("Bluetooth connected");
}

static void ble_disc_callback(void)
{
    ble_connected = RT_FALSE;
		connecting = RT_FALSE;
    BLE_DBG("Bluetooth disconnected");
}

static void scan_result_callback(void)
{
    // 读取一行数据，格式: NAME:xxx,<MAC>,<TYPE>,<RSSI>,MANU:xx
    char line[128];
    if (!ATCmdParser_recv("%[^\r]\r\n", line)) {
        return;
    }
		rt_kprintf("Scan:NAME:%s\r\n",line);
    // 解析名称和 MAC
    char name[32] = {0};
    char mac[13] = {0};
    // 简单解析： NAME:xxxx,112233445566,...
    if (sscanf(line, "%[^,],%12[^,],", name, mac) == 2) {
        BLE_DBG("Scan: name=%s, mac=%s", name, mac);
        if (strncmp(name, TARGET_SLAVE_NAME, strlen(TARGET_SLAVE_NAME)) == 0 && !connecting) {
						connecting = RT_TRUE;
            // 找到目标从机，停止扫描（可选）并连接
            BLE_DBG("Found target slave %s, connecting...", mac);
            // 注意：连接前可能需要停止扫描，但连接指令本身会处理
            if (at_module_set_CONN(mac) == kNoErr) {
                // 设置自动连接后，模块会自动尝试连接
                 // 连接成功后，模块会进入透传模式，并输出 IM_CONN
            }
        }
    }
}
/*---------------------------------------------------------------------------
 *  Initialize AT command parser and set module to master mode
 *---------------------------------------------------------------------------*/
static rt_err_t car_ble_init_once(void)
{
    rt_err_t ret = RT_EOK;
    
    BLE_DBG("Initializing Bluetooth master mode...");    
    
    /* Hard reboot module to ensure clean state */
    ble_module_Hard_reboot();
//    rt_thread_mdelay(500);

    // 先允许 OOB 处理（因为按键线程已经在扫描，此处只需短暂等待）
    // 注意：此时 at_mode 可能为 1（AT模式），OOB 不处理。我们需要临时设为 0 以允许 OOB
		BLE_DBG("wait connect if not first");
    ATCmdParser_set_mode(0);
    int wait_cnt = 500;  // 等待最多 5S (20ms * 50)
    while (wait_cnt-- && !ble_connected) {
				rt_kprintf(".");
        rt_thread_mdelay(20);
    }
    if (ble_connected) {
        BLE_DBG("Already connected (auto-reconnect), skip AT init.");
        return RT_EOK;   // 直接成功，不发送任何 AT 指令
    }	
		BLE_DBG("no connect,AT mode");
    // 未连接，切回 AT 模式进行正常初始化
    ATCmdParser_set_mode(1);
		
  /* Wait for module to enter AT mode */
    if (waitReady() != 0) {
        LOG_E("Module not responding after reboot");
        ble_state = BLE_STATE_ERROR;
        return RT_ERROR;
    }

		/*发起连接并自动扫描 */
		at_module_set_PINCODE("1234");
		at_module_set_SETSCANP(SCAN_INTERVAL, SCAN_WINDOW);		
		at_module_get_fw_version();		
    /* Set master mode 设为主模式，此时模块将重启，自动发起扫描*/
    if (ble_set_master_mode() != RT_EOK) {
        LOG_E("Failed to set master mode");
        ble_state = BLE_STATE_ERROR;
        return RT_ERROR;
    }	
    if (ble_connected) {
        BLE_DBG("Already connected (auto-reconnect), skip AT init.");
        return RT_EOK;   // 直接成功，不发送任何 AT 指令
    }			
						
		/*获取模块所有信息 */
		if (ble_module_init() != kNoErr) {
        LOG_E("Failed to read module info");
        return RT_ERROR;
    }
		
    /* Switch to transparent mode (auto after connection) */
    ATCmdParser_set_mode(0);
    
    ble_state = BLE_STATE_INIT;
    BLE_DBG("Bluetooth master initialized, waiting for connection...");

    return RT_EOK;
}

rt_err_t car_ble_init(void)
{
    int retry = 3;
    rt_err_t err;

    while (retry--) {
        err = car_ble_init_once();
        if (err == RT_EOK) {
            return RT_EOK;
        }
        LOG_W("BLE init failed, retrying... (%d retries left)", retry);
        rt_thread_mdelay(1000);  // 等待1秒后重试
    }

    LOG_E("BLE init failed after multiple attempts");
    ble_state = BLE_STATE_ERROR;
    return RT_ERROR;
}

/*---------------------------------------------------------------------------
 *  Set module as master
 *---------------------------------------------------------------------------*/
static rt_err_t ble_set_master_mode(void)
{
    uint8_t mode;
    int retry = 3;
    
    /* Check current mode */
    mode = at_module_get_MASTER();
		BLE_DBG("mode=%d",mode);
//		if(mode){
//				BLE_DBG("Already in master mode");
//				return RT_EOK;
//		}

    /* Set master mode */
    while (retry--) {
        if (at_module_set_MASTER(1) == kNoErr) {
            BLE_DBG("Master mode set, module will reboot...");
            /* Module reboots automatically, wait for it */
            rt_thread_mdelay(1500);
            /* Re-enter AT mode */
            if (waitReady() == 0) {
                return RT_EOK;
            }
        }
        rt_thread_mdelay(500);
    }
    return RT_ERROR;
}

/*---------------------------------------------------------------------------
 *  Configure automatic connection to car slave
 *---------------------------------------------------------------------------*/
static rt_err_t ble_connect_to_car(void)
{
    char *saved_mac;
    int retry = 3;
    
    /* Check if already set */
    saved_mac = (char*)at_module_get_CONN();
		BLE_DBG("get MAC = %s", saved_mac);
    if (saved_mac && strcmp(saved_mac, CAR_SLAVE_MAC) == 0) {
        BLE_DBG("MAC already set to %s", CAR_SLAVE_MAC);
        return RT_EOK;
    }
    
    while (retry--) {
        if (at_module_set_CONN((char*)CAR_SLAVE_MAC) == kNoErr) {
            BLE_DBG("Connection MAC set to %s", CAR_SLAVE_MAC);
            return RT_EOK;
        }
        rt_thread_mdelay(500);
    }
    return RT_ERROR;
}

/*---------------------------------------------------------------------------
 *  Send key value over Bluetooth (raw data)
 *---------------------------------------------------------------------------*/
static void ble_send_key(uint8_t key_msg)
{
    uint8_t buf[2];
    
    /* Simple protocol: send one byte key code */
    buf[0] = (uint8_t)(key_msg & 0xFF);
    /* Optionally add second byte for extended keys */
    
//    if (ATCmdParser_write((const char*)buf, 1) != 1) {
	
    // 发送格式：QY=<key_msg>\r\n
    if (ATCmdParser_send("QY=%d", key_msg)) {	
//        BLE_DBG("Key %d sent", key_msg);
    } else {        
			BLE_DBG("Failed to send key %d", key_msg);
    }
}

/*---------------------------------------------------------------------------
 *  Handle connection state (monitor IM_CONN/IM_DISC events)
 *  This is called from OOB callback when module sends unsolicited messages.
 *---------------------------------------------------------------------------*/
static void ble_on_connection_event(void)
{
    char arg[16];
    /* The OOB callback will be invoked by ATCmdParser_process_oob()
     * The actual parsing is done in ble_conn_event_handler (original),
     * but we can implement a simple handler here.
     * However, for simplicity, we just rely on the fact that when connected,
     * ATCmdParser_process_oob() will read data and we will send keys.
     * To detect connection, we can check if we can receive any data or check
     * module status via AT commands. But for this project, we don't need
     * explicit connection state; we just attempt to send keys.
     * If disconnected, the module will automatically attempt to reconnect
     * because we set AT+CONN.
     */
    BLE_DBG("Connection event received");
}

/*---------------------------------------------------------------------------
 *  Main loop: wait for keys from mailbox and send them
 *---------------------------------------------------------------------------*/
static void ble_main_loop(void)
{
    rt_ubase_t  key_msg = 0;
    
    while (1) {
        /* Wait for key press from mailbox (blocking) RT_WAITING_FOREVER */
        if (rt_mb_recv(&mbKeyMsg, &key_msg,20 ) == RT_EOK) {
						if(key_msg != KEY_RELEASE){
								reset_idle_timer();        // 有按键时重置计时							 
						}
            // 发送数据
            if (ble_connected) {
                ble_send_key((uint8_t)key_msg);
            } else {
                LOG_W("Key %d dropped, Bluetooth not ready", key_msg);
								Alarm(1);
            }
        }
				// 处理断开请求 实测功耗更大
//        if (need_disconnect && ble_connected) {
//            LOG_I("Disconnecting Bluetooth for low power...");
//            at_module_DSCET();           // 断开连接并清除记忆（模块会重启）
//            ble_connected = RT_FALSE;
//            need_disconnect = RT_FALSE;
//            // 可选：设置模块进入深睡眠（如果支持）
//            // at_module_set_AUST(5);     // 主机模式下无效，但可尝试
//            is_sleeping = RT_TRUE;       // 允许进入深度睡眠
//        }

				// 检查是否需要进入低功耗
//        if (is_sleeping) {
//            LOG_I("Enter low power mode (MCU sleep only)");
//            enter_low_power_with_key_wakeup();  // 内部调用 __WFI()
//            LOG_I("Wake up from low power");
//            // 唤醒后重置标志
//            is_sleeping = RT_FALSE;
//            reset_idle_timer();
//        }
				
				// 直接用关机去掉低功耗模式，在中断中实现
				
    }
}

void ble_oob_scan(void)
{
		while (ATCmdParser_process_oob());
}

static void ble_oob_thread_entry(void *param)
{
    while (1) {
        ATCmdParser_process_oob();
        rt_thread_mdelay(10);   // 10ms 轮询，及时处理
    }
}

/*---------------------------------------------------------------------------
 *  Thread entry
 *---------------------------------------------------------------------------*/
void car_ble_thread_entry(void *parameter)
{
    rt_err_t err;
	    /* Initialize AT command parser (UART2, timeout 1000ms) */
    ATCmdParser_init("\r\n", "\r\n", 1000);
	
    /* Add OOB handler for connection events (optional) */
    ATCmdParser_add_oob("IM_CONN", ble_conn_callback);
    ATCmdParser_add_oob("IM_DISC", ble_disc_callback);
		ATCmdParser_add_oob("NAME:", scan_result_callback);
		rt_thread_t oob_thread = rt_thread_create("ble_oob",
																							ble_oob_thread_entry,
																							RT_NULL,
																							4096,
																							16 ,
																							10);
		if (oob_thread) rt_thread_startup(oob_thread);
	
    /* Initialize Bluetooth */
    car_ble_init();
    
    /* 创建空闲定时器：5分钟超时 */
    idle_timer = rt_timer_create("idle_tmr", idle_timeout, RT_NULL,
                                  10 * 60 * RT_TICK_PER_SECOND,
                                  RT_TIMER_FLAG_ONE_SHOT);
    if (idle_timer) rt_timer_start(idle_timer);
		
    /* Enter main loop */
    ble_main_loop();
}

int ble_disc(void)
{
	//主机发送AT+DSCET=1 断开连接时，会清除记忆的从机蓝牙，并且重启
    if (at_module_DSCET() == kNoErr) {
        LOG_I("Bluetooth disconnected by user command");
        ble_connected = RT_FALSE;
        connecting = RT_FALSE;
        return 0;
    } else {
        LOG_E("Failed to disconnect");
        return -1;
    }
}

static int ble_disconnect(int argc, char **argv)
{
		return ble_disc();
}
MSH_CMD_EXPORT(ble_disconnect, disconnect Bluetooth);

static int ble_connect(int argc, char **argv)
{
    if (argc != 2) {
        LOG_I("Usage: ble_connect <mac>");
        return -1;
    }
    if (at_module_set_CONN(argv[1]) == kNoErr) {
        LOG_I("Connecting to %s...", argv[1]);
        return 0;
    } else {
        LOG_E("Connect failed");
        return -1;
    }
}
MSH_CMD_EXPORT(ble_connect, connect to BLE MAC);

static int ble_status(int argc, char **argv)
{
    LOG_I("ble_connected = %d, connecting = %d", ble_connected, connecting);
    return 0;
}
MSH_CMD_EXPORT(ble_status, show BLE connection status);
