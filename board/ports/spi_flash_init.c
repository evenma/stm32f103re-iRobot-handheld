/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     SummerGift   add spi flash port file
 */

#include <rtthread.h>
#include "spi_flash.h"
#include "spi_flash_sfud.h"
#include "drv_spi.h"

#if defined(BSP_USING_SPI_FLASH)
static int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_12);

    if (RT_NULL == rt_sfud_flash_probe(FAL_USING_NOR_FLASH_DEV_NAME, "spi20"))   // "W25Q64CV"
    {
        return -RT_ERROR;
    };

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(rt_hw_spi_flash_init);
#endif


void W25Q_Flash_test(void)
{
    sfud_err result;
    uint8_t *read_data;  // 读取到的数据
    uint8_t *write_data; // 将要写入的数据
    sfud_flash *sfud_dev = NULL;
 
    sfud_dev = rt_sfud_flash_find("spi20"); // 获取 sfud_dev
    // 或者 sfud_dev = rt_sfud_flash_find_by_dev_name("W25Q128");
 
    /*擦除 Flash 数据；flash： Flash 设备对象；addr：起始地址；size：从起始地址开始擦除数据的总大小*/
    result = sfud_erase(sfud_dev, 0, 4096);           // 擦除从 0 开始的 4096 字节
		rt_kprintf("sfud_erase = %d\r\n",result);
	
    write_data = rt_malloc(4096);              // 内存申请,函数会从系统堆空间中找到合适大小的内存块，然后把内存块可用地址返回给用户。
    rt_memset(write_data, 0xaa, 4096);            // 作用是在一段内存块中填充某个给定的值，将  write_data 32 个地址填入1
 
    /*往 Flash 写数据：flash：Flash 设备对象；addr：起始地址；size：从起始地址开始写入数据的总大小；data：待写入的数据*/
    result = sfud_write(sfud_dev, 0, 4096, write_data); // 将数据 32 字节的 write_data 从 0 开始写入 flash
 rt_kprintf("sfud_write = %d\r\n",result);
	
    read_data = rt_malloc(4096);
    rt_memset(read_data, 0, 4096);
 
    /*读取 Flash 数据； flash： Flash 设备对象 ；addr： 起始地址；size：从起始地址开始读取数据的总大小；data：读取到的数据*/
   result = sfud_read(sfud_dev, 0, 4096, read_data);   // 读取从 0 开始的 32 字节，存入 read_data
  rt_kprintf("sfud_read = %d\r\n",result);
		
    for (uint16_t var = 0; var < 100; ++var)
    {
        rt_kprintf("var = %d ,data = %d \n",var,read_data[var]) ;
    }
		    result = sfud_erase(sfud_dev, 0, 4096);           // 擦除从 0 开始的 4096 字节
		rt_kprintf("sfud_erase = %d\r\n",result);
}

