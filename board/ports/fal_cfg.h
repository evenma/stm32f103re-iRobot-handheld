/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-8      zylx         first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

#define RT_APP_PART_ADDR 0x08020000

#ifndef FAL_USING_NOR_FLASH_DEV_NAME
#define NOR_FLASH_DEV_NAME             "norflash0"
#else
#define NOR_FLASH_DEV_NAME              FAL_USING_NOR_FLASH_DEV_NAME
#endif

/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev stm32_onchip_flash;
extern struct fal_flash_dev nor_flash0;

/* flash device table */
//&stm32_onchip_flash,                                            
#define FAL_FLASH_DEV_TABLE                                          \
{                                                                    \
    &stm32_onchip_flash,                                           \
    &nor_flash0, 																										 \
}
    
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table bootloader 实际大小0x43c8 18k */
#define FAL_PART_TABLE                                                               \
{                                                                                    \
    {FAL_PART_MAGIC_WORD,        "bl",     "onchip_flash",         0,   128*1024, 0}, \
    {FAL_PART_MAGIC_WORD,       "app",     "onchip_flash",   128*1024,  192*1024, 0}, \
    {FAL_PART_MAGIC_WORD,       "download",     "onchip_flash",   320*1024,  192*1024, 0}, \
    {FAL_PART_MAGIC_WORD, "extFlash", NOR_FLASH_DEV_NAME,         0, 7*1024*1024, 0}, \
		{FAL_PART_MAGIC_WORD,       "factory",     NOR_FLASH_DEV_NAME,   7*1024*1024,  192*1024, 0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
