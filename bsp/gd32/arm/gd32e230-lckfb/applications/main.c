/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-07-28     Yunkun Huang    Init Project
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "at24cxx.h"


#define POWER_PIN  GET_PIN(B, 11)   
#define EEPROM_TEST_ADDR  10        // 要写入数据的EEPROM起始地址


int main(void)
{
    rt_pin_mode(POWER_PIN, PIN_MODE_OUTPUT);

    // Step 1: 启动时先拉低（确保断电）
    rt_pin_write(POWER_PIN, PIN_LOW);
    rt_thread_mdelay(100);  // 稳定断电

    // Step 2: 延时 1 秒
    rt_kprintf("AT24C64: Power off, waiting 1s...\n");
    rt_thread_mdelay(1000);

    // Step 3: 上电
    rt_pin_write(POWER_PIN, PIN_HIGH);
    rt_kprintf("AT24C64: Power ON, wait for stable...\n");

    // 建议再延时几 ms 等待芯片稳定
    rt_thread_mdelay(10);

    at24cxx_device_t dev;

    // 等待 I2C 总线就绪（可选：加点延时或检测设备）
    rt_thread_mdelay(10);

    dev = at24cxx_init("i2c0", 0);

    if (dev != RT_NULL)
    {
        rt_kprintf("AT24C64: Device initialized successfully.\n");

        rt_err_t t = at24cxx_check(dev);

        rt_kprintf("AT24C64: Check result: %s\n", (t == RT_EOK) ? "OK" : "Failed");
    }

    return 0;
}

