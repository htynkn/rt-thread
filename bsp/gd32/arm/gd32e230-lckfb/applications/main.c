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

#include "ssd1306.h"
void ssd1306_TestCircle()
{
    uint32_t delta;

    for(delta = 0; delta < 5; delta ++) 
    {
        ssd1306_DrawCircle(20* delta+30, 30, 10, White);
    }
    ssd1306_UpdateScreen();
    return;
}


int main(void)
{
    ssd1306_Init();

    ssd1306_TestCircle();

    return 0;
}

