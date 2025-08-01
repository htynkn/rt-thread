/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_hw_i2c.h"

#ifdef RT_USING_I2C

#define LOG_TAG              "drv.i2c"
#include <rtdbg.h>

/* I2C configuration list */
static const struct gd32_i2c_config i2c_configs[] =
{
#ifdef BSP_USING_HW_I2C0
     {
        .i2c_periph = I2C0, .device_name = "i2c0", .periph_clk = RCU_I2C0,.i2c_clock_hz = BSP_HW_I2C0_CLOCK_SPEED,
        .scl_clk = RCU_GPIOA, .scl_port = GPIOA, .scl_pin = GPIO_PIN_9, .scl_af = GPIO_AF_4,
        .sda_clk = RCU_GPIOA, .sda_port = GPIOA, .sda_pin = GPIO_PIN_10, .sda_af = GPIO_AF_4,
        .ev_irq_type = I2C0_EV_IRQn, .er_irq_type = I2C0_ER_IRQn,
    },
#endif
#ifdef BSP_USING_HW_I2C1
    {
        .i2c_periph = I2C1, .device_name = "i2c1", .periph_clk = RCU_I2C1, .i2c_clock_hz = BSP_HW_I2C1_CLOCK_SPEED,
        .scl_clk = RCU_GPIOB, .scl_port = GPIOB, .scl_pin = GPIO_PIN_10, .scl_af = GPIO_AF_1,
        .sda_clk = RCU_GPIOB, .sda_port = GPIOB, .sda_pin = GPIO_PIN_11, .sda_af = GPIO_AF_1,
        .ev_irq_type = I2C1_EV_IRQn, .er_irq_type = I2C1_ER_IRQn,
    },
#endif
};

static struct gd32_i2c i2c_objs[sizeof(i2c_configs) / sizeof(i2c_configs[0])];

/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 * @param i2c_obj: The I2C object context.
 */
static void gd32_i2c_irq_handler(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    /* Start bit sent successfully */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_SBSEND))
    {
        uint8_t direction = (i2c_obj->msg->flags & RT_I2C_RD) ? I2C_RECEIVER : I2C_TRANSMITTER;
        i2c_master_addressing(i2c_periph, i2c_obj->msg->addr << 1, direction);
    }
    /* Address sent successfully */
    else if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_ADDSEND))
    {
        /* For a zero-length transfer, we can stop here. This is common for device presence checks. */
        if (i2c_obj->msg->len == 0)
        {
            if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
            {
                i2c_stop_on_bus(i2c_periph);
            }
            i2c_obj->result = RT_EOK;
            rt_completion_done(&i2c_obj->completion);
        }
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_ADDSEND);
    }
    /* Master transmitter logic */
    else if (i2c_flag_get(i2c_periph, I2C_FLAG_MASTER) && i2c_flag_get(i2c_periph, I2C_FLAG_TR))
    {
        /* Check if transmit buffer is empty */
        if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_TBE))
        {
            if (i2c_obj->count < i2c_obj->msg->len)
            {
                i2c_data_transmit(i2c_periph, i2c_obj->msg->buf[i2c_obj->count++]);
            }
        }
        /* Check if byte transfer is finished */
        if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_BTC))
        {
             if (i2c_obj->count >= i2c_obj->msg->len)
            {
                if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
                {
                    i2c_stop_on_bus(i2c_periph);
                }
                i2c_obj->result = RT_EOK;
                rt_completion_done(&i2c_obj->completion);
                /* Clear BTC flag by reading SR1 and SR2 */
                i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_BTC);
            }
        }
    }
    /* Master receiver logic */
    else if(i2c_flag_get(i2c_periph, I2C_FLAG_MASTER) && !i2c_flag_get(i2c_periph, I2C_FLAG_TR))
    {
        /* Check if receive buffer is not empty */
        if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_RBNE))
        {
            /* Special timing logic for N > 2 bytes, based on official GD32 example */
            if (i2c_obj->msg->len > 2 && (i2c_obj->msg->len - i2c_obj->count) == 3)
            {
                /* Wait for BTC to be set for byte N-2. This ensures N-2 is fully shifted
                   out before we disable ACK for byte N-1. */
                while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC));
                /* Disable ACK to send NACK after the next byte (N-1) is received */
                i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            }

            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);

            /* Check if this was the last byte */
            if (i2c_obj->count == i2c_obj->msg->len)
            {
                if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
                {
                    i2c_stop_on_bus(i2c_periph);
                }
                i2c_obj->result = RT_EOK;
                rt_completion_done(&i2c_obj->completion);
            }
        }
    }
}

/**
 * @brief I2C error interrupt handler.
 * @param i2c_obj: The I2C object context.
 */
static void gd32_i2c_err_irq_handler(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    /* Acknowledge failure */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_AERR))
    {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_AERR);
        i2c_obj->result = -RT_EIO;
        LOG_D("I2C NACK error.");
    }
    /* Bus error */
    else if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_BERR))
    {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_BERR);
        i2c_obj->result = -RT_ERROR;
        LOG_E("I2C bus error.");
    }
    /* Other errors should be handled as needed */
    else
    {
        if(i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_LOSTARB)) i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_LOSTARB);
        if(i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_OUERR)) i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_OUERR);
        i2c_obj->result = -RT_ERROR;
        LOG_E("I2C other error.");
    }

    /* In case of error, always try to send a stop condition to release the bus */
    i2c_stop_on_bus(i2c_periph);
    /* Wake up the waiting thread */
    rt_completion_done(&i2c_obj->completion);
}


#ifdef BSP_USING_HW_I2C0
void I2C0_EV_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_irq_handler(&i2c_objs[0]); 
    rt_interrupt_leave(); 
}
void I2C0_ER_IRQHandler(void) {
    rt_interrupt_enter(); 
    gd32_i2c_err_irq_handler(&i2c_objs[0]); 
    rt_interrupt_leave(); 
}
#endif
#ifdef BSP_USING_HW_I2C1
void I2C1_EV_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_irq_handler(&i2c_objs[0]); 
    rt_interrupt_leave(); 
}
void I2C1_ER_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_err_irq_handler(&i2c_objs[0]); 
    rt_interrupt_leave(); }
#endif


static rt_ssize_t gd32_i2c_master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct gd32_i2c *i2c_obj = rt_container_of(bus, struct gd32_i2c, parent);
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;
    rt_err_t ret;
    rt_uint32_t i;

    /* Wait until I2C bus is idle */
    rt_tick_t start_tick = rt_tick_get();
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        if (rt_tick_get() - start_tick > bus->timeout)
        {
            LOG_E("I2C bus busy timeout!");
            return -RT_EBUSY;
        }
    }

    for (i = 0; i < num; i++)
    {
        i2c_obj->msg = &msgs[i];
        i2c_obj->count = 0;
        i2c_obj->result = -RT_ERROR; /* Default to error */

        rt_completion_init(&i2c_obj->completion);

        /* Pre-configuration for read transfers based on official GD32 examples */
        if (i2c_obj->msg->flags & RT_I2C_RD)
        {
            /* Always enable acknowledge by default for reads > 1 byte */
            i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
            /* For N=1 byte read, ACK must be disabled before sending address */
            if (i2c_obj->msg->len == 1)
            {
                i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            }
            /* For N=2 bytes read, POS must be set to ACK the next byte */
            else if (i2c_obj->msg->len == 2)
            {
                i2c_ackpos_config(i2c_periph, I2C_ACKPOS_NEXT);
            }
        }

        /* Enable interrupts */
        i2c_interrupt_enable(i2c_periph, I2C_INT_EV);
        i2c_interrupt_enable(i2c_periph, I2C_INT_BUF);
        i2c_interrupt_enable(i2c_periph, I2C_INT_ERR);
        /* Generate start condition */
        i2c_start_on_bus(i2c_periph);

        /* Wait for transfer completion or timeout */
        ret = rt_completion_wait(&i2c_obj->completion, bus->timeout);

        /* Disable interrupts after transfer is done or timed out */
        i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
        i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
        i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);

        /* Restore ACK and ACKPOS to default state for next transfer */
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
        if (i2c_obj->msg->flags & RT_I2C_RD && i2c_obj->msg->len == 2)
        {
            i2c_ackpos_config(i2c_periph, I2C_ACKPOS_CURRENT);
        }

        if (ret != RT_EOK)
        {
            LOG_E("I2C transfer message %d timeout", i);
            /* Attempt to generate a stop condition to reset the bus state */
            i2c_stop_on_bus(i2c_periph);
            return -RT_ETIMEOUT;
        }

        if (i2c_obj->result != RT_EOK)
        {
            LOG_D("I2C transfer message %d error: %d", i, i2c_obj->result);
            return i2c_obj->result;
        }
    }

    return num;
}

static const struct rt_i2c_bus_device_ops gd32_i2c_ops =
{
    .master_xfer = gd32_i2c_master_xfer,
    .slave_xfer  = RT_NULL,
    .i2c_bus_control = RT_NULL,
};

int rt_hw_i2c_init(void)
{
    rt_size_t obj_num = sizeof(i2c_objs) / sizeof(struct gd32_i2c);
    rt_err_t result;

    for (int i = 0; i < obj_num; i++)
    {
        const struct gd32_i2c_config *config = &i2c_configs[i];
        struct gd32_i2c *i2c_obj = &i2c_objs[i];

        i2c_obj->config = config;
        i2c_obj->parent.ops = &gd32_i2c_ops;
        i2c_obj->parent.timeout = RT_TICK_PER_SECOND; // Default 1s timeout

        /* Enable clocks */
        rcu_periph_clock_enable(config->periph_clk);
        rcu_periph_clock_enable(config->scl_clk);
        if (config->scl_clk != config->sda_clk)
        {
            rcu_periph_clock_enable(config->sda_clk);
        }

        /* Configure GPIO */
        gpio_mode_set(config->scl_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->scl_pin);
        gpio_output_options_set(config->scl_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->scl_pin);
        gpio_af_set(config->scl_port, config->scl_af, config->scl_pin);

        gpio_mode_set(config->sda_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->sda_pin);
        gpio_output_options_set(config->sda_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->sda_pin);
        gpio_af_set(config->sda_port, config->sda_af, config->sda_pin);

        /* Configure I2C peripheral */
        i2c_deinit(config->i2c_periph);
        i2c_clock_config(config->i2c_periph, config->i2c_clock_hz, I2C_DTCY_2);
        i2c_mode_addr_config(config->i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
        i2c_enable(config->i2c_periph);
        i2c_ack_config(config->i2c_periph, I2C_ACK_ENABLE);

        /* Configure NVIC for interrupts */
        nvic_irq_enable(config->ev_irq_type, 2);
        nvic_irq_enable(config->er_irq_type, 2);

        /* Register I2C bus device */
        result = rt_i2c_bus_device_register(&i2c_obj->parent, config->device_name);
        if (result != RT_EOK)
        {
            LOG_E("Failed to register i2c bus %s", config->device_name);
            return result;
        }
         LOG_D("I2C bus %s registered", config->device_name);
    }
    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /* RT_USING_I2C */