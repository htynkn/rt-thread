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
        .scl_clk = RCU_GPIOB, .scl_port = GPIOB, .scl_pin = GPIO_PIN_6, .scl_af = GPIO_AF_1,
        .sda_clk = RCU_GPIOB, .sda_port = GPIOB, .sda_pin = GPIO_PIN_7, .sda_af = GPIO_AF_1,
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
/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 * @param i2c_obj: The I2C object context.
 */
/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 *        This implementation is heavily inspired by the robust state machine found
 *        in the Zephyr OS GD32 I2C driver.
 * @param i2c_obj: The I2C object context.
 */
/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 *        This version is corrected to use macros compatible with the provided gd32e23x_i2c.h.
 * @param i2c_obj: The I2C object context.
 */
/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 *        This version corrects the BTC/RBNE logic for short (N<=2) vs long (N>2) reads.
 * @param i2c_obj: The I2C object context.
 */
static void gd32_i2c_irq_handler(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;
    uint32_t stat0 = I2C_STAT0(i2c_periph);
    uint32_t stat1 = I2C_STAT1(i2c_periph);

    LOG_D("IRQ handler: STAT0=0x%08x, STAT1=0x%08x", stat0, stat1);

    if (stat0 & I2C_STAT0_SBSEND)
    {
        uint8_t dir = (i2c_obj->msg->flags & RT_I2C_RD) ? I2C_RECEIVER : I2C_TRANSMITTER;
        i2c_master_addressing(i2c_periph, i2c_obj->msg->addr << 1, dir);
    }
    else if (stat0 & I2C_STAT0_ADDSEND)
    {
        (void)stat1; // Clear ADDSEND
        if ((i2c_obj->msg->flags & RT_I2C_RD) && (i2c_obj->msg->len == 1))
        {
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        }
    }
    else if ((stat1 & I2C_STAT1_MASTER) && (stat1 & I2C_STAT1_TR)) /* Master Transmitter */
    {
        if (stat0 & I2C_STAT0_TBE)
        {
            if (i2c_obj->count < i2c_obj->msg->len)
            {
                i2c_data_transmit(i2c_periph, i2c_obj->msg->buf[i2c_obj->count++]);
            }
            else
            {
                i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            }
        }
        if (stat0 & I2C_STAT0_BTC)
        {
            if (i2c_obj->count >= i2c_obj->msg->len)
            {
                if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
                {
                    i2c_stop_on_bus(i2c_periph);
                }
                i2c_interrupt_disable(i2c_periph, I2C_INT_EV | I2C_INT_ERR | I2C_INT_BUF); // IMMEDIATE DISABLE
                i2c_obj->result = RT_EOK;
                rt_completion_done(&i2c_obj->completion);
            }
        }
    }
    else if ((stat1 & I2C_STAT1_MASTER) && !(stat1 & I2C_STAT1_TR)) /* Master Receiver */
    {
        if ((stat0 & I2C_STAT0_BTC) && (i2c_obj->msg->len > 2))
        {
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
            if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
            {
                i2c_stop_on_bus(i2c_periph);
            }
            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
            i2c_interrupt_disable(i2c_periph, I2C_INT_EV | I2C_INT_ERR | I2C_INT_BUF); // IMMEDIATE DISABLE
            i2c_obj->result = RT_EOK;
            rt_completion_done(&i2c_obj->completion);
        }
        else if (stat0 & I2C_STAT0_RBNE)
        {
            if ((i2c_obj->msg->len > 2) && (i2c_obj->count == i2c_obj->msg->len - 3))
            {
                i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
                i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
            }
            else
            {
                if (i2c_obj->count < i2c_obj->msg->len)
                {
                    i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
                }
                if (i2c_obj->count == i2c_obj->msg->len)
                {
                    if (i2c_obj->msg->len <= 2)
                    {
                        if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
                        {
                            i2c_stop_on_bus(i2c_periph);
                        }
                        i2c_interrupt_disable(i2c_periph, I2C_INT_EV | I2C_INT_ERR | I2C_INT_BUF); // IMMEDIATE DISABLE
                        i2c_obj->result = RT_EOK;
                        rt_completion_done(&i2c_obj->completion);
                    }
                }
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
    LOG_D("gd32_i2c_err_irq_handler.");

    uint32_t i2c_periph = i2c_obj->config->i2c_periph;
    i2c_obj->result = -RT_ERROR; // Default error

    /* Acknowledge failure */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_AERR)) {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_AERR);
        i2c_obj->result = -RT_EIO; // More specific error
        LOG_D("I2C NACK/Acknowledge error (AERR).");
    }
    /* Bus error */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_BERR)) {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_BERR);
        i2c_obj->result = -RT_ERROR;
        LOG_E("I2C Bus error (BERR).");
    }
    /* Arbitration lost */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_LOSTARB)) {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_LOSTARB);
        i2c_obj->result = -RT_EBUSY; // Appropriate error?
        LOG_W("I2C Arbitration lost (LOSTARB).");
    }
    /* Overrun/Underrun */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_OUERR)) {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_OUERR);
        i2c_obj->result = -RT_EIO; // Input/Output error?
        LOG_E("I2C Overrun/Underrun error (OUERR).");
    }
    /* SMBus related errors (if applicable) */
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_SMBALT)) {
        i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_SMBALT);
        LOG_W("I2C SMBus Alert (SMBALT).");
    }
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_SMBTO)) {
         i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_SMBTO);
         LOG_W("I2C SMBus Timeout (SMBTO).");
    }
    if (i2c_interrupt_flag_get(i2c_periph, I2C_INT_FLAG_PECERR)) {
         i2c_interrupt_flag_clear(i2c_periph, I2C_INT_FLAG_PECERR);
         LOG_W("I2C PEC Error (PECERR).");
    }

    /* In case of error, always try to send a stop condition to release the bus */
    i2c_stop_on_bus(i2c_periph);
    /* Disable all I2C interrupts to prevent further interrupts from this error */
    i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
    i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
    i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);

    /* Wake up the waiting thread */
    rt_completion_done(&i2c_obj->completion);
}

/**
 * @brief Tries to recover the I2C bus when it's stuck in a busy state.
 * @param config The I2C configuration structure with GPIO info.
 * @param i2c_periph The I2C peripheral base address.
 */
static rt_err_t i2c_bus_recovery(const struct gd32_i2c_config *config, uint32_t i2c_periph)
{
    LOG_W("Attempting I2C bus recovery on %s...", config->device_name);

    i2c_disable(i2c_periph);

    gpio_mode_set(config->scl_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, config->scl_pin);
    gpio_output_options_set(config->scl_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->scl_pin);
    gpio_mode_set(config->sda_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, config->sda_pin);
    gpio_output_options_set(config->sda_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->sda_pin);

    gpio_bit_set(config->scl_port, config->scl_pin);
    gpio_bit_set(config->sda_port, config->sda_pin);
    rt_thread_mdelay(2);

    if (gpio_input_bit_get(config->sda_port, config->sda_pin) == RESET)
    {
        LOG_D("SDA is stuck low, generating 9 clock pulses on SCL...");
        for (int i = 0; i < 9; i++)
        {
            gpio_bit_reset(config->scl_port, config->scl_pin);
            rt_thread_mdelay(2);
            gpio_bit_set(config->scl_port, config->scl_pin);
            rt_thread_mdelay(2);
            if (gpio_input_bit_get(config->sda_port, config->sda_pin) == SET)
            {
                 LOG_D("SDA was released after %d pulses.", i + 1);
                 break;
            }
        }
    }

    LOG_D("Generating manual STOP condition.");
    gpio_bit_reset(config->sda_port, config->sda_pin);
    rt_thread_mdelay(2);
    gpio_bit_set(config->sda_port, config->sda_pin);
    rt_thread_mdelay(2);

    gpio_af_set(config->scl_port, config->scl_af, config->scl_pin);
    gpio_mode_set(config->scl_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->scl_pin);
    gpio_af_set(config->sda_port, config->sda_af, config->sda_pin);
    gpio_mode_set(config->scl_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->sda_pin);

    LOG_D("Performing software reset on I2C peripheral.");
    i2c_software_reset_config(i2c_periph, I2C_CTL0_SRESET);
    rt_thread_mdelay(2);
    i2c_software_reset_config(i2c_periph, 0);

    LOG_D("Re-initializing I2C peripheral post-recovery.");
    i2c_clock_config(i2c_periph, config->i2c_clock_hz, I2C_DTCY_2);
    i2c_mode_addr_config(i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
    i2c_enable(i2c_periph);

    if (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        LOG_E("I2C bus recovery failed. The bus is still busy.");
        return -RT_ERROR;
    }

    LOG_I("I2C bus recovery successful.");
    return RT_EOK;
}

static rt_ssize_t gd32_i2c_master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct gd32_i2c *i2c_obj = rt_container_of(bus, struct gd32_i2c, parent);
    const struct gd32_i2c_config *config = i2c_obj->config;
    uint32_t i2c_periph = config->i2c_periph;
    rt_err_t ret;
    rt_uint32_t i;

    rt_tick_t start_tick = rt_tick_get();
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
    {
        if (rt_tick_get() - start_tick > bus->timeout)
        {
            if (i2c_bus_recovery(config, i2c_periph) != RT_EOK)
            {
                return -RT_EBUSY;
            }
        }
        rt_thread_yield();
    }

    for (i = 0; i < num; i++)
    {
        i2c_obj->msg = &msgs[i];
        i2c_obj->count = 0;
        i2c_obj->result = -RT_ERROR;
        rt_completion_init(&i2c_obj->completion);

        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
        i2c_ackpos_config(i2c_periph, I2C_ACKPOS_CURRENT);

        if ((i2c_obj->msg->flags & RT_I2C_RD) && (i2c_obj->msg->len == 2))
        {
            i2c_ackpos_config(i2c_periph, I2C_ACKPOS_NEXT);
        }

        i2c_interrupt_enable(i2c_periph, I2C_INT_EV);
        i2c_interrupt_enable(i2c_periph, I2C_INT_ERR);
        i2c_interrupt_enable(i2c_periph, I2C_INT_BUF);

        i2c_start_on_bus(i2c_periph);
        ret = rt_completion_wait(&i2c_obj->completion, bus->timeout);

        if (ret != RT_EOK)
        {
            LOG_E("I2C msg %d timeout. Forcing cleanup.", i);
            i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
            i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
            i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            i2c_stop_on_bus(i2c_periph);
            return -RT_ETIMEOUT;
        }

        if (i2c_obj->result != RT_EOK)
        {
            LOG_D("I2C msg %d transfer error: %d", i, i2c_obj->result);
            return i2c_obj->result;
        }
    }
    return num;
}

#if defined(BSP_USING_HW_I2C0)
    #define I2C0_ENABLED_INDEX 0
    #if defined(BSP_USING_HW_I2C1)
        #define I2C1_ENABLED_INDEX 1
    #else
        #define I2C1_ENABLED_INDEX -1
    #endif
#else 
    #define I2C0_ENABLED_INDEX -1
    #if defined(BSP_USING_HW_I2C1)
        #define I2C1_ENABLED_INDEX 0
    #else
        #define I2C1_ENABLED_INDEX -1
    #endif
#endif

#ifdef BSP_USING_HW_I2C0
void I2C0_EV_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_irq_handler(&i2c_objs[I2C0_ENABLED_INDEX]); 
    rt_interrupt_leave(); 
}
void I2C0_ER_IRQHandler(void) {
    rt_interrupt_enter(); 
    gd32_i2c_err_irq_handler(&i2c_objs[I2C0_ENABLED_INDEX]); 
    rt_interrupt_leave(); 
}
#endif
#ifdef BSP_USING_HW_I2C1
void I2C1_EV_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_irq_handler(&i2c_objs[I2C1_ENABLED_INDEX]); 
    rt_interrupt_leave(); 
}
void I2C1_ER_IRQHandler(void) { 
    rt_interrupt_enter(); 
    gd32_i2c_err_irq_handler(&i2c_objs[I2C1_ENABLED_INDEX]); 
    rt_interrupt_leave(); }
#endif

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
        rcu_periph_clock_enable(config->scl_clk);
        if (config->scl_clk != config->sda_clk)
        {
            rcu_periph_clock_enable(config->sda_clk);
        }
        rcu_periph_clock_enable(config->periph_clk);

        /* Configure GPIO */
        gpio_af_set(config->scl_port, config->scl_af, config->scl_pin);
        gpio_mode_set(config->scl_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->scl_pin);
        gpio_output_options_set(config->scl_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->scl_pin);
        
        gpio_af_set(config->sda_port, config->sda_af, config->sda_pin);
        gpio_mode_set(config->sda_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, config->sda_pin);
        gpio_output_options_set(config->sda_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, config->sda_pin);

        /* Configure I2C peripheral */
        i2c_deinit(config->i2c_periph);
        i2c_clock_config(config->i2c_periph, config->i2c_clock_hz, I2C_DTCY_2);
        i2c_mode_addr_config(config->i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0);
        i2c_enable(config->i2c_periph);
        i2c_ack_config(config->i2c_periph, I2C_ACK_ENABLE);

        /* Configure NVIC for interrupts */
        nvic_irq_enable(config->ev_irq_type, 2);
        nvic_irq_enable(config->er_irq_type, 1);

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
