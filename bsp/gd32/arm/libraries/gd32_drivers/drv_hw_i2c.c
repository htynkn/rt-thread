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


// 在 gd32_i2c_irq_handler 函数定义之前，加入以下辅助函数

/**
 * @brief 处理起始位已发送 (SBSEND) 事件
 */
static inline void gd32_i2c_handle_sbsend(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;
    uint8_t dir = (i2c_obj->msg->flags & RT_I2C_RD) ? I2C_RECEIVER : I2C_TRANSMITTER;
    i2c_master_addressing(i2c_periph, i2c_obj->msg->addr << 1, dir);
}

/**
 * @brief 处理地址已发送 (ADDSEND) 事件
 */
static inline void gd32_i2c_handle_addsend(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    /* 清除 ADDSEND 标志位 (通过读取 STAT0 和 STAT1) */
    (void)I2C_STAT0(i2c_periph);
    (void)I2C_STAT1(i2c_periph);

    if (i2c_obj->msg->flags & RT_I2C_RD) /* Master Receiver */
    {
        if (i2c_obj->msg->len == 1)
        {
            // 对于 N=1 的读取，此时必须禁用 ACK
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        }
        // 对于 N=2 的读取，POS 位已在 xfer 中设置，ACK 在 BTC 中处理
    }
}

/**
 * @brief 处理发送缓冲区为空 (TBE) 事件
 */
static inline void gd32_i2c_handle_tbe(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    if (i2c_obj->count < i2c_obj->msg->len)
    {
        i2c_data_transmit(i2c_periph, i2c_obj->msg->buf[i2c_obj->count++]);
    }
    else
    {
        // 所有数据已发送，等待 BTC 标志来确认传输完成
        // 此处不再发送数据，TBE 中断会在 BTC 置位后自动停止
    }
}

/**
 * @brief 处理接收缓冲区非空 (RBNE) 事件
 */
static inline void gd32_i2c_handle_rbne(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    // 只要 RBNE 置位，就读取数据
    if (i2c_obj->count < i2c_obj->msg->len)
    {
        i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
    }
    else
    {
        // 意外接收到数据，丢弃
        (void)i2c_data_receive(i2c_periph);
    }
}

/**
 * @brief 处理字节传输完成 (BTC) 事件
 */
static inline void gd32_i2c_handle_btc(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;

    if (i2c_obj->msg->flags & RT_I2C_RD) /* Master Receiver */
    {
        /*
         * BTC 在接收模式下的处理逻辑非常关键
         * 它用于处理 N>2 时的最后3个字节 和 N=2 的情况
         */
        if (i2c_obj->msg->len > 2 && i2c_obj->count == i2c_obj->msg->len - 3)
        {
            // 场景：还剩最后3个字节时，我们收到了 BTC
            // 1. 关闭 BUF 中断，切换到 BTC 驱动的接收模式
            i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            // 2. 读取当前数据
            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
        }
        else if (i2c_obj->count == i2c_obj->msg->len - 2)
        {
             // 场景：还剩最后2个字节时，收到了 BTC
            // 1. 关闭 ACK
            i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
            // 2. 读取倒数第二个字节
            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
            // 3. 发送 STOP
            if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
            {
                i2c_stop_on_bus(i2c_periph);
            }
            // 4. 读取最后一个字节 (它已经在移位寄存器里了)
            i2c_obj->msg->buf[i2c_obj->count++] = i2c_data_receive(i2c_periph);
            
            // 5. 传输完成
            i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
            i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
            i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            i2c_obj->result = RT_EOK;
            rt_completion_done(&i2c_obj->completion);
        }
    }
    else /* Master Transmitter */
    {
        // 发送模式下，BTC 表示最后一个字节已发送完成
        if (i2c_obj->count >= i2c_obj->msg->len)
        {
            if (!(i2c_obj->msg->flags & RT_I2C_NO_STOP))
            {
                i2c_stop_on_bus(i2c_periph);
            }
            i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
            i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
            i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            i2c_obj->result = RT_EOK;
            rt_completion_done(&i2c_obj->completion);
        }
    }
}

/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 *        This version is refactored to be cleaner and more robust, inspired by Zephyr OS.
 * @param i2c_obj: The I2C object context.
 */
/**
 * @brief I2C event interrupt handler. This is the core state machine for I2C transfers.
 *        This version is refactored to be cleaner and more robust.
 * @param i2c_obj: The I2C object context.
 */
static void gd32_i2c_irq_handler(struct gd32_i2c *i2c_obj)
{
    uint32_t i2c_periph = i2c_obj->config->i2c_periph;
    uint32_t stat0, stat1;

    stat0 = I2C_STAT0(i2c_periph);
    stat1 = I2C_STAT1(i2c_periph); // 读取STAT1用于清除某些标志位，如ADDSEND

    LOG_D("IRQ handler: STAT0=0x%08x, STAT1=0x%08x", stat0, stat1);

    // --- Master Mode Event Handling ---
    if (stat1 & I2C_STAT1_MASTER)
    {
        if (stat0 & I2C_STAT0_SBSEND)
        {
            gd32_i2c_handle_sbsend(i2c_obj);
            return; // SBSEND 是独立的起始事件
        }

        if (stat0 & I2C_STAT0_ADDSEND)
        {
            gd32_i2c_handle_addsend(i2c_obj);
            // 对于发送器，地址发送后TBE会置位，需要继续处理
            // 对于接收器，地址发送后需要等待RBNE，所以可以不立即返回
        }

        if (i2c_obj->msg->flags & RT_I2C_RD) /* Master Receiver */
        {
            /* 接收器逻辑: BTC优先，因为它用于处理N>1接收的结束阶段 */
            if (stat0 & I2C_STAT0_BTC)
            {
                gd32_i2c_handle_btc(i2c_obj);
            }
            else if (stat0 & I2C_STAT0_RBNE)
            {
                gd32_i2c_handle_rbne(i2c_obj);
            }
        }
        else /* Master Transmitter */
        {
            /*
             * 发送器逻辑:
             * 当TBE置位时，我们就可以发送数据。
             * 当BTC也置位时，表示前一个字节已经完整发送出去。
             * 我们只在所有数据都发送完毕后，才关心BTC，用它来触发STOP。
             */
            if (i2c_obj->count < i2c_obj->msg->len)
            {
                // 如果还有数据要发送，并且发送缓冲区为空
                if (stat0 & I2C_STAT0_TBE)
                {
                    gd32_i2c_handle_tbe(i2c_obj); // 发送下一个字节
                }
            }
            else
            {
                // 所有数据已写入缓冲区，等待最后一个字节发送完成
                if (stat0 & I2C_STAT0_BTC)
                {
                    gd32_i2c_handle_btc(i2c_obj); // 调用它来发送STOP并完成传输
                }
            }
        }
    }
    // --- Slave Mode event handling would go here (if implemented) ---
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

        /* 默认开启ACK */
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
        /* 默认ACK当前字节 */
        i2c_ackpos_config(i2c_periph, I2C_ACKPOS_CURRENT);

        /* 针对特定长度读取的预处理 */
        if (i2c_obj->msg->flags & RT_I2C_RD)
        {
            if (i2c_obj->msg->len == 1)
            {
                // N=1: ADDSEND之后会关闭ACK, 此处无需操作
            }
            else if (i2c_obj->msg->len == 2)
            {
                // N=2: 设置POS位，让硬件在接收第2个字节时准备NACK
                i2c_ackpos_config(i2c_periph, I2C_ACKPOS_NEXT);
            }
        }
        
        /* 使能所有需要的 I2C 中断 */
        i2c_interrupt_enable(i2c_periph, I2C_INT_EV);
        i2c_interrupt_enable(i2c_periph, I2C_INT_ERR);
        i2c_interrupt_enable(i2c_periph, I2C_INT_BUF);

        /* 发送起始位，启动传输 */
        i2c_start_on_bus(i2c_periph);
        
        ret = rt_completion_wait(&i2c_obj->completion, bus->timeout);

        /* 后续的超时和错误处理... (保持不变) */
        if (ret != RT_EOK)
        {
            LOG_E("I2C msg %d timeout. Forcing cleanup.", i);
            i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
            i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
            i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
            // 尝试发送一个STOP位来释放总线
            if (!(msgs[i].flags & RT_I2C_NO_STOP))
            {
                i2c_stop_on_bus(i2c_periph);
            }
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
