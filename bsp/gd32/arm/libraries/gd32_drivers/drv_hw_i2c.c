/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <rthw.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C hardware configuration */
struct gd32_i2c_config
{
    rt_uint32_t i2c_periph;     /* I2C peripheral base address */
    rcu_periph_enum periph_clk; /* I2C peripheral clock */

    rcu_periph_enum scl_clk;    /* SCL pin clock */
    rt_uint32_t scl_port;       /* SCL pin port */
    rt_uint32_t scl_pin;        /* SCL pin */
    rt_uint32_t scl_af;         /* SCL pin alternate function */

    rcu_periph_enum sda_clk;    /* SDA pin clock */
    rt_uint32_t sda_port;       /* SDA pin port */
    rt_uint32_t sda_pin;        /* SDA pin */
    rt_uint32_t sda_af;         /* SDA pin alternate function */

    IRQn_Type ev_irq_type;      /* Event IRQn */
    IRQn_Type er_irq_type;      /* Error IRQn */

    rt_uint32_t i2c_clock_hz;        /* I2C clock speed in Hz, e.g., 100000 for 100kHz */

    const char *device_name;    /* Device name */
};

/* I2C runtime context */
struct gd32_i2c
{
    struct rt_i2c_bus_device parent;
    const struct gd32_i2c_config *config;

    /* Interrupt-driven transfer context */
    struct rt_i2c_msg *msg;
    volatile rt_size_t count;
    volatile rt_err_t result;
    struct rt_completion completion;

    struct rt_completion bus_mutex;
	struct rt_completion sync_sem;
	uint32_t dev_config;
	uint16_t addr1;
	uint16_t addr2;
	uint32_t xfer_len;
	struct rt_i2c_msg *current;
	uint8_t errs;
	rt_bool_t is_restart;
};

int rt_hw_i2c_init(void);

#ifdef __cplusplus
}
#endif


#ifdef RT_USING_I2C

#define LOG_TAG              "drv.i2c"
#include <rtdbg.h>

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

static inline void i2c_enable_interrupts(uint32_t i2c_periph)
{
    i2c_interrupt_enable(i2c_periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c_periph, I2C_INT_EV);
    i2c_interrupt_enable(i2c_periph, I2C_INT_BUF);
}

static inline void i2c_disable_interrupts(uint32_t i2c_periph)
{
    i2c_interrupt_disable(i2c_periph, I2C_INT_ERR);
    i2c_interrupt_disable(i2c_periph, I2C_INT_EV);
    i2c_interrupt_disable(i2c_periph, I2C_INT_BUF);
}


static inline void i2c_xfer_read(struct gd32_i2c *i2c_obj)
{
    i2c_obj->current->len--;
	*i2c_obj->current->buf = I2C_DATA(i2c_obj->config->i2c_periph);
	i2c_obj->current->buf++;

	if ((i2c_obj->xfer_len > 0U) &&
	    (i2c_obj->current->len == 0U)) {
		i2c_obj->current++;
	}
}

static inline void i2c_xfer_write(struct gd32_i2c *i2c_obj)
{
   
}

static void gd32_i2c_event_handler(struct gd32_i2c *i2c_obj)
{
    
}

// Error interrupt handler
static void gd32_i2c_error_handler(struct gd32_i2c *i2c_obj)
{
    
}

static rt_err_t i2c_xfer_begin(struct gd32_i2c *i2c_obj)
{
    
}

static rt_ssize_t gd32_i2c_master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
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
void I2C0_EV_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_i2c_event_handler(&i2c_objs[I2C0_ENABLED_INDEX]);
    rt_interrupt_leave();
}
void I2C0_ER_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_i2c_error_handler(&i2c_objs[I2C0_ENABLED_INDEX]);
    rt_interrupt_leave();
}
#endif
#ifdef BSP_USING_HW_I2C1
void I2C1_EV_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_i2c_event_handler(&i2c_objs[I2C1_ENABLED_INDEX]);
    rt_interrupt_leave();
}
void I2C1_ER_IRQHandler(void)
{
    rt_interrupt_enter();
    gd32_i2c_error_handler(&i2c_objs[I2C1_ENABLED_INDEX]);
    rt_interrupt_leave();
}
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
        LOG_I("I2C bus %s registered successfully.", config->device_name);
    }
    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /* RT_USING_I2C */
