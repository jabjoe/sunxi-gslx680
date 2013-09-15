#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>// gpio base address
#define PIO_BASE_ADDRESS    (0x01c20800)
#define PIO_RANGE_SIZE      (0x400)
#define GPIO_ENABLE
#define SYSCONFIG_GPIO_ENABLE

#define PIO_INT_STAT_OFFSET          (0x214)
#define PIO_INT_CTRL_OFFSET          (0x210)
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <linux/i2c.h>


typedef enum {
     PIO_INT_CFG0_OFFSET = 0x200,
     PIO_INT_CFG1_OFFSET = 0x204,
     PIO_INT_CFG2_OFFSET = 0x208,
     PIO_INT_CFG3_OFFSET = 0x20c,
} int_cfg_offset;

typedef enum{
    POSITIVE_EDGE = 0x0,
    NEGATIVE_EDGE = 0x1,
    HIGH_LEVEL = 0x2,
    LOW_LEVEL = 0x3,
    DOUBLE_EDGE = 0x4
} ext_int_mode;

// gpio base address
#define PIO_BASE_ADDRESS    (0x01c20800)
#define PIO_RANGE_SIZE      (0x400)
#define GPIO_ENABLE
#define SYSCONFIG_GPIO_ENABLE

#define PIO_INT_STAT_OFFSET          (0x214)
#define PIO_INT_CTRL_OFFSET          (0x210)

