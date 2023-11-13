#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#define DHT11_MAJOR 220
#define DHT11_NAME "DHT11_DRIVER"
#define BCM2711_PERL_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERL_BASE + 0x200000)
#define GPIO_SIZE 256
#define DHT11_GPIO_PIN 26
#define MAX_TRIES 100

typedef unsigned char u8;
typedef unsigned short u16;

typedef struct dht11_sensor_data {
    u16 temp;
    u16 hum;
} dht11_data;

char dht11_usage = 0;
static void *dht11_map;
volatile unsigned *dht11;

inline void set_input_mode(void) {
    *(dht11 + (DHT11_GPIO_PIN / 10)) &= ~(0x07 << (3 * (DHT11_GPIO_PIN % 10)));  // Clear func DHT11_GPIO_PIN
    *(dht11 + (DHT11_GPIO_PIN / 10)) |= (0x0 << (3 * (DHT11_GPIO_PIN % 10)));
}

inline void set_output_mode(void) {                                              // Set output mode DHT11_GPIO_PIN
    *(dht11 + (DHT11_GPIO_PIN / 10)) &= ~(0x07 << (3 * (DHT11_GPIO_PIN % 10)));  // Clear func DHT11_GPIO_PIN
    *(dht11 + (DHT11_GPIO_PIN / 10)) |= 0x01 << (3 * (DHT11_GPIO_PIN % 10));
}

inline void set_output_high(void) { *(dht11 + 7) |= (0x01 << DHT11_GPIO_PIN); }

inline void set_output_low(void) { *(dht11 + 10) |= (0x01 << DHT11_GPIO_PIN); }

static void DHT11_reset(void) {
    set_output_mode();
    set_output_low();
    msleep(20);
    set_output_high();
    udelay(30);
}

static u8 DHT11_read_pin(void) {
    set_input_mode();
    return ((*(dht11 + 13) & (1 << DHT11_GPIO_PIN)) == 0 ? 0 : 1);  // 해당 핀 값 읽기
}

static u8 DHT11_check(void) {
    u8 retry = 0;
    set_input_mode();
    while ((DHT11_read_pin() == 1) && retry < MAX_TRIES) {
        ++retry;
        udelay(1);
    }
    if (retry >= MAX_TRIES)
        return 1;
    else
        retry = 0;
    while ((DHT11_read_pin() == 0) && retry < MAX_TRIES) {
        ++retry;
        udelay(1);
    }
    if (retry >= MAX_TRIES) return 1;
    return 0;
}

static u8 DHT11_read_bit(void) {
    u8 retry = 0;
    while ((DHT11_read_pin() == 1) && retry < MAX_TRIES) {
        ++retry;
        udelay(1);
    }
    retry = 0;
    while ((DHT11_read_pin() == 0) && retry < MAX_TRIES) {
        ++retry;
        udelay(1);
    }
    udelay(40);
    if (DHT11_read_pin() == 1) return 1;
    return 0;
}

static u8 DHT11_read_byte(void) {
    u8 i, dat = 0;
    for (i = 0; i < 8; ++i) {  // 8 비트(1바이트) 읽기
        dat <<= 1;
        dat |= DHT11_read_bit();
    }
    return dat;
}

static u8 DHT11_read_data(u16 *temp, u16 *humi) {
    u8 buf[5];
    u8 i;
    DHT11_reset();
    if (DHT11_check() != 0) {
        return 1;
    }
    for (i = 0; i < 5; ++i) {  // 5 바이트 읽기
        buf[i] = DHT11_read_byte();
    }
    if (buf[4] == (buf[0] + buf[1] + buf[2] + buf[3])) {
        *humi = buf[0] << 8 | buf[1];
        *temp = buf[2] << 8 | buf[3];
        printk("buf=%d,%d,%d,%d,%d\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    }
    return 0;
}

static int DHT11_open(struct inode *inode, struct file *file) {
    printk("------%s------\n", __FUNCTION__);
    if (dht11_usage != 0) {
        return -EBUSY;
    }
    dht11_usage = 1;  // DHT11 usage check

    dht11_map = ioremap(GPIO_BASE, GPIO_SIZE);  // Physical addr. mapping
    if (!dht11_map) {                           // To handle mapping error
        printk("error: mapping gpio memory");
        iounmap(dht11_map);
        return -EBUSY;
    }
    dht11 = (volatile unsigned int *) dht11_map;

    *(dht11 + (DHT11_GPIO_PIN / 10)) &= ~(0x07 << (3 * (DHT11_GPIO_PIN % 10)));  // Clear func DHT11_GPIO_PIN
    set_output_mode();
    return 0;
}

static ssize_t DHT11_read(struct file *file, char *buf, size_t length, loff_t *off_what) {
    int result;
    dht11_data pre_dht11_data;
    printk("------%s------\n", __FUNCTION__);
    if (DHT11_read_data(&pre_dht11_data.temp, &pre_dht11_data.hum) == 0) {
        result = copy_to_user(buf, &pre_dht11_data, sizeof(pre_dht11_data));
        if (result < 0) {
            printk("error: copy_to_user()");
            return result;
        }
    }
    return length;
}

static int DHT11_release(struct inode *inode, struct file *file) {
    printk("------%s------\n", __FUNCTION__);
    dht11_usage = 0;

    if (dht11) {
        iounmap(dht11);
    }
    return 0;
}

static struct file_operations dht11_fops = {
    .owner = THIS_MODULE,
    .read = DHT11_read,
    .open = DHT11_open,
    .release = DHT11_release,
};

static int DHT11_init(void) {
    int result = register_chrdev(DHT11_MAJOR, DHT11_NAME, &dht11_fops);
    if (result < 0) {
        printk(KERN_WARNING "Can't get any major!\n");
        return result;
    }
    printk("DHT11 module uploaded!\n");
    return 0;
}

static void DHT11_exit(void) {
    unregister_chrdev(DHT11_MAJOR, DHT11_NAME);
    printk("DHT11 module removed.\n");
}

module_init(DHT11_init);
module_exit(DHT11_exit);
MODULE_LICENSE("GPL");
