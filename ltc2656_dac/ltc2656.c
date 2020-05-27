/*
 * ltc2656.c
 *
 * Driver for Linear Tech. Digital Analog Converters.
 * Supports ltc2656 in dasiy chain With external Logic to genrate CS line
 * to support single write txn from spi.
 *
 * Copyright (c) 2019 Circuitvalley.com.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/jiffies.h>

#define DEVICE_NAME "dac-ltc"
#define CLASS_NAME "dac"

#define IRQ_GPIO 	23 //standard gpio number
#define OUT_GPIO 	24
#define LDAC_GPIO 	5
#define CLR_GPIO	6

#define LTC2656_MAX_DAC_CHANNELS	32

#define LTC2656_ADDR(x)				((x) << 16)
#define LTC2656_CMD(x)				((x) << 20)

#define LTC2656_VAL1(x)				(x & 0xFFFF)
#define LTC2656_VAL2(x)             ((x >> 16) & 0xFFFF)
#define LTC2656_VAL3(x)             ((x >> 32) & 0xFFFF)
#define LTC2656_VAL4(x)             ((x >> 48) & 0xFFFF)


#define LTC2656_CMD_WRITE_INPUT_N 				0x0
#define LTC2656_CMD_UPDATE_DAC_N				0x1
#define LTC2656_CMD_WRITE_INPUT_N_UPDATE_ALL	0x2
#define LTC2656_CMD_WRITE_INPUT_N_UPDATE_N		0x3
#define LTC2656_CMD_POWERDOWN_N					0x4
#define LTC2656_CMD_POWERDOWN_CHIP				0x5
#define LTC2656_CMD_SELECT_INTERNAL_REFERENCE	0x6
#define LTC2656_CMD_SELECT_EXTERNAL_REFERENCE	0x7
#define LTC2656_CMD_NOP							0xF


static unsigned int irqNumber;
static int majorNumber;
static struct class *dev_class;
static struct device *dev_device;

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
loff_t dev_llseek(struct file *flip, loff_t off, int whence);

static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);
static struct workqueue_struct *work_queue;
DECLARE_WAIT_QUEUE_HEAD(write_wait_queue);

volatile static uint32_t buffer_index;
static bool last_spi_write_buffer = true;
static void work_func(struct work_struct *work);
static DECLARE_WORK(work, work_func);
static int ltc2656_sample_write( void );
char *ltc2656_sample_buffer;

struct ltc2656_state
{
	struct device *dev;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 spi[64]; //4byte per dac + 4byte per dac dummy , (32 * 8)
	} data ____cacheline_aligned;
};

static struct ltc2656_state st;

enum ltc2656_type
{
	ID_LTC2656
};

enum
{
	LTC2656_BUFFER_SIZE = 524288, //1sec on 32 16bit channel @ 8.192Khz
};

#define IS_WRITE_BUFFER_A(pos, len) ((pos <= LTC2656_BUFFER_SIZE) && (pos+len <= LTC2656_BUFFER_SIZE))	//check if expected write is within buffer A
#define IS_WRITE_BUFFER_B(pos, len) ((pos >  LTC2656_BUFFER_SIZE) && (pos+len >  LTC2656_BUFFER_SIZE))

#define IS_SPI_BUFFER_A       ((buffer_index * 64) < LTC2656_BUFFER_SIZE)
#define IS_SPI_BUFFER_B       ((buffer_index * 64) > LTC2656_BUFFER_SIZE)

#define IS_BUFFER_AVAILABLE(poj,len) !((IS_WRITE_BUFFER_A(poj, len) && IS_SPI_BUFFER_A) || (IS_WRITE_BUFFER_B(poj, len) && IS_SPI_BUFFER_B))

#define IS_OUT_OF_BOUNDS(pos,len) ((( pos < LTC2656_BUFFER_SIZE) && ((pos+len) > LTC2656_BUFFER_SIZE)) || ((pos >= (LTC2656_BUFFER_SIZE)) && ((pos+len)> (LTC2656_BUFFER_SIZE *2))))


static struct file_operations fops =
{
		.llseek = dev_llseek,
		.open = dev_open,
		.read = dev_read,
		.write = dev_write,
		.release = dev_release,
};

struct mmap_info
{
	char *data;
};



typedef struct ltc2656_slice_s
{
	uint16_t dac_slice[LTC2656_MAX_DAC_CHANNELS];
}ltc2656_slice_t;



static void work_func(struct work_struct *work)
{
	static bool ledOn = false;

	ledOn = !ledOn;                          // Invert the LED state on each button press
	gpio_set_value(OUT_GPIO, ledOn);          // Set the physical LED accordingly
	ltc2656_sample_write();

	if (last_spi_write_buffer != IS_SPI_BUFFER_A)
	{
		wake_up_interruptible(&write_wait_queue);
	}

	last_spi_write_buffer = IS_SPI_BUFFER_A;
}


static int dev_open(struct inode *inodep, struct file *filep)
{
	int result;
	struct mmap_info *info;
	info = kmalloc(sizeof (struct mmap_info), GFP_KERNEL);

	if (!info)
	{
		return -ENOMEM;
	}

	info->data = kmalloc(LTC2656_BUFFER_SIZE * 2, GFP_KERNEL);
	ltc2656_sample_buffer = info->data;
	filep->private_data = info;

	buffer_index = 0;
	last_spi_write_buffer = true;
	result = request_irq(irqNumber, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING,  "ltc2656_gpio_handler",	NULL);      // *dev_id when shared 

	if(result !=0)
	{
		printk(KERN_INFO "LTC2656: IRQ Request Failed\n");
		return result;
	}

	return 0;
}


static int dev_release(struct inode *inode, struct file *filep)
{
	struct mmap_info *info;
	free_irq(irqNumber, NULL);
	info = filep->private_data;

	kfree(info->data);
	kfree(info);
	filep->private_data = NULL;
	return 0;
}


static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct mmap_info *info;

	info = filep->private_data;

	if(IS_OUT_OF_BOUNDS(filep->f_pos, len))
	{
		printk(KERN_INFO "LTC2656: Write Out of bounds\n");
		return -ENOSPC;
	}

	wait_event_interruptible(write_wait_queue, IS_BUFFER_AVAILABLE(filep->f_pos, len));

	if (copy_from_user(info->data + (filep->f_pos), buffer , min(len, (size_t)LTC2656_BUFFER_SIZE)))
	{
		return -EFAULT;
	}
	else
	{
		*offset += len;
		filep->f_pos += len;
		return len;
	}


}


static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct mmap_info *info;
	int ret;

	info = filep->private_data;
	ret = min(len, (size_t)LTC2656_BUFFER_SIZE);
	if (copy_to_user(buffer, info->data + (filep->f_pos), ret)) {
		ret = -EFAULT;
	}
	return ret;
}


loff_t dev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch(whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;

	case 1: /* SEEK_CUR */

		newpos = filp->f_pos + off;
		break;
	case 2: /* SEEK_END */ //there is no point of doing seek end in this driver as mem size is fix
		newpos = (LTC2656_BUFFER_SIZE * 2) - 1;
		break;
	default: /* can't happen */
		return -EINVAL;
	}

	if (newpos<0 || IS_OUT_OF_BOUNDS(newpos, 0) ) return -EINVAL;

	filp->f_pos = newpos;
	return newpos;
}


static int ltc2656_spi_write(uint32_t cmd, uint64_t *val)
{
	uint32_t i;
	struct spi_device *spi = to_spi_device(st.dev);
	spi->controller->rt = true;
	spi->max_speed_hz = 40000000;

	for(i=0; i<8; i = i+1)
	{
		st.data.spi[(i*8) + 0] = cpu_to_be32(LTC2656_CMD(cmd) |  LTC2656_ADDR(i & 0x7) | (uint16_t)LTC2656_VAL1( *(val+ i)) );
		st.data.spi[(i*8) + 1] = cpu_to_be32(LTC2656_CMD(cmd) |  LTC2656_ADDR(i & 0x7) | (uint16_t)LTC2656_VAL2( *(val+ i)) );
		st.data.spi[(i*8) + 2] = cpu_to_be32(LTC2656_CMD(cmd) |  LTC2656_ADDR(i & 0x7) | (uint16_t)LTC2656_VAL3( *(val+ i)) );
		st.data.spi[(i*8) + 3] = cpu_to_be32(LTC2656_CMD(cmd) |  LTC2656_ADDR(i & 0x7) | (uint16_t)LTC2656_VAL4( *(val+ i)) );
	}

	return spi_write(spi, &st.data.spi, sizeof(st.data.spi));
}


static int ltc2656_sample_write( void )
{

	uint64_t *sample = (uint64_t *)(((ltc2656_slice_t *)ltc2656_sample_buffer) + buffer_index);
	ltc2656_spi_write(LTC2656_CMD_WRITE_INPUT_N_UPDATE_N, sample);

	return 0;
}


static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	buffer_index = (buffer_index + 1) & 0x3FFF; //roll after 16383
	queue_work(work_queue, &work);
	return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}


static int ltc2656_probe(struct device *dev)
{
	st.dev = dev;

	ltc2656_spi_write(LTC2656_CMD_SELECT_INTERNAL_REFERENCE, ((uint64_t *)( &st.data.spi))); //init all DAC to internal referece data does not matter
	return 0;
}


static int ltc2656_remove(struct device *dev)
{
	return 0;
}


static int ltc2656_spi_probe(struct spi_device *spi)
{
	return ltc2656_probe(&spi->dev);
}


static int ltc2656_spi_remove(struct spi_device *spi)
{
	return ltc2656_remove(&spi->dev);
}


static const struct spi_device_id ltc2656_spi_ids[] = {
		{"ltc2656", ID_LTC2656},
		{}
};

MODULE_DEVICE_TABLE(spi, ltc2656_spi_ids);

static const struct of_device_id ltc_2656_dt_id[] =
{
		{ .compatible = "ltc,ltc2656",},
		{}
};

MODULE_DEVICE_TABLE(of, ltc_2656_dt_id);

static struct spi_driver ltc2656_spi_driver = {
		.driver = {
				.name = "ltc2656",
		},
		.probe = ltc2656_spi_probe,
		.remove = ltc2656_spi_remove,
		.id_table = ltc2656_spi_ids
};

static int __init ltc2656_spi_register_driver(void)
{

	return spi_register_driver(&ltc2656_spi_driver);
}

static void ltc2656_spi_unregister_driver(void)
{
	spi_unregister_driver(&ltc2656_spi_driver);
}


static int __init ltc2656_init(void)
{
	int result;
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber < 0)
	{
		return majorNumber;
	}

	dev_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(dev_class))
	{
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to register device class");
		return PTR_ERR(dev_class);
	}


	dev_device = device_create(dev_class, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(dev_device))
	{
		class_destroy(dev_class);
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to cerate device\n");
	}

	work_queue = create_singlethread_workqueue("ltc2656_workqueue");

	result = ltc2656_spi_register_driver();

	if (result) {
		ltc2656_spi_unregister_driver();
		printk(KERN_ALERT "SPI register failed\n");
		return result;
	}
	result = gpio_request(IRQ_GPIO, "IRQ");
	result |= gpio_direction_input(IRQ_GPIO);
	result |= gpio_export(IRQ_GPIO, false);

	result |= gpio_request(OUT_GPIO, "DEBUG_OUT");
	result |= gpio_direction_output(OUT_GPIO, false);
	gpio_set_value(OUT_GPIO, false);
	result |= gpio_export(OUT_GPIO, false);

	result |= gpio_request(CLR_GPIO, "DAC_CLR");
	result |= gpio_direction_output(CLR_GPIO, false);
	gpio_set_value(CLR_GPIO, false);
	result |= gpio_export(CLR_GPIO, false);


	if (result == 0)
	{
		irqNumber = gpio_to_irq(IRQ_GPIO);
		printk(KERN_INFO "LTC2656: Mapped to IRQ: %d\n", irqNumber);

	}
	else
	{
		printk(KERN_INFO "LTC2656: GPIO Register Failed\n");
	}


	printk(KERN_INFO "LTC2656: DAC Created\n");
	return 0;
}
module_init(ltc2656_init);

static void __exit ltc2656_exit(void)
{
	free_irq(irqNumber, NULL);
	gpio_unexport(IRQ_GPIO);
	gpio_free(IRQ_GPIO);
	gpio_set_value(CLR_GPIO, 0);
	gpio_unexport(CLR_GPIO);
	gpio_free(CLR_GPIO);
	gpio_set_value(OUT_GPIO, 0);
	gpio_unexport(OUT_GPIO);
	gpio_free(OUT_GPIO);


	cancel_work_sync(&work);
	destroy_workqueue(work_queue);
	ltc2656_spi_unregister_driver();
	device_destroy(dev_class, MKDEV(majorNumber, 0));
	class_unregister(dev_class);
	class_destroy(dev_class);
	unregister_chrdev(majorNumber, DEVICE_NAME);

	printk(KERN_INFO "LTC2656: DAC Removed\n");
}
module_exit(ltc2656_exit);

MODULE_AUTHOR("Gaurav Singh <gauravsingh@circuitvalley.com>");
MODULE_DESCRIPTION("Linear Tech LTC2656 DAC Daisy chain x4");
MODULE_LICENSE("GPL v2");

