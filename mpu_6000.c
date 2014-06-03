#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>

#include "mpu_6000.h"

#define CLASS_NAME "mpu6000"

#define swap_uint16_t(x)((x & 0xff) << 8) | ((x & 0xff00) >> 8)

typedef struct MPUReport {
	struct timeval timestamp;
	unsigned long error_count;

	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;

	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;
	int16_t temperature_raw;
}MPUReport;


MODULE_LICENSE("Dual BSD/GPL");

static DEFINE_MUTEX(read_lock);

#define FIFO_SIZE 32

static DECLARE_KFIFO(mpu_reports, MPUReport, FIFO_SIZE);

static struct task_struct *mpu_sample_thread;

static struct class* mpu_class = NULL;
static struct device* mpu_device = NULL;

static int write_reg(struct spi_device *spi, uint8_t reg, uint8_t val);
static ssize_t read_reg(struct spi_device *spi, uint8_t reg);
static void set_sample_rate(struct spi_device *spi, uint16_t desired_sample_rate_hz);
static void set_dlpf_filter(struct spi_device *spi, uint16_t frequency_hz);
static ssize_t read_multiple_regs(struct spi_device *spi, uint8_t reg, uint8_t * rx_buf, size_t length);
static int sample_mpu(void * data);

struct {
	unsigned int sample_rate;
	uint8_t product;
} MPU6000_data = { .sample_rate = 1000, .product = 0 };

static struct of_device_id imu_of_match[] = {
		{ .compatible = "ti,imu", },
		{ }
};

MODULE_DEVICE_TABLE(of, imu_of_match);

static int imu_remove(struct spi_device *spi) {
	printk("Module removed\n");
	return 0;
}

static ssize_t sys_store_sample_rate(struct device* dev,
								   struct device_attribute* attr,
								   const char* buf,
								   size_t count)  {
	unsigned int sample_rate;
	int status;

	status = kstrtouint(buf, 10, &sample_rate);

	if(status < 0) {
		return status;
	}

	set_sample_rate((struct spi_device *) dev->platform_data, sample_rate);

	printk(KERN_INFO "MPU 6000 Sample rate set to %d\n", sample_rate);

	return count;
}

static DEVICE_ATTR(sample_rate, S_IWUSR, NULL, sys_store_sample_rate);

static int imu_probe(struct spi_device *spi) {

	int status = 0;

	// Save spi pointer info

	spi->dev.platform_data = spi;

	//reset
	status = write_reg(spi, MPUREG_PWR_MGMT_1, BIT_H_RESET);
	mdelay(200);

	//sleep mode off
	status = write_reg(spi, MPUREG_PWR_MGMT_1, 0x00); //clear SLEEP bit
	mdelay(200);

	//disable I2C
	status = write_reg(spi, MPUREG_USER_CTRL, BIT_I2C_IF_DIS); //set bit I2C_IF_DIS to 1
	mdelay(200);

	// SAMPLE RATE
	set_sample_rate(spi, MPU6000_data.sample_rate);
	mdelay(200);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)

	set_dlpf_filter(spi, MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);
	mdelay(200);

	// Gyro scale 2000 deg/s ()
	write_reg(spi, MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	mdelay(200);

	MPU6000_data.product = read_reg(spi, MPUREG_PRODUCT_ID);

	// product-specific scaling
	switch (MPU6000_data.product) {
	case MPU6000ES_REV_C4:
	case MPU6000ES_REV_C5:
	case MPU6000_REV_C4:
	case MPU6000_REV_C5:
		// Accel scale 8g (4096 LSB/g)
		// Rev C has different scaling than rev D
		write_reg(spi, MPUREG_ACCEL_CONFIG, 1 << 3);
		break;

	case MPU6000ES_REV_D6:
	case MPU6000ES_REV_D7:
	case MPU6000ES_REV_D8:
	case MPU6000_REV_D6:
	case MPU6000_REV_D7:
	case MPU6000_REV_D8:
	case MPU6000_REV_D9:
	case MPU6000_REV_D10:
		// default case to cope with new chip revisions, which
		// presumably won't have the accel scaling bug
	default:
		// Accel scale 8g (4096 LSB/g)
		write_reg(spi, MPUREG_ACCEL_CONFIG, 2 << 3);
		break;
	}

	mdelay(200);

	// INT CFG => Interrupt on Data Ready
	write_reg(spi, MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);    // INT: Raw data ready
	mdelay(100);
	write_reg(spi, MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	mdelay(300);

	mpu_sample_thread = kthread_create(sample_mpu, spi, "mpu_sampling");

	if(mpu_sample_thread) {
		printk(KERN_INFO "New thread created");
		wake_up_process(mpu_sample_thread);
	}

	status = device_create_file(&spi->dev, &dev_attr_sample_rate);

	return 0;
}


int write_reg(struct spi_device *spi, uint8_t reg, uint8_t val) {

	uint8_t tx[2];

	tx[0] = reg;
	tx[1] = val;

	return spi_write(spi, tx, sizeof(tx));
}

/*
 set the DLPF filter frequency. This affects both accel and gyro.
 */
void set_dlpf_filter(struct spi_device *spi, uint16_t frequency_hz) {
	uint8_t filter;

	/*
	 choose next highest filter frequency available
	 */
	if (frequency_hz <= 5) {
		filter = BITS_DLPF_CFG_5HZ;
	} else if (frequency_hz <= 10) {
		filter = BITS_DLPF_CFG_10HZ;
	} else if (frequency_hz <= 20) {
		filter = BITS_DLPF_CFG_20HZ;
	} else if (frequency_hz <= 42) {
		filter = BITS_DLPF_CFG_42HZ;
	} else if (frequency_hz <= 98) {
		filter = BITS_DLPF_CFG_98HZ;
	} else if (frequency_hz <= 188) {
		filter = BITS_DLPF_CFG_188HZ;
	} else if (frequency_hz <= 256) {
		filter = BITS_DLPF_CFG_256HZ_NOLPF2;
	} else {
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
	}

	write_reg(spi, MPUREG_CONFIG, filter);
}

ssize_t read_reg(struct spi_device *spi, uint8_t reg) {
	return spi_w8r8(spi, reg | 0x80);
}

static ssize_t read_multiple_regs(struct spi_device *spi, uint8_t reg, uint8_t * rx_buf, size_t length) {

	reg = reg | 0x80;

	return spi_write_then_read(spi, &reg, sizeof(uint8_t), rx_buf, length);
}

static ssize_t mpu_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
	int ret;
	unsigned int copied;

	if (mutex_lock_interruptible(&read_lock))
		return -ERESTARTSYS;

	ret = kfifo_to_user(&mpu_reports, buf, count, &copied);

	mutex_unlock(&read_lock);

	return ret ? ret : copied;
}

static ssize_t mpu_open(struct inode * inode, struct file * file_ptr) {
	printk("Opened mpu 6000\n");
	return 0;
}

static ssize_t mpu_close(struct inode * inode, struct file * file_ptr) {
	printk("Closed mpu 6000\n");
	return 0;
}

static struct file_operations fops = {
		.read = mpu_read,
		.open = mpu_open,
		.release = mpu_close
};

static struct spi_driver imu_driver = {
		.driver = {
				.name = "imu",
				.owner = THIS_MODULE,
				.of_match_table = imu_of_match,
		},
		.probe = imu_probe,
		.remove = imu_remove,
};

void set_sample_rate(struct spi_device *spi, uint16_t desired_sample_rate_hz) {
	uint8_t div = 1000 / desired_sample_rate_hz;
	if (div > 200)
		div = 200;
	if (div < 1)
		div = 1;
	write_reg(spi, MPUREG_SMPLRT_DIV, div - 1);
	MPU6000_data.sample_rate = 1000 / div;
}

static int mpu_init(void) {

	int mpu_major;
	int retval = 0;

	INIT_KFIFO(mpu_reports);

	retval = spi_register_driver(&imu_driver);

	if (retval < 0) {
		printk(KERN_ERR "failed to register driver: error %d\n", retval);
		return retval;
	}

	mpu_major = register_chrdev(0, "imu", &fops);
	if (mpu_major < 0) {
		printk(KERN_ERR "failed to register device: error %d\n", mpu_major);
		retval = mpu_major;
		goto init_failed;
	}

	mpu_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mpu_class)) {
		printk(KERN_ERR "failed to register device class '%s'\n", CLASS_NAME);
		retval = PTR_ERR(mpu_class);
		goto init_failed;
	}

	mpu_device = device_create(mpu_class, NULL, MKDEV(mpu_major, 0), NULL, CLASS_NAME);
	if (IS_ERR(mpu_device)) {
		printk(KERN_ERR "failed to create device '%s'\n", CLASS_NAME);
		retval = PTR_ERR(mpu_device);
		goto init_failed;
	}

	return retval;

	init_failed:

	//spi_unregister_device(spi);

	return retval;
}

static int sample_mpu(void * data) {

	struct spi_device *spi = (struct spi_device *) data;

	MPUReport		mpu_report;

#pragma pack(push, 1)
	struct mpu_report {
		uint8_t		status;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		temp;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
	} mpu_raw_report = {0};
#pragma pack(pop)

	while(1) {

		set_current_state(TASK_INTERRUPTIBLE);

		// change this to be controlled by mpu interrupt
		schedule_timeout (HZ/100);

		// change this to be a dma transfer
		read_multiple_regs(spi, MPUREG_INT_STATUS, ((uint8_t *)&mpu_raw_report), sizeof(mpu_raw_report));

		do_gettimeofday(&mpu_report.timestamp);

		mpu_report.error_count = 0; // not reported

		mpu_report.accel_x_raw = swap_uint16_t(mpu_raw_report.accel_x);
		mpu_report.accel_y_raw = swap_uint16_t(mpu_raw_report.accel_y);
		mpu_report.accel_z_raw = swap_uint16_t(mpu_raw_report.accel_z);

		mpu_report.temperature_raw = swap_uint16_t(mpu_raw_report.temp);

		mpu_report.gyro_x_raw = swap_uint16_t(mpu_raw_report.gyro_x);
		mpu_report.gyro_y_raw = swap_uint16_t(mpu_raw_report.gyro_y);
		mpu_report.gyro_z_raw = swap_uint16_t(mpu_raw_report.gyro_z);

		kfifo_put(&mpu_reports, &mpu_report);
	}

	return 0;
}

module_init(mpu_init);

static void exit_mpu(void) {
	spi_unregister_driver(&imu_driver);
}

module_exit(exit_mpu);

MODULE_DESCRIPTION("Driver for MPU 6000");
MODULE_AUTHOR("Jimmy Johnson");
MODULE_LICENSE("GPL");
MODULE_ALIAS("imu");
