#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>

#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/of.h>
#include <linux/delay.h>

#include "mpu_6000.h"
#define M_PI_F			3.14159265358979323846f

MODULE_LICENSE("Dual BSD/GPL");

static int write_reg(struct spi_device *spi, uint8_t reg, uint8_t val);
static ssize_t read_reg(struct spi_device *spi, uint8_t reg);
static void set_sample_rate(struct spi_device *spi, uint16_t desired_sample_rate_hz);
static void set_dlpf_filter(struct spi_device *spi, uint16_t frequency_hz);

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */

struct gyro_scale {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
} gyro_scale = {0, 1.0f, 0, 1.0f, 0, 1.0f};


/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */

struct accel_scale {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
} accel_scale = {0, 1.0f, 0, 1.0f, 0, 1.0f};

struct {
	unsigned int sample_rate;
	uint8_t		 product;
	float			gyro_range_scale;
	float			gyro_range_rad_s;

	struct accel_scale	_accel_scale;
	float			accel_range_scale;
	float			accel_range_m_s2;

} MPU6000_data = {.sample_rate = 1000, .product = 0};


static struct of_device_id imu_of_match[] = {
  { .compatible = "ti,imu", },
  {}
};

MODULE_DEVICE_TABLE(of, imu_of_match);

static int imu_remove(struct spi_device *spi) {
	printk("<1> Goodbye world probe!\n");
	return 0;
}

static int imu_probe(struct spi_device *spi) {
	int status = 0;

	//reset
	status = write_reg(spi,MPUREG_PWR_MGMT_1,BIT_H_RESET);
	mdelay(100);

	//sleep mode off
	status = write_reg(spi, MPUREG_PWR_MGMT_1, 0x00); //clear SLEEP bit
	mdelay(100);

	//disable I2C
	status = write_reg(spi, MPUREG_USER_CTRL, BIT_I2C_IF_DIS); //set bit I2C_IF_DIS to 1
	mdelay(100);

	// SAMPLE RATE
	set_sample_rate(spi, MPU6000_data.sample_rate);
	mdelay(100);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	set_dlpf_filter(spi, MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);
	mdelay(100);

	// Gyro scale 2000 deg/s ()
	write_reg(spi,MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	mdelay(100);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	MPU6000_data.gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	MPU6000_data.gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

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

	// Correct accel scale factors of 4096 LSB/g
	// scale to m/s^2 ( 1g = 9.81 m/s^2)
	MPU6000_data.accel_range_scale = (MPU6000_ONE_G / 4096.0f);
	MPU6000_data.accel_range_m_s2 = 8.0f * MPU6000_ONE_G;

	mdelay(100);

	// INT CFG => Interrupt on Data Ready
	write_reg(spi, MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	mdelay(100);
	write_reg(spi, MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	mdelay(100);

	// Oscillator set
	// write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	//usleep(1000);
}

int write_reg(struct spi_device *spi, uint8_t reg, uint8_t val) {

	uint8_t tx[2];

	tx[0] = reg;
	tx[1] = val;

	return spi_write(spi,tx,sizeof(tx));
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
set_dlpf_filter(struct spi_device *spi, uint16_t frequency_hz)
{
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
    return spi_w8r8(spi,reg|0x80);
}


static struct spi_driver imu_driver = {
	.driver = {
		.name		= "imu",
		.owner		= THIS_MODULE,
		.of_match_table = imu_of_match,
	},
	.probe		= imu_probe,
	.remove		= imu_remove,
};

void set_sample_rate(struct spi_device *spi, uint16_t desired_sample_rate_hz)
{
  uint8_t div = 1000 / desired_sample_rate_hz;
  if(div>200) div=200;
  if(div<1) div=1;
  write_reg(spi, MPUREG_SMPLRT_DIV, div-1);
  MPU6000_data.sample_rate = 1000 / div;
}

module_spi_driver(imu_driver);

MODULE_DESCRIPTION("Driver for IMU");
MODULE_AUTHOR("Jimmy Johnson");
MODULE_LICENSE("GPL");
MODULE_ALIAS("imu");

//module_init(hello_init);
//module_exit(hello_exit);
