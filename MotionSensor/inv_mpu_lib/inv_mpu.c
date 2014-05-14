/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "inv_mpu.h"

#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "I2Cdev/I2Cdev.h"

/* The following functions must be defined for this platform:
 * i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data)
 * i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
 * delay_ms(uint32_t num_ms)
 * min(int a, int b)
 */
#define min(a,b) ((a)<(b)?(a):(b))
#define i2c_write   writeBytes
#define i2c_read(a,b,c,d)    (readBytes(a,b,c,d)!=-1?0:1)
#define delay_ms(a)    usleep(a*1000)
#define printf_P	printf

#if !defined MPU6050 && !defined MPU9150 && !defined MPU6500 && !defined MPU9250
#error  Which gyro are you using? Define MPUxxxx in your compiler options.
#endif

/* Time for some messy macro work. =]
 * #define MPU9150
 * is equivalent to..
 * #define MPU6050
 * #define AK8975_SECONDARY
 *
 * #define MPU9250
 * is equivalent to..
 * #define MPU6500
 * #define AK8963_SECONDARY
 */
#if defined MPU9150
#ifndef MPU6050
#define MPU6050
#endif                          /* #ifndef MPU6050 */
#if defined AK8963_SECONDARY
#error "MPU9150 and AK8963_SECONDARY cannot both be defined."
#elif !defined AK8975_SECONDARY /* #if defined AK8963_SECONDARY */
#define AK8975_SECONDARY
#endif                          /* #if defined AK8963_SECONDARY */
#elif defined MPU9250           /* #if defined MPU9150 */
#ifndef MPU6500
#define MPU6500
#endif                          /* #ifndef MPU6500 */
#if defined AK8975_SECONDARY
#error "MPU9250 and AK8975_SECONDARY cannot both be defined."
#elif !defined AK8963_SECONDARY /* #if defined AK8975_SECONDARY */
#define AK8963_SECONDARY
#endif                          /* #if defined AK8975_SECONDARY */
#endif                          /* #if defined MPU9150 */

#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY
#else
/* #warning "No compass = less profit for Invensense. Lame." */
#endif

static int set_int_enable(uint8_t enable);

/* Hardware registers needed by driver. */
struct gyro_reg_s
{
	uint8_t who_am_i;
	uint8_t rate_div;
	uint8_t lpf;
	uint8_t prod_id;
	uint8_t user_ctrl;
	uint8_t fifo_en;
	uint8_t gyro_cfg;
	uint8_t accel_cfg;
	uint8_t accel_cfg2;
	uint8_t lp_accel_odr;
	uint8_t motion_thr;
	uint8_t motion_dur;
	uint8_t fifo_count_h;
	uint8_t fifo_r_w;
	uint8_t raw_gyro;
	uint8_t raw_accel;
	uint8_t temp;
	uint8_t int_enable;
	uint8_t dmp_int_status;
	uint8_t int_status;
	uint8_t accel_intel;
	uint8_t pwr_mgmt_1;
	uint8_t pwr_mgmt_2;
	uint8_t int_pin_cfg;
	uint8_t mem_r_w;
	uint8_t accel_offs;
	uint8_t i2c_mst;
	uint8_t bank_sel;
	uint8_t mem_start_addr;
	uint8_t prgm_start_h;
#if defined AK89xx_SECONDARY
	uint8_t s0_addr;
	uint8_t s0_reg;
	uint8_t s0_ctrl;
	uint8_t s1_addr;
	uint8_t s1_reg;
	uint8_t s1_ctrl;
	uint8_t s4_ctrl;
	uint8_t s0_do;
	uint8_t s1_do;
	uint8_t i2c_delay_ctrl;
	uint8_t raw_compass;
	/* The I2C_MST_VDDIO bit is in this register. */
	uint8_t yg_offs_tc;
#endif
};

/* Information specific to a particular device. */
struct hw_s
{
	uint8_t addr;
	uint16_t max_fifo;
	uint8_t num_reg;
	uint16_t temp_sens;
	int16_t temp_offset;
	uint16_t bank_size;
#if defined AK89xx_SECONDARY
	uint16_t compass_fsr;
#endif
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s
{
	uint16_t gyro_fsr;
	uint8_t accel_fsr;
	uint16_t lpf;
	uint16_t sample_rate;
	uint8_t sensors_on;
	uint8_t fifo_sensors;
	uint8_t dmp_on;
};

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s
{
	/* Matches gyro_cfg >> 3 & 0x03 */
	uint8_t gyro_fsr;
	/* Matches accel_cfg >> 3 & 0x03 */
	uint8_t accel_fsr;
	/* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
	uint8_t sensors;
	/* Matches config register. */
	uint8_t lpf;
	uint8_t clk_src;
	/* Sample rate, NOT rate divider. */
	uint16_t sample_rate;
	/* Matches fifo_en register. */
	uint8_t fifo_enable;
	/* Matches int enable register. */
	uint8_t int_enable;
	/* 1 if devices on auxiliary I2C bus appear on the primary. */
	uint8_t bypass_mode;
	/* 1 if half-sensitivity.
	 * NOTE: This doesn't beint32_t here, but everything else in hw_s is const,
	 * and this allows us to save some precious RAM.
	 */
	uint8_t accel_half;
	/* 1 if device in low-power accel-only mode. */
	uint8_t lp_accel_mode;
	/* 1 if interrupts are only triggered on motion events. */
	uint8_t int_motion_only;
	struct motion_int_cache_s cache;
	/* 1 for active low interrupts. */
	uint8_t active_low_int;
	/* 1 for latched interrupts. */
	uint8_t latched_int;
	/* 1 if DMP is enabled. */
	uint8_t dmp_on;
	/* Ensures that DMP will only be loaded once. */
	uint8_t dmp_loaded;
	/* Sampling rate used when DMP is enabled. */
	uint16_t dmp_sample_rate;
#ifdef AK89xx_SECONDARY
	/* Compass sample rate. */
	uint16_t compass_sample_rate;
	uint8_t compass_addr;
	int16_t mag_sens_adj[3];
#endif
};

/* Information for self-test. */
struct test_s
{
	uint32_t gyro_sens;
	uint32_t accel_sens;
	uint8_t reg_rate_div;
	uint8_t reg_lpf;
	uint8_t reg_gyro_fsr;
	uint8_t reg_accel_fsr;
	uint16_t wait_ms;
	uint8_t packet_thresh;
	float min_dps;
	float max_dps;
	float max_gyro_var;
	float min_g;
	float max_g;
	float max_accel_var;
};

/* Gyro driver state variables. */
struct gyro_state_s
{
	const struct gyro_reg_s *reg;
	const struct hw_s *hw;
	const struct test_s *test;
	struct chip_cfg_s chip_cfg;
};

/* Filter configurations. */
enum lpf_e
{
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e
{
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e
{
	INV_FSR_2G = 0,
	INV_FSR_4G,
	INV_FSR_8G,
	INV_FSR_16G,
	NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e
{
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e
{
#if defined MPU6050
	INV_LPA_1_25HZ,
	INV_LPA_5HZ,
	INV_LPA_20HZ,
	INV_LPA_40HZ
#elif defined MPU6500
	INV_LPA_0_3125HZ,
	INV_LPA_0_625HZ,
	INV_LPA_1_25HZ,
	INV_LPA_2_5HZ,
	INV_LPA_5HZ,
	INV_LPA_10HZ,
	INV_LPA_20HZ,
	INV_LPA_40HZ,
	INV_LPA_80HZ,
	INV_LPA_160HZ,
	INV_LPA_320HZ,
	INV_LPA_640HZ
#endif
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

#if defined AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)
#elif defined AK8963_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#endif

#ifdef AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#endif

#if defined MPU6050
const struct gyro_reg_s reg =
{
	.who_am_i       = 0x75,
	.rate_div       = 0x19,
	.lpf            = 0x1A,
	.prod_id        = 0x0C,
	.user_ctrl      = 0x6A,
	.fifo_en        = 0x23,
	.gyro_cfg       = 0x1B,
	.accel_cfg      = 0x1C,
	.accel_cfg2     = 0x00,
	.lp_accel_odr   = 0x00,
	.motion_thr     = 0x1F,
	.motion_dur     = 0x20,
	.fifo_count_h   = 0x72,
	.fifo_r_w       = 0x74,
	.raw_gyro       = 0x43,
	.raw_accel      = 0x3B,
	.temp           = 0x41,
	.int_enable     = 0x38,
	.dmp_int_status = 0x39,
	.int_status     = 0x3A,
	.accel_intel    = 0x00,
	.pwr_mgmt_1     = 0x6B,
	.pwr_mgmt_2     = 0x6C,
	.int_pin_cfg    = 0x37,
	.mem_r_w        = 0x6F,
	.accel_offs     = 0x06,
	.i2c_mst        = 0x24,
	.bank_sel       = 0x6D,
	.mem_start_addr = 0x6E,
	.prgm_start_h   = 0x70
#ifdef AK89xx_SECONDARY
	,.s0_addr       = 0x25,
	.s0_reg         = 0x26,
	.s0_ctrl        = 0x27,
	.s1_addr        = 0x28,
	.s1_reg         = 0x29,
	.s1_ctrl        = 0x2A,
	.s4_ctrl        = 0x34,
	.s0_do          = 0x63,
	.s1_do          = 0x64,
	.i2c_delay_ctrl = 0x67,
	.raw_compass    = 0x49,
	.yg_offs_tc     = 0x01
#endif
};
const struct hw_s hw =
{
	//.addr           = 0x68,
	.addr           = 0x69,
	.max_fifo       = 1024,
	.num_reg        = 118,
	.temp_sens      = 340,
	.temp_offset    = -521,
	.bank_size      = 256
#if defined AK89xx_SECONDARY
	,.compass_fsr    = AK89xx_FSR
#endif
};

const struct test_s test =
{
	.gyro_sens      = 32768/250,
	.accel_sens     = 32768/16,
	.reg_rate_div   = 0,    /* 1kHz. */
	.reg_lpf        = 1,    /* 188Hz. */
	.reg_gyro_fsr   = 0,    /* 250dps. */
	.reg_accel_fsr  = 0x18, /* 16g. */
	.wait_ms        = 50,
	.packet_thresh  = 5,    /* 5% */
	.min_dps        = 10.f,
	.max_dps        = 105.f,
	.max_gyro_var   = 0.14f,
	.min_g          = 0.3f,
	.max_g          = 0.95f,
	.max_accel_var  = 0.14f
};

static struct gyro_state_s st =
{
	.reg = &reg,
	.hw = &hw,
	.test = &test
};
#elif defined MPU6500
const struct gyro_reg_s reg =
{
	.who_am_i       = 0x75,
	.rate_div       = 0x19,
	.lpf            = 0x1A,
	.prod_id        = 0x0C,
	.user_ctrl      = 0x6A,
	.fifo_en        = 0x23,
	.gyro_cfg       = 0x1B,
	.accel_cfg      = 0x1C,
	.accel_cfg2     = 0x1D,
	.lp_accel_odr   = 0x1E,
	.motion_thr     = 0x1F,
	.motion_dur     = 0x20,
	.fifo_count_h   = 0x72,
	.fifo_r_w       = 0x74,
	.raw_gyro       = 0x43,
	.raw_accel      = 0x3B,
	.temp           = 0x41,
	.int_enable     = 0x38,
	.dmp_int_status = 0x39,
	.int_status     = 0x3A,
	.accel_intel    = 0x69,
	.pwr_mgmt_1     = 0x6B,
	.pwr_mgmt_2     = 0x6C,
	.int_pin_cfg    = 0x37,
	.mem_r_w        = 0x6F,
	.accel_offs     = 0x77,
	.i2c_mst        = 0x24,
	.bank_sel       = 0x6D,
	.mem_start_addr = 0x6E,
	.prgm_start_h   = 0x70
#ifdef AK89xx_SECONDARY
	,.raw_compass   = 0x49,
	.s0_addr        = 0x25,
	.s0_reg         = 0x26,
	.s0_ctrl        = 0x27,
	.s1_addr        = 0x28,
	.s1_reg         = 0x29,
	.s1_ctrl        = 0x2A,
	.s4_ctrl        = 0x34,
	.s0_do          = 0x63,
	.s1_do          = 0x64,
	.i2c_delay_ctrl = 0x67
#endif
};
const struct hw_s hw =
{
	//.addr           = 0x68,
	.addr           = 0x69,
	.max_fifo       = 1024,
	.num_reg        = 128,
	.temp_sens      = 321,
	.temp_offset    = 0,
	.bank_size      = 256
#if defined AK89xx_SECONDARY
	,.compass_fsr    = AK89xx_FSR
#endif
};

const struct test_s test =
{
	.gyro_sens      = 32768/250,
	.accel_sens     = 32768/16,
	.reg_rate_div   = 0,    /* 1kHz. */
	.reg_lpf        = 1,    /* 188Hz. */
	.reg_gyro_fsr   = 0,    /* 250dps. */
	.reg_accel_fsr  = 0x18, /* 16g. */
	.wait_ms        = 50,
	.packet_thresh  = 5,    /* 5% */
	.min_dps        = 10.f,
	.max_dps        = 105.f,
	.max_gyro_var   = 0.14f,
	.min_g          = 0.3f,
	.max_g          = 0.95f,
	.max_accel_var  = 0.14f
};

static struct gyro_state_s st =
{
	.reg = &reg,
	.hw = &hw,
	.test = &test
};
#endif

#define MAX_PACKET_LENGTH (12)

#ifdef AK89xx_SECONDARY
static int setup_compass(void);
#define MAX_COMPASS_SAMPLE_RATE (100)
#endif

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
static int set_int_enable(uint8_t enable)
{
	uint8_t tmp;

	if (st.chip_cfg.dmp_on)
	{
		if (enable)
			tmp = BIT_DMP_INT_EN;
		else
			tmp = 0x00;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
			return 1;
		st.chip_cfg.int_enable = tmp;
	}
	else
	{
		if (!st.chip_cfg.sensors)
			return 1;
		if (enable && st.chip_cfg.int_enable)
			return 0;
		if (enable)
			tmp = BIT_DATA_RDY_EN;
		else
			tmp = 0x00;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
			return 1;
		st.chip_cfg.int_enable = tmp;
	}
	return 0;
}

/**
 *  @brief      Register dump for testing.
 *  @return     0 if successful.
 */
uint8_t mpu_reg_dump(void)
{
	uint8_t ii;
	uint8_t data;

	for (ii = 0; ii < st.hw->num_reg; ii++)
	{
		if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
			continue;
		if (i2c_read(st.hw->addr, ii, 1, &data))
			return 1;
#if defined MPU_DEBUG
		printf_P("%#5x: %#5x\r\r\n", ii, data);
#endif
	}
	return 0;
}

/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
uint8_t mpu_read_reg(uint8_t reg, uint8_t *data)
{
	if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
		return 1;
	if (reg >= st.hw->num_reg)
		return 1;
	return i2c_read(st.hw->addr, reg, 1, data);
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
uint8_t mpu_init(struct int_param_s *int_param)
{
	uint8_t data[6], rev;

	/* Reset device. */
	data[0] = BIT_RESET;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
		return 1;
	delay_ms(100);

	/* Wake up chip. */
	data[0] = 0x00;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
		return 1;

#if defined MPU6050
	/* Check product revision. */
	if (i2c_read(st.hw->addr, st.reg->accel_offs, 6, data))
		return 1;
	rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) |
				(data[1] & 0x01);
#if defined MPU_DEBUG
	printf_P("Software product rev. %d.\r\n", rev);
#endif
	if (rev)
	{
		/* Congrats, these parts are better. */
		if (rev == 1)
			st.chip_cfg.accel_half = 1;
		else if (rev == 2)
			st.chip_cfg.accel_half = 0;
		else
		{
#if defined MPU_DEBUG
			printf_P("Unsupported software product rev. %d.\r\n", rev);
#endif
			return 1;
		}
	}
	else
	{
		if (i2c_read(st.hw->addr, st.reg->prod_id, 1, data))
			return 1;
		rev = data[0] & 0x0F;
		if (!rev)
		{
#if defined MPU_DEBUG
			printf_P("Product ID read as 0 indicates device is either "
						"incompatible or an MPU3050.\r\n");
#endif
			return 1;
		}
		else if (rev == 4)
		{
#if defined MPU_DEBUG
			printf_P("Half sensitivity part found.\r\n");
#endif
			st.chip_cfg.accel_half = 1;
		}
		else
			st.chip_cfg.accel_half = 0;
	}
#elif defined MPU6500
#define MPU6500_MEM_REV_ADDR    (0x17)
	if (mpu_read_mem(MPU6500_MEM_REV_ADDR, 1, &rev))
		return 1;
	if (rev == 0x1)
		st.chip_cfg.accel_half = 0;
	else
	{
#if defined MPU_DEBUG
		printf_P("Unsupported software product rev. %d.\r\n", rev);
#endif
		return 1;
	}

	/* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
	 * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
	 */
	data[0] = BIT_FIFO_SIZE_1024 | 0x8;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data))
		return 1;
#endif

	/* Set to invalid values to ensure no I2C writes are skipped. */
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.bypass_mode = 0xFF;
#ifdef AK89xx_SECONDARY
	st.chip_cfg.compass_sample_rate = 0xFFFF;
#endif
	/* mpu_set_sensors always preserves this setting. */
	st.chip_cfg.clk_src = INV_CLK_PLL;
	/* Handled in next call to mpu_set_bypass. */
	st.chip_cfg.active_low_int = 1;
	st.chip_cfg.latched_int = 0;
	st.chip_cfg.int_motion_only = 0;
	st.chip_cfg.lp_accel_mode = 0;
	memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
	st.chip_cfg.dmp_on = 0;
	st.chip_cfg.dmp_loaded = 0;
	st.chip_cfg.dmp_sample_rate = 0;

	if (mpu_set_gyro_fsr(2000))
		return 1;
	if (mpu_set_accel_fsr(2))
		return 1;
	if (mpu_set_lpf(42))
		return 1;
	if (mpu_set_sample_rate(50))
		return 1;
	if (mpu_configure_fifo(0))
		return 1;

	/* if (int_param)
	    reg_int_cb(int_param); */

#ifdef AK89xx_SECONDARY
	setup_compass();
	if (mpu_set_compass_sample_rate(10))
		return 1;
#else
	/* Already disabled by setup_compass. */
	if (mpu_set_bypass(0))
		return 1;
#endif

	mpu_set_sensors(0);
#if defined MPU_DEBUG
	printf_P("Initializing is done...\r\n");
#endif
	return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
uint8_t mpu_lp_accel_mode(uint8_t rate)
{
	uint8_t tmp[2];

	if (rate > 40)
		return 1;

	if (!rate)
	{
		mpu_set_int_latched(0);
		tmp[0] = 0;
		tmp[1] = BIT_STBY_XYZG;
		if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
			return 1;
		st.chip_cfg.lp_accel_mode = 0;
		return 0;
	}
	/* For LP accel, we automatically configure the hardware to produce latched
	 * interrupts. In LP accel mode, the hardware cycles into sleep mode before
	 * it gets a chance to deassert the interrupt pin; therefore, we shift this
	 * responsibility over to the MCU.
	 *
	 * Any register read will clear the interrupt.
	 */
	mpu_set_int_latched(1);
#if defined MPU6050
	tmp[0] = BIT_LPA_CYCLE;
	if (rate == 1)
	{
		tmp[1] = INV_LPA_1_25HZ;
		mpu_set_lpf(5);
	}
	else if (rate <= 5)
	{
		tmp[1] = INV_LPA_5HZ;
		mpu_set_lpf(5);
	}
	else if (rate <= 20)
	{
		tmp[1] = INV_LPA_20HZ;
		mpu_set_lpf(10);
	}
	else
	{
		tmp[1] = INV_LPA_40HZ;
		mpu_set_lpf(20);
	}
	tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
		return 1;
#elif defined MPU6500
	/* Set wake frequency. */
	if (rate == 1)
		tmp[0] = INV_LPA_1_25HZ;
	else if (rate == 2)
		tmp[0] = INV_LPA_2_5HZ;
	else if (rate <= 5)
		tmp[0] = INV_LPA_5HZ;
	else if (rate <= 10)
		tmp[0] = INV_LPA_10HZ;
	else if (rate <= 20)
		tmp[0] = INV_LPA_20HZ;
	else if (rate <= 40)
		tmp[0] = INV_LPA_40HZ;
	else if (rate <= 80)
		tmp[0] = INV_LPA_80HZ;
	else if (rate <= 160)
		tmp[0] = INV_LPA_160HZ;
	else if (rate <= 320)
		tmp[0] = INV_LPA_320HZ;
	else
		tmp[0] = INV_LPA_640HZ;
	if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, tmp))
		return 1;
	tmp[0] = BIT_LPA_CYCLE;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, tmp))
		return 1;
#endif
	st.chip_cfg.sensors = INV_XYZ_ACCEL;
	st.chip_cfg.clk_src = 0;
	st.chip_cfg.lp_accel_mode = 1;
	mpu_configure_fifo(0);

	return 0;
}

/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_get_gyro_reg(int16_t *data)
{
	uint8_t tmp[6];

	if (!(st.chip_cfg.sensors & INV_XYZ_GYRO))
		return 1;

	if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp))
		return 1;
	data[0] = (tmp[0] << 8) | tmp[1];
	data[1] = (tmp[2] << 8) | tmp[3];
	data[2] = (tmp[4] << 8) | tmp[5];
	return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_get_accel_reg(int16_t *data)
{
	uint8_t tmp[6];

	if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL))
		return 1;

	if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp))
		return 1;
	data[0] = (tmp[0] << 8) | tmp[1];
	data[1] = (tmp[2] << 8) | tmp[3];
	data[2] = (tmp[4] << 8) | tmp[5];
	return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @return     0 if successful.
 */
uint8_t mpu_get_temperature(int32_t *data)
{
	uint8_t tmp[2];
	int16_t raw;

	if (!(st.chip_cfg.sensors))
		return 1;

	if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp))
		return 1;
	raw = (tmp[0] << 8) | tmp[1];

	data[0] = (int32_t)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
	return 0;
}

/**
 *  @brief      Push biases to the accel bias registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
uint8_t mpu_set_accel_bias(const int32_t *accel_bias)
{
	uint8_t data[6];
	int16_t accel_hw[3];
	int16_t got_accel[3];
	int16_t fg[3];

	if (!accel_bias)
		return 1;
	if (!accel_bias[0] && !accel_bias[1] && !accel_bias[2])
		return 0;

	if (i2c_read(st.hw->addr, 3, 3, data))
		return 1;
	fg[0] = ((data[0] >> 4) + 8) & 0xf;
	fg[1] = ((data[1] >> 4) + 8) & 0xf;
	fg[2] = ((data[2] >> 4) + 8) & 0xf;

	accel_hw[0] = (int16_t)(accel_bias[0] * 2 / (64 + fg[0]));
	accel_hw[1] = (int16_t)(accel_bias[1] * 2 / (64 + fg[1]));
	accel_hw[2] = (int16_t)(accel_bias[2] * 2 / (64 + fg[2]));

	if (i2c_read(st.hw->addr, 0x06, 6, data))
		return 1;

	got_accel[0] = ((int16_t)data[0] << 8) | data[1];
	got_accel[1] = ((int16_t)data[2] << 8) | data[3];
	got_accel[2] = ((int16_t)data[4] << 8) | data[5];

	accel_hw[0] += got_accel[0];
	accel_hw[1] += got_accel[1];
	accel_hw[2] += got_accel[2];

	data[0] = (accel_hw[0] >> 8) & 0xff;
	data[1] = (accel_hw[0]) & 0xff;
	data[2] = (accel_hw[1] >> 8) & 0xff;
	data[3] = (accel_hw[1]) & 0xff;
	data[4] = (accel_hw[2] >> 8) & 0xff;
	data[5] = (accel_hw[2]) & 0xff;

	if (i2c_write(st.hw->addr, 0x06, 6, data))
		return 1;
	return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
uint8_t mpu_reset_fifo(void)
{
	uint8_t data;

	if (!(st.chip_cfg.sensors))
		return 1;

	data = 0;
	if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
		return 1;

	if (st.chip_cfg.dmp_on)
	{
		data = BIT_FIFO_RST | BIT_DMP_RST;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return 1;
		delay_ms(50);
		data = BIT_DMP_EN | BIT_FIFO_EN;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
			data |= BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return 1;
		if (st.chip_cfg.int_enable)
			data = BIT_DMP_INT_EN;
		else
			data = 0;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
			return 1;
		data = 0;
		if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
			return 1;
	}
	else
	{
		data = BIT_FIFO_RST;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return 1;
		if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS))
			data = BIT_FIFO_EN;
		else
			data = BIT_FIFO_EN | BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
			return 1;
		delay_ms(50);
		if (st.chip_cfg.int_enable)
			data = BIT_DATA_RDY_EN;
		else
			data = 0;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
			return 1;
		if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
			return 1;
	}
	return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_get_gyro_fsr(uint16_t *fsr)
{
	switch (st.chip_cfg.gyro_fsr)
	{
	case INV_FSR_250DPS:
		fsr[0] = 250;
		break;
	case INV_FSR_500DPS:
		fsr[0] = 500;
		break;
	case INV_FSR_1000DPS:
		fsr[0] = 1000;
		break;
	case INV_FSR_2000DPS:
		fsr[0] = 2000;
		break;
	default:
		fsr[0] = 0;
		break;
	}
	return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_set_gyro_fsr(uint16_t fsr)
{
	uint8_t data;

	if (!(st.chip_cfg.sensors))
		return 1;

	switch (fsr)
	{
	case 250:
		data = INV_FSR_250DPS << 3;
		break;
	case 500:
		data = INV_FSR_500DPS << 3;
		break;
	case 1000:
		data = INV_FSR_1000DPS << 3;
		break;
	case 2000:
		data = INV_FSR_2000DPS << 3;
		break;
	default:
		return 1;
	}

	if (st.chip_cfg.gyro_fsr == (data >> 3))
		return 0;
	if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data))
		return 1;
	st.chip_cfg.gyro_fsr = data >> 3;
	return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_get_accel_fsr(uint8_t *fsr)
{
	switch (st.chip_cfg.accel_fsr)
	{
	case INV_FSR_2G:
		fsr[0] = 2;
		break;
	case INV_FSR_4G:
		fsr[0] = 4;
		break;
	case INV_FSR_8G:
		fsr[0] = 8;
		break;
	case INV_FSR_16G:
		fsr[0] = 16;
		break;
	default:
		return 1;
	}
	if (st.chip_cfg.accel_half)
		fsr[0] <<= 1;
	return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
	uint8_t data;

	if (!(st.chip_cfg.sensors))
		return 1;

	switch (fsr)
	{
	case 2:
		data = INV_FSR_2G << 3;
		break;
	case 4:
		data = INV_FSR_4G << 3;
		break;
	case 8:
		data = INV_FSR_8G << 3;
		break;
	case 16:
		data = INV_FSR_16G << 3;
		break;
	default:
		return 1;
	}

	if (st.chip_cfg.accel_fsr == (data >> 3))
		return 0;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data))
		return 1;
	st.chip_cfg.accel_fsr = data >> 3;
	return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
uint8_t mpu_get_lpf(uint16_t *lpf)
{
	switch (st.chip_cfg.lpf)
	{
	case INV_FILTER_188HZ:
		lpf[0] = 188;
		break;
	case INV_FILTER_98HZ:
		lpf[0] = 98;
		break;
	case INV_FILTER_42HZ:
		lpf[0] = 42;
		break;
	case INV_FILTER_20HZ:
		lpf[0] = 20;
		break;
	case INV_FILTER_10HZ:
		lpf[0] = 10;
		break;
	case INV_FILTER_5HZ:
		lpf[0] = 5;
		break;
	case INV_FILTER_256HZ_NOLPF2:
	case INV_FILTER_2100HZ_NOLPF:
	default:
		lpf[0] = 0;
		break;
	}
	return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
uint8_t mpu_set_lpf(uint16_t lpf)
{
	uint8_t data;

	if (!(st.chip_cfg.sensors))
		return 1;

	if (lpf >= 188)
		data = INV_FILTER_188HZ;
	else if (lpf >= 98)
		data = INV_FILTER_98HZ;
	else if (lpf >= 42)
		data = INV_FILTER_42HZ;
	else if (lpf >= 20)
		data = INV_FILTER_20HZ;
	else if (lpf >= 10)
		data = INV_FILTER_10HZ;
	else
		data = INV_FILTER_5HZ;

	if (st.chip_cfg.lpf == data)
		return 0;
	if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data))
		return 1;
	st.chip_cfg.lpf = data;
	return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_get_sample_rate(uint16_t *rate)
{
	if (st.chip_cfg.dmp_on)
		return 1;
	else
		rate[0] = st.chip_cfg.sample_rate;
	return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_set_sample_rate(uint16_t rate)
{
	uint8_t data;

	if (!(st.chip_cfg.sensors))
		return 1;

	if (st.chip_cfg.dmp_on)
		return 1;
	else
	{
		if (st.chip_cfg.lp_accel_mode)
		{
			if (rate && (rate <= 40))
			{
				/* Just stay in low-power accel mode. */
				mpu_lp_accel_mode(rate);
				return 0;
			}
			/* Requested rate exceeds the allowed frequencies in LP accel mode,
			 * switch back to full-power mode.
			 */
			mpu_lp_accel_mode(0);
		}
		if (rate < 4)
			rate = 4;
		else if (rate > 1000)
			rate = 1000;

		data = 1000 / rate - 1;
		if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))
			return 1;

		st.chip_cfg.sample_rate = 1000 / (1 + data);

#ifdef AK89xx_SECONDARY
		mpu_set_compass_sample_rate(min(st.chip_cfg.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
#endif

		/* Automatically set LPF to 1/2 sampling rate. */
		mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
		return 0;
	}
}

/**
 *  @brief      Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_get_compass_sample_rate(uint16_t *rate)
{
#ifdef AK89xx_SECONDARY
	rate[0] = st.chip_cfg.compass_sample_rate;
	return 0;
#else
	rate[0] = 0;
	return 1;
#endif
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_set_compass_sample_rate(uint16_t rate)
{
#ifdef AK89xx_SECONDARY
	uint8_t div;
	if (!rate || rate > st.chip_cfg.sample_rate || rate > MAX_COMPASS_SAMPLE_RATE)
		return 1;

	div = st.chip_cfg.sample_rate / rate - 1;
	if (i2c_write(st.hw->addr, st.reg->s4_ctrl, 1, &div))
		return 1;
	st.chip_cfg.compass_sample_rate = st.chip_cfg.sample_rate / (div + 1);
	return 0;
#else
	return 1;
#endif
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
uint8_t mpu_get_gyro_sens(float *sens)
{
	switch (st.chip_cfg.gyro_fsr)
	{
	case INV_FSR_250DPS:
		sens[0] = 131.f;
		break;
	case INV_FSR_500DPS:
		sens[0] = 65.5f;
		break;
	case INV_FSR_1000DPS:
		sens[0] = 32.8f;
		break;
	case INV_FSR_2000DPS:
		sens[0] = 16.4f;
		break;
	default:
		return 1;
	}
	return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
uint8_t mpu_get_accel_sens(uint16_t *sens)
{
	switch (st.chip_cfg.accel_fsr)
	{
	case INV_FSR_2G:
		sens[0] = 16384;
		break;
	case INV_FSR_4G:
		sens[0] = 8092;
		break;
	case INV_FSR_8G:
		sens[0] = 4096;
		break;
	case INV_FSR_16G:
		sens[0] = 2048;
		break;
	default:
		return 1;
	}
	if (st.chip_cfg.accel_half)
		sens[0] >>= 1;
	return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
uint8_t mpu_get_fifo_config(uint8_t *sensors)
{
	sensors[0] = st.chip_cfg.fifo_enable;
	return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
uint8_t mpu_configure_fifo(uint8_t sensors)
{
	uint8_t prev;
	int result = 0;

	/* Compass data isn't going into the FIFO. Stop trying. */
	sensors &= ~INV_XYZ_COMPASS;

	if (st.chip_cfg.dmp_on)
		return 0;
	else
	{
		if (!(st.chip_cfg.sensors))
			return 1;
		prev = st.chip_cfg.fifo_enable;
		st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
		if (st.chip_cfg.fifo_enable != sensors)
			/* You're not getting what you asked for. Some sensors are
			 * asleep.
			 */
			result = -1;
		else
			result = 0;
		if (sensors || st.chip_cfg.lp_accel_mode)
			set_int_enable(1);
		else
			set_int_enable(0);
		if (sensors)
		{
			if (mpu_reset_fifo())
			{
				st.chip_cfg.fifo_enable = prev;
				return 1;
			}
		}
	}

	return result;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
uint8_t mpu_get_power_state(uint8_t *power_on)
{
	if (st.chip_cfg.sensors)
		power_on[0] = 1;
	else
		power_on[0] = 0;
	return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
uint8_t mpu_set_sensors(uint8_t sensors)
{
	uint8_t data;
#ifdef AK89xx_SECONDARY
	uint8_t user_ctrl;
#endif

	if (sensors & INV_XYZ_GYRO)
		data = INV_CLK_PLL;
	else if (sensors)
		data = 0;
	else
		data = BIT_SLEEP;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data))
	{
		st.chip_cfg.sensors = 0;
		return 1;
	}
	st.chip_cfg.clk_src = data & ~BIT_SLEEP;

	data = 0;
	if (!(sensors & INV_X_GYRO))
		data |= BIT_STBY_XG;
	if (!(sensors & INV_Y_GYRO))
		data |= BIT_STBY_YG;
	if (!(sensors & INV_Z_GYRO))
		data |= BIT_STBY_ZG;
	if (!(sensors & INV_XYZ_ACCEL))
		data |= BIT_STBY_XYZA;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data))
	{
		st.chip_cfg.sensors = 0;
		return 1;
	}

	if (sensors && (sensors != INV_XYZ_ACCEL))
		/* Latched interrupts only used in LP accel mode. */
		mpu_set_int_latched(0);

#ifdef AK89xx_SECONDARY
#ifdef AK89xx_BYPASS
	if (sensors & INV_XYZ_COMPASS)
		mpu_set_bypass(1);
	else
		mpu_set_bypass(0);
#else
	if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
		return 1;
	/* Handle AKM power management. */
	if (sensors & INV_XYZ_COMPASS)
	{
		data = AKM_SINGLE_MEASUREMENT;
		user_ctrl |= BIT_AUX_IF_EN;
	}
	else
	{
		data = AKM_POWER_DOWN;
		user_ctrl &= ~BIT_AUX_IF_EN;
	}
	if (st.chip_cfg.dmp_on)
		user_ctrl |= BIT_DMP_EN;
	else
		user_ctrl &= ~BIT_DMP_EN;
	if (i2c_write(st.hw->addr, st.reg->s1_do, 1, &data))
		return 1;
	/* Enable/disable I2C master mode. */
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
		return 1;
#endif
#endif

	st.chip_cfg.sensors = sensors;
	st.chip_cfg.lp_accel_mode = 0;
	delay_ms(50);
	return 0;
}

/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
uint8_t mpu_get_int_status(int16_t *status)
{
	uint8_t tmp[2];
	if (!st.chip_cfg.sensors)
		return 1;
	if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp))
		return 1;
	status[0] = (tmp[0] << 8) | tmp[1];
	return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
uint8_t mpu_read_fifo(int16_t *gyro, int16_t *accel,
											uint8_t *sensors, uint8_t *more)
{
	/* Assumes maximum packet size is gyro (6) + accel (6). */
	uint8_t data[MAX_PACKET_LENGTH];
	uint8_t packet_size = 0;
	uint16_t fifo_count, index = 0;

	if (st.chip_cfg.dmp_on)
		return 1;

	sensors[0] = 0;
	if (!st.chip_cfg.sensors)
		return 1;
	if (!st.chip_cfg.fifo_enable)
		return 1;

	if (st.chip_cfg.fifo_enable & INV_X_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Y_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_Z_GYRO)
		packet_size += 2;
	if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
		packet_size += 6;

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
		return 1;
	fifo_count = (data[0] << 8) | data[1];
	if (fifo_count < packet_size)
		return 0;
#if defined MPU_DEBUG
    printf_P("FIFO count: %hd\r\n", fifo_count);
#endif
	if (fifo_count > (st.hw->max_fifo >> 1))
	{
		/* FIFO is 50% full, better check overflow bit. */
		if (i2c_read(st.hw->addr, st.reg->int_status, 1, data))
			return 1;
		if (data[0] & BIT_FIFO_OVERFLOW)
		{
			mpu_reset_fifo();
			return 2;
		}
	}

	if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data))
		return 1;
	more[0] = fifo_count / packet_size - 1;
	sensors[0] = 0;

	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
	{
		accel[0] = (data[index+0] << 8) | data[index+1];
		accel[1] = (data[index+2] << 8) | data[index+3];
		accel[2] = (data[index+4] << 8) | data[index+5];
		sensors[0] |= INV_XYZ_ACCEL;
		index += 6;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO)
	{
		gyro[0] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_X_GYRO;
		index += 2;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO)
	{
		gyro[1] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Y_GYRO;
		index += 2;
	}
	if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO)
	{
		gyro[2] = (data[index+0] << 8) | data[index+1];
		sensors[0] |= INV_Z_GYRO;
		index += 2;
	}

	return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
uint8_t mpu_read_fifo_stream(uint16_t length, uint8_t *data,
														 uint8_t *more)
{
	uint8_t tmp[2];
	uint16_t fifo_count;
	if (!st.chip_cfg.dmp_on)
    {
		return 1;
    }
	if (!st.chip_cfg.sensors)
    {
		return 1;
    }

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp))
		return 1;
	fifo_count = (tmp[0] << 8) | tmp[1];
	if (fifo_count < length)
	{
		more[0] = 0;
		return 1;
	}
	if (fifo_count > (st.hw->max_fifo >> 1))
	{
		/* FIFO is 50% full, better check overflow bit. */
		if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp))
			return 1;
		if (tmp[0] & BIT_FIFO_OVERFLOW)
		{
			mpu_reset_fifo();
			return 2;
		}
	}
#if defined MPU_DEBUG
    printf_P("FIFO count: %hd\r\n", fifo_count);
#endif

	if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data))
		return 1;
	more[0] = fifo_count / length - 1;
	return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
uint8_t mpu_set_bypass(uint8_t bypass_on)
{
	uint8_t tmp;

	if (st.chip_cfg.bypass_mode == bypass_on)
		return 0;

	if (bypass_on)
	{
		if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return 1;
		tmp &= ~BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return 1;
		delay_ms(3);
		tmp = BIT_BYPASS_EN;
		if (st.chip_cfg.active_low_int)
			tmp |= BIT_ACTL;
		if (st.chip_cfg.latched_int)
			tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
		if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
			return 1;
	}
	else
	{
		/* Enable I2C master mode if compass is being used. */
		if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return 1;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
			tmp |= BIT_AUX_IF_EN;
		else
			tmp &= ~BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return 1;
		delay_ms(3);
		if (st.chip_cfg.active_low_int)
			tmp = BIT_ACTL;
		else
			tmp = 0;
		if (st.chip_cfg.latched_int)
			tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
		if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
			return 1;
	}
	st.chip_cfg.bypass_mode = bypass_on;
	return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
uint8_t mpu_set_int_level(uint8_t active_low)
{
	st.chip_cfg.active_low_int = active_low;
	return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
uint8_t mpu_set_int_latched(uint8_t enable)
{
	uint8_t tmp;
	if (st.chip_cfg.latched_int == enable)
		return 0;

	if (enable)
		tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
	else
		tmp = 0;
	if (st.chip_cfg.bypass_mode)
		tmp |= BIT_BYPASS_EN;
	if (st.chip_cfg.active_low_int)
		tmp |= BIT_ACTL;
	if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
		return 1;
	st.chip_cfg.latched_int = enable;
	return 0;
}

#ifdef MPU6050
static int get_accel_prod_shift(float *st_shift)
{
	uint8_t tmp[4], shift_code[3], ii;

	if (i2c_read(st.hw->addr, 0x0D, 4, tmp))
		return 0x07;

	shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
	shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
	shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
	for (ii = 0; ii < 3; ii++)
	{
		if (!shift_code[ii])
		{
			st_shift[ii] = 0.f;
			continue;
		}
		/* Equivalent to..
		 * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
		 */
		st_shift[ii] = 0.34f;
		while (--shift_code[ii])
			st_shift[ii] *= 1.034f;
	}
	return 0;
}

static int accel_self_test(int32_t *bias_regular, int32_t *bias_st)
{
	int jj, result = 0;
	float st_shift[3], st_shift_cust, st_shift_var;

	get_accel_prod_shift(st_shift);
	for(jj = 0; jj < 3; jj++)
	{
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (st_shift[jj])
		{
			st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
			if (fabs(st_shift_var) > test.max_accel_var)
				result |= 1 << jj;
		}
		else if ((st_shift_cust < test.min_g) ||
						 (st_shift_cust > test.max_g))
			result |= 1 << jj;
	}

	return result;
}

static int gyro_self_test(int32_t *bias_regular, int32_t *bias_st)
{
	int jj, result = 0;
	uint8_t tmp[3];
	float st_shift, st_shift_cust, st_shift_var;

	if (i2c_read(st.hw->addr, 0x0D, 3, tmp))
		return 0x07;

	tmp[0] &= 0x1F;
	tmp[1] &= 0x1F;
	tmp[2] &= 0x1F;

	for (jj = 0; jj < 3; jj++)
	{
		st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
		if (tmp[jj])
		{
			st_shift = 3275.f / test.gyro_sens;
			while (--tmp[jj])
				st_shift *= 1.046f;
			st_shift_var = st_shift_cust / st_shift - 1.f;
			if (fabs(st_shift_var) > test.max_gyro_var)
				result |= 1 << jj;
		}
		else if ((st_shift_cust < test.min_dps) ||
						 (st_shift_cust > test.max_dps))
			result |= 1 << jj;
	}
	return result;
}

#ifdef AK89xx_SECONDARY
static int compass_self_test(void)
{
	uint8_t tmp[6];
	uint8_t tries = 10;
	int result = 0x07;
	int16_t data;

	mpu_set_bypass(1);

	tmp[0] = AKM_POWER_DOWN;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
		return 0x07;
	tmp[0] = AKM_BIT_SELF_TEST;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp))
		goto AKM_restore;
	tmp[0] = AKM_MODE_SELF_TEST;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
		goto AKM_restore;

	do
	{
		delay_ms(10);
		if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ST1, 1, tmp))
			goto AKM_restore;
		if (tmp[0] & AKM_DATA_READY)
			break;
	}
	while (tries--);
	if (!(tmp[0] & AKM_DATA_READY))
		goto AKM_restore;

	if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_HXL, 6, tmp))
		goto AKM_restore;

	result = 0;
	data = (int16_t)(tmp[1] << 8) | tmp[0];
	if ((data > 100) || (data < -100))
		result |= 0x01;
	data = (int16_t)(tmp[3] << 8) | tmp[2];
	if ((data > 100) || (data < -100))
		result |= 0x02;
	data = (int16_t)(tmp[5] << 8) | tmp[4];
	if ((data > -300) || (data < -1000))
		result |= 0x04;

AKM_restore:
	tmp[0] = 0 | SUPPORTS_AK89xx_HIGH_SENS;
	i2c_write(st.chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp);
	tmp[0] = SUPPORTS_AK89xx_HIGH_SENS;
	i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp);
	mpu_set_bypass(0);
	return result;
}
#endif
#endif

static int get_st_biases(int32_t *gyro, int32_t *accel, uint8_t hw_test)
{
	uint8_t data[MAX_PACKET_LENGTH];
	uint8_t packet_count, ii;
	uint16_t fifo_count;

	data[0] = 0x01;
	data[1] = 0;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
		return 1;
	delay_ms(200);
	data[0] = 0;
	if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return 1;
	data[0] = BIT_FIFO_RST | BIT_DMP_RST;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return 1;
	delay_ms(15);
	data[0] = st.test->reg_lpf;
	if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
		return 1;
	data[0] = st.test->reg_rate_div;
	if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
		return 1;
	if (hw_test)
		data[0] = st.test->reg_gyro_fsr | 0xE0;
	else
		data[0] = st.test->reg_gyro_fsr;
	if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
		return 1;

	if (hw_test)
		data[0] = st.test->reg_accel_fsr | 0xE0;
	else
		data[0] = test.reg_accel_fsr;
	if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
		return 1;
	if (hw_test)
		delay_ms(200);

	/* Fill FIFO for test.wait_ms milliseconds. */
	data[0] = BIT_FIFO_EN;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
		return 1;

	data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return 1;
	delay_ms(test.wait_ms);
	data[0] = 0;
	if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
		return 1;

	if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
		return 1;

	fifo_count = (data[0] << 8) | data[1];
	packet_count = fifo_count / MAX_PACKET_LENGTH;
	gyro[0] = gyro[1] = gyro[2] = 0;
	accel[0] = accel[1] = accel[2] = 0;

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_cur[3], gyro_cur[3];
		if (i2c_read(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data))
			return 1;
		accel_cur[0] = ((int16_t)data[0] << 8) | data[1];
		accel_cur[1] = ((int16_t)data[2] << 8) | data[3];
		accel_cur[2] = ((int16_t)data[4] << 8) | data[5];
		accel[0] += (int32_t)accel_cur[0];
		accel[1] += (int32_t)accel_cur[1];
		accel[2] += (int32_t)accel_cur[2];
		gyro_cur[0] = (((int16_t)data[6] << 8) | data[7]);
		gyro_cur[1] = (((int16_t)data[8] << 8) | data[9]);
		gyro_cur[2] = (((int16_t)data[10] << 8) | data[11]);
		gyro[0] += (int32_t)gyro_cur[0];
		gyro[1] += (int32_t)gyro_cur[1];
		gyro[2] += (int32_t)gyro_cur[2];
	}
#ifdef EMPL_NO_64BIT
	gyro[0] = (int32_t)(((float)gyro[0]*65536.f) / test.gyro_sens / packet_count);
	gyro[1] = (int32_t)(((float)gyro[1]*65536.f) / test.gyro_sens / packet_count);
	gyro[2] = (int32_t)(((float)gyro[2]*65536.f) / test.gyro_sens / packet_count);
	if (has_accel)
	{
		accel[0] = (int32_t)(((float)accel[0]*65536.f) / test.accel_sens /
												 packet_count);
		accel[1] = (int32_t)(((float)accel[1]*65536.f) / test.accel_sens /
												 packet_count);
		accel[2] = (int32_t)(((float)accel[2]*65536.f) / test.accel_sens /
												 packet_count);
		/* Don't remove gravity! */
		accel[2] -= 65536L;
	}
#else
	gyro[0] = (int32_t)(((int64_t)gyro[0]<<16) / test.gyro_sens / packet_count);
	gyro[1] = (int32_t)(((int64_t)gyro[1]<<16) / test.gyro_sens / packet_count);
	gyro[2] = (int32_t)(((int64_t)gyro[2]<<16) / test.gyro_sens / packet_count);
	accel[0] = (int32_t)(((int64_t)accel[0]<<16) / test.accel_sens /
											 packet_count);
	accel[1] = (int32_t)(((int64_t)accel[1]<<16) / test.accel_sens /
											 packet_count);
	accel[2] = (int32_t)(((int64_t)accel[2]<<16) / test.accel_sens /
											 packet_count);
	/* Don't remove gravity! */
	if (accel[2] > 0L)
		accel[2] -= 65536L;
	else
		accel[2] += 65536L;
#endif

	return 0;
}

/**
 *  @brief      Trigger gyro/accel/compass self-test.
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  \n Currently, the hardware self-test is unsupported for MPU6500. However,
 *  this function can still be used to obtain the accel and gyro biases.
 *
 *  \n This function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @return     Result mask (see above).
 */
uint8_t mpu_run_self_test(int32_t *gyro, int32_t *accel)
{
#ifdef MPU6050
	const uint8_t tries = 2;
	int32_t gyro_st[3], accel_st[3];
	uint8_t accel_result, gyro_result;
#ifdef AK89xx_SECONDARY
	uint8_t compass_result;
#endif
	int ii;
#endif
	int result;
	uint8_t accel_fsr, fifo_sensors, sensors_on;
	uint16_t gyro_fsr, sample_rate, lpf;
	uint8_t dmp_was_on;

	if (st.chip_cfg.dmp_on)
	{
		mpu_set_dmp_state(0);
		dmp_was_on = 1;
	}
	else
		dmp_was_on = 0;

	/* Get initial settings. */
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
	mpu_get_lpf(&lpf);
	mpu_get_sample_rate(&sample_rate);
	sensors_on = st.chip_cfg.sensors;
	mpu_get_fifo_config(&fifo_sensors);

	/* For older chips, the self-test will be different. */
#if defined MPU6050
	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro, accel, 0))
			break;
	if (ii == tries)
	{
		/* If we reach this point, we most likely encountered an I2C error.
		 * We'll just report an error for all three sensors.
		 */
		result = 0;
		goto restore;
	}
	for (ii = 0; ii < tries; ii++)
		if (!get_st_biases(gyro_st, accel_st, 1))
			break;
	if (ii == tries)
	{
		/* Again, probably an I2C error. */
		result = 0;
		goto restore;
	}
	accel_result = accel_self_test(accel, accel_st);
	gyro_result = gyro_self_test(gyro, gyro_st);

	result = 0;
	if (!gyro_result)
		result |= 0x01;
	if (!accel_result)
		result |= 0x02;

#ifdef AK89xx_SECONDARY
	compass_result = compass_self_test();
	if (!compass_result)
		result |= 0x04;
#endif
restore:
#elif defined MPU6500
	/* For now, this function will return a "pass" result for all three sensors
	 * for compatibility with current test applications.
	 */
	get_st_biases(gyro, accel, 0);
	result = 0x7;
#endif
	/* Set to invalid values to ensure no I2C writes are skipped. */
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	mpu_set_lpf(lpf);
	mpu_set_sample_rate(sample_rate);
	mpu_set_sensors(sensors_on);
	mpu_configure_fifo(fifo_sensors);

	if (dmp_was_on)
		mpu_set_dmp_state(1);

	return result;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
uint8_t mpu_write_mem(uint16_t mem_addr, uint16_t length,
											uint8_t *data)
{
	uint8_t tmp[2];

	if (!data)
		return 1;
	if (!st.chip_cfg.sensors)
		return 1;

	tmp[0] = (uint8_t)(mem_addr >> 8);
	tmp[1] = (uint8_t)(mem_addr & 0xFF);

	/* Check bank boundaries. */
	if (tmp[1] + length > st.hw->bank_size)
		return 1;

	if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
		return 1;
	if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data))
		return 1;
	return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
uint8_t mpu_read_mem(uint16_t mem_addr, uint16_t length,
										 uint8_t *data)
{
	uint8_t tmp[2];

	if (!data)
		return 1;
	if (!st.chip_cfg.sensors)
		return 1;

	tmp[0] = (uint8_t)(mem_addr >> 8);
	tmp[1] = (uint8_t)(mem_addr & 0xFF);

	/* Check bank boundaries. */
	if (tmp[1] + length > st.hw->bank_size)
		return 1;

	if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
		return 1;
	if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data))
		return 1;
	return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
uint8_t mpu_load_firmware(uint16_t length, const uint8_t *firmware,
													uint16_t start_addr, uint16_t sample_rate)
{
	uint16_t ii;
	uint16_t this_write;
	uint8_t *pgm_buf;
	uint8_t jj;
	/* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
	uint8_t cur[LOAD_CHUNK], tmp[2];
	pgm_buf = (uint8_t *)malloc(LOAD_CHUNK);

	if (st.chip_cfg.dmp_loaded)
		/* DMP should only be loaded once. */
		return 1;

	if (!firmware)
		return 1;
	for (ii = 0; ii < length; ii += this_write)
	{
		this_write = min(LOAD_CHUNK, length - ii);		
		for (jj = 0; jj < LOAD_CHUNK; jj++) pgm_buf[jj] = firmware[ii+jj];//pgm_read_byte(firmware + ii + jj);
		if (mpu_write_mem(ii, this_write, pgm_buf))
			return 1;
		if (mpu_read_mem(ii, this_write, cur))
			return 1;
		if (memcmp(pgm_buf, cur, this_write))
			return 2;
	}

	/* Set program start address. */
	tmp[0] = start_addr >> 8;
	tmp[1] = start_addr & 0xFF;
	if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
		return 1;

	st.chip_cfg.dmp_loaded = 1;
	st.chip_cfg.dmp_sample_rate = sample_rate;
	return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
uint8_t mpu_set_dmp_state(uint8_t enable)
{
	uint8_t tmp;
	if (st.chip_cfg.dmp_on == enable)
		return 0;

	if (enable)
	{
		if (!st.chip_cfg.dmp_loaded)
			return 1;
		/* Disable data ready interrupt. */
		set_int_enable(0);
		/* Disable bypass mode. */
		mpu_set_bypass(0);
		/* Keep constant sample rate, FIFO rate controlled by DMP. */
		mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
		/* Remove FIFO elements. */
		tmp = 0;
		i2c_write(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 1;
		/* Enable DMP interrupt. */
		set_int_enable(1);
		mpu_reset_fifo();
	}
	else
	{
		/* Disable DMP interrupt. */
		set_int_enable(0);
		/* Restore FIFO settings. */
		tmp = st.chip_cfg.fifo_enable;
		i2c_write(st.hw->addr, 0x23, 1, &tmp);
		st.chip_cfg.dmp_on = 0;
		mpu_reset_fifo();
	}
	return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
uint8_t mpu_get_dmp_state(uint8_t *enabled)
{
	enabled[0] = st.chip_cfg.dmp_on;
	return 0;
}


/* This initialization is similar to the one in ak8975.c. */
static int setup_compass(void)
{
#ifdef AK89xx_SECONDARY
	uint8_t data[4], akm_addr;
#if defined MPU_DEBUG
	printf_P("Initializing compass...\r\n");
#endif
	mpu_set_bypass(1);

	/* Find compass. Possible addresses range from 0x0C to 0x0F. */
	for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++)
	{
		int result;
		result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data);
		if (!result && (data[0] == AKM_WHOAMI))
			break;
	}

	if (akm_addr > 0x0F)
	{
		/* TODO: Handle this case in all compass-related functions. */
#if defined MPU_DEBUG
		printf_P("Compass not found.\r\n");
#endif
		return 1;
	}

	st.chip_cfg.compass_addr = akm_addr;

	data[0] = AKM_POWER_DOWN;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return 1;
	delay_ms(1);

	data[0] = AKM_FUSE_ROM_ACCESS;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return 1;
	delay_ms(1);

	/* Get sensitivity adjustment data from fuse ROM. */
	if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ASAX, 3, data))
		return 1;
	st.chip_cfg.mag_sens_adj[0] = (int32_t)data[0] + 128;
	st.chip_cfg.mag_sens_adj[1] = (int32_t)data[1] + 128;
	st.chip_cfg.mag_sens_adj[2] = (int32_t)data[2] + 128;

	data[0] = AKM_POWER_DOWN;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return 1;
	delay_ms(1);

	mpu_set_bypass(0);

	/* Set up master mode, master clock, and ES bit. */
	data[0] = 0x40;
	if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
		return 1;

	/* Slave 0 reads from AKM data registers. */
	data[0] = BIT_I2C_READ | st.chip_cfg.compass_addr;
	if (i2c_write(st.hw->addr, st.reg->s0_addr, 1, data))
		return 1;

	/* Compass reads start at this register. */
	data[0] = AKM_REG_ST1;
	if (i2c_write(st.hw->addr, st.reg->s0_reg, 1, data))
		return 1;

	/* Enable slave 0, 8-byte reads. */
	data[0] = BIT_SLAVE_EN | 8;
	if (i2c_write(st.hw->addr, st.reg->s0_ctrl, 1, data))
		return 1;

	/* Slave 1 changes AKM measurement mode. */
	data[0] = st.chip_cfg.compass_addr;
	if (i2c_write(st.hw->addr, st.reg->s1_addr, 1, data))
		return 1;

	/* AKM measurement mode register. */
	data[0] = AKM_REG_CNTL;
	if (i2c_write(st.hw->addr, st.reg->s1_reg, 1, data))
		return 1;

	/* Enable slave 1, 1-byte writes. */
	data[0] = BIT_SLAVE_EN | 1;
	if (i2c_write(st.hw->addr, st.reg->s1_ctrl, 1, data))
		return 1;

	/* Set slave 1 data. */
	data[0] = AKM_SINGLE_MEASUREMENT;
	if (i2c_write(st.hw->addr, st.reg->s1_do, 1, data))
		return 1;

	/* Trigger slave 0 and slave 1 actions at each sample. */
	data[0] = 0x03;
	if (i2c_write(st.hw->addr, st.reg->i2c_delay_ctrl, 1, data))
		return 1;

#ifdef MPU9150
	/* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
#if defined MPU_DEBUG
	printf_P("Initializing compass at IMU...\r\n");
#endif
	data[0] = BIT_I2C_MST_VDDIO;
	if (i2c_write(st.hw->addr, st.reg->yg_offs_tc, 1, data))
		return 1;
#endif

	return 0;
#else
	return 1;
#endif
}

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_get_compass_reg(int16_t *data)
{
#ifdef AK89xx_SECONDARY
	uint8_t tmp[9];

	if (!(st.chip_cfg.sensors & INV_XYZ_COMPASS))
		return 1;

#ifdef AK89xx_BYPASS
	if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ST1, 8, tmp))
		return 1;
	tmp[8] = AKM_SINGLE_MEASUREMENT;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp+8))
		return 1;
#else
	if (i2c_read(st.hw->addr, st.reg->raw_compass, 8, tmp))
		return 1;
#endif

#if defined AK8975_SECONDARY
	/* AK8975 doesn't have the overrun error bit. */
	if (!(tmp[0] & AKM_DATA_READY))
		return 2;
	if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
		return 3;
#elif defined AK8963_SECONDARY
	/* AK8963 doesn't have the data read error bit. */
	if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
		return 2;
	if (tmp[7] & AKM_OVERFLOW)
		return 3;
#endif
	data[0] = (tmp[2] << 8) | tmp[1];
	data[1] = (tmp[4] << 8) | tmp[3];
	data[2] = (tmp[6] << 8) | tmp[5];

	data[0] = ((int32_t)data[0] * st.chip_cfg.mag_sens_adj[0]) >> 8;
	data[1] = ((int32_t)data[1] * st.chip_cfg.mag_sens_adj[1]) >> 8;
	data[2] = ((int32_t)data[2] * st.chip_cfg.mag_sens_adj[2]) >> 8;

	return 0;
#else
	return 1;
#endif
}

/**
 *  @brief      Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_get_compass_fsr(uint16_t *fsr)
{
#ifdef AK89xx_SECONDARY
	fsr[0] = st.hw->compass_fsr;
	return 0;
#else
	return 1;
#endif
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *  The behavior of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n MPU6050:
 *  \n When this mode is first enabled, the hardware captures a single accel
 *  sample, and subsequent samples are compared with this one to determine if
 *  the device is in motion. Therefore, whenever this "locked" sample needs to
 *  be changed, this function must be called again.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data and detects any large change
 *  over a int16_t period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
uint8_t mpu_lp_motion_interrupt(uint16_t thresh, uint8_t time,
																uint8_t lpa_freq)
{
	uint8_t data[3];

	if (lpa_freq)
	{
		uint8_t thresh_hw;

#if defined MPU6050
		/* TODO: Make these const/#defines. */
		/* 1LSb = 32mg. */
		if (thresh > 8160)
			thresh_hw = 255;
		else if (thresh < 32)
			thresh_hw = 1;
		else
			thresh_hw = thresh >> 5;
#elif defined MPU6500
		/* 1LSb = 4mg. */
		if (thresh > 1020)
			thresh_hw = 255;
		else if (thresh < 4)
			thresh_hw = 1;
		else
			thresh_hw = thresh >> 2;
#endif

		if (!time)
			/* Minimum duration must be 1ms. */
			time = 1;

#if defined MPU6050
		if (lpa_freq > 40)
#elif defined MPU6500
		if (lpa_freq > 640)
#endif
			/* At this point, the chip has not been re-configured, so the
			 * function can safely exit.
			 */
			return 1;

		if (!st.chip_cfg.int_motion_only)
		{
			/* Store current settings for later. */
			if (st.chip_cfg.dmp_on)
			{
				mpu_set_dmp_state(0);
				st.chip_cfg.cache.dmp_on = 1;
			}
			else
				st.chip_cfg.cache.dmp_on = 0;
			mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
			mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
			mpu_get_lpf(&st.chip_cfg.cache.lpf);
			mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
			st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
			mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
		}

#ifdef MPU6050
		/* Disable hardware interrupts for now. */
		set_int_enable(0);

		/* Enter full-power accel-only mode. */
		mpu_lp_accel_mode(0);

		/* Override current LPF (and HPF) settings to obtain a valid accel
		 * reading.
		 */
		data[0] = INV_FILTER_256HZ_NOLPF2;
		if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
			return 1;

		/* NOTE: Digital high pass filter should be configured here. Since this
		 * driver doesn't modify those bits anywhere, they should already be
		 * cleared by default.
		 */

		/* Configure the device to send motion interrupts. */
		/* Enable motion interrupt. */
		data[0] = BIT_MOT_INT_EN;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
			goto lp_int_restore;

		/* Set motion interrupt parameters. */
		data[0] = thresh_hw;
		data[1] = time;
		if (i2c_write(st.hw->addr, st.reg->motion_thr, 2, data))
			goto lp_int_restore;

		/* Force hardware to "lock" current accel sample. */
		delay_ms(5);
		data[0] = (st.chip_cfg.accel_fsr << 3) | BITS_HPF;
		if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
			goto lp_int_restore;

		/* Set up LP accel mode. */
		data[0] = BIT_LPA_CYCLE;
		if (lpa_freq == 1)
			data[1] = INV_LPA_1_25HZ;
		else if (lpa_freq <= 5)
			data[1] = INV_LPA_5HZ;
		else if (lpa_freq <= 20)
			data[1] = INV_LPA_20HZ;
		else
			data[1] = INV_LPA_40HZ;
		data[1] = (data[1] << 6) | BIT_STBY_XYZG;
		if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
			goto lp_int_restore;

		st.chip_cfg.int_motion_only = 1;
		return 0;
#elif defined MPU6500
		/* Disable hardware interrupts. */
		set_int_enable(0);

		/* Enter full-power accel-only mode, no FIFO/DMP. */
		data[0] = 0;
		data[1] = 0;
		data[2] = BIT_STBY_XYZG;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 3, data))
			goto lp_int_restore;

		/* Set motion threshold. */
		data[0] = thresh_hw;
		if (i2c_write(st.hw->addr, st.reg->motion_thr, 1, data))
			goto lp_int_restore;

		/* Set wake frequency. */
		if (lpa_freq == 1)
			data[0] = INV_LPA_1_25HZ;
		else if (lpa_freq == 2)
			data[0] = INV_LPA_2_5HZ;
		else if (lpa_freq <= 5)
			data[0] = INV_LPA_5HZ;
		else if (lpa_freq <= 10)
			data[0] = INV_LPA_10HZ;
		else if (lpa_freq <= 20)
			data[0] = INV_LPA_20HZ;
		else if (lpa_freq <= 40)
			data[0] = INV_LPA_40HZ;
		else if (lpa_freq <= 80)
			data[0] = INV_LPA_80HZ;
		else if (lpa_freq <= 160)
			data[0] = INV_LPA_160HZ;
		else if (lpa_freq <= 320)
			data[0] = INV_LPA_320HZ;
		else
			data[0] = INV_LPA_640HZ;
		if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, data))
			goto lp_int_restore;

		/* Enable motion interrupt (MPU6500 version). */
		data[0] = BITS_WOM_EN;
		if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
			goto lp_int_restore;

		/* Enable cycle mode. */
		data[0] = BIT_LPA_CYCLE;
		if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
			goto lp_int_restore;

		/* Enable interrupt. */
		data[0] = BIT_MOT_INT_EN;
		if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
			goto lp_int_restore;

		st.chip_cfg.int_motion_only = 1;
		return 0;
#endif
	}
	else
	{
		/* Don't "restore" the previous state if no state has been saved. */
		int ii;
		char *cache_ptr = (char*)&st.chip_cfg.cache;
		for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++)
		{
			if (cache_ptr[ii] != 0)
				goto lp_int_restore;
		}
		/* If we reach this point, motion interrupt mode hasn't been used yet. */
		return 1;
	}
lp_int_restore:
	/* Set to invalid values to ensure no I2C writes are skipped. */
	st.chip_cfg.gyro_fsr = 0xFF;
	st.chip_cfg.accel_fsr = 0xFF;
	st.chip_cfg.lpf = 0xFF;
	st.chip_cfg.sample_rate = 0xFFFF;
	st.chip_cfg.sensors = 0xFF;
	st.chip_cfg.fifo_enable = 0xFF;
	st.chip_cfg.clk_src = INV_CLK_PLL;
	mpu_set_sensors(st.chip_cfg.cache.sensors_on);
	mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
	mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
	mpu_set_lpf(st.chip_cfg.cache.lpf);
	mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
	mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);

	if (st.chip_cfg.cache.dmp_on)
		mpu_set_dmp_state(1);

#ifdef MPU6500
	/* Disable motion interrupt (MPU6500 version). */
	data[0] = 0;
	if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
		goto lp_int_restore;
#endif

	st.chip_cfg.int_motion_only = 0;
	return 0;
}

/**
 *  @}
 */

