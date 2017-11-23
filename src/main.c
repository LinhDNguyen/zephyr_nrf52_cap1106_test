/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <logging/sys_log.h>
#include <i2c.h>

#include "cap1106.h"

/* Change this if you have an LED connected to a custom port */
#define PORT	CONFIG_GPIO_NRF5_P0_DEV_NAME

#define I2C_DEV CONFIG_I2C_0_NAME

/* Change this if you have an LED connected to a custom pin */
#define LED1	0
#define LED2	30
#define LED3	31

#define ALERT_PIN 25

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000

static int cap_write_reg(struct device *i2c_dev, u8_t addr, u8_t data);
static int cap_read_reg(struct device *i2c_dev, u8_t addr, u8_t *data);
static int cap_reset_state(struct device *i2c_dev);
static int cap_read_touch(struct device *i2c_dev, int *latched, int *current);
static void led_set(struct device *gpio_dev, int val);
static void cap_init(struct device *i2c_dev);
void gpio_alert_hdl(struct device *port, struct gpio_callback *cb, u32_t pins);

static struct gpio_callback gpio_cb;
static struct device *i2c_dev = NULL;
static struct device *gpio_dev = NULL;
static int latched, current;
static int touch_ret = -1;
static int _test = 0;

void main(void)
{
	int ret;

	gpio_dev = device_get_binding(PORT);
	/* Set LED pin as output */
	gpio_pin_configure(gpio_dev, LED1, GPIO_DIR_OUT);
	gpio_pin_configure(gpio_dev, LED2, GPIO_DIR_OUT);
	gpio_pin_configure(gpio_dev, LED3, GPIO_DIR_OUT);

	ret = gpio_pin_configure(gpio_dev, ALERT_PIN, (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW));
	if (ret) {
		SYS_LOG_ERR("Error config interrupt pin %d!", ALERT_PIN);
		return;
	}
	gpio_init_callback(&gpio_cb, gpio_alert_hdl, BIT(ALERT_PIN));

	ret = gpio_add_callback(gpio_dev, &gpio_cb);
	if (ret) {
		SYS_LOG_ERR("Cannot setup callback!");
	}

	ret = gpio_pin_enable_callback(gpio_dev, ALERT_PIN);
	if (ret) {
		SYS_LOG_ERR("Error enabling callback!");
	}

	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) {
		SYS_LOG_ERR("I2C: Device driver not found.");
		return;
	}

	cap_init(i2c_dev);

	SYS_LOG_INF("BLINKY START...");

	while (1) {
		k_sleep(SLEEP_TIME);

		SYS_LOG_INF("latched %x, current %x", latched, current);
		// led_set(gpio_dev, current);
		SYS_LOG_DBG("_test %d, touch_ret %d", _test, touch_ret);
	}
}

static void cap_init(struct device *i2c_dev)
{
	int ret;
	u8_t reg;
	SYS_LOG_DBG("I2C master created");

	// Read some info
	ret = cap_read_reg(i2c_dev, CAP1106_REG_REVISION, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("I2C read revision failed, err %d", ret);
	} else {
		SYS_LOG_INF("Revision: 0x%x", reg);
	}

	ret = cap_read_reg(i2c_dev, CAP1106_REG_PRODUCT_ID, &reg);
	if (ret != 0) {
		SYS_LOG_ERR ("I2C read product id failed, err %d", ret);
	} else {
		SYS_LOG_INF("Product ID: 0x%x", reg);
	}

	ret = cap_read_reg(i2c_dev, CAP1106_REG_MANUFACTURER_ID, &reg);
	if (ret != 0) {
		SYS_LOG_ERR ("I2C read manufacturer id failed, err %d", ret);
	} else {
		SYS_LOG_INF("Manufacturer ID: 0x%x", reg);
	}

	ret = cap_read_reg(i2c_dev, CAP1106_REG_SENSOR_ENABLE, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("SENSOR ENABLE failed %d", ret);
		return;
	}
	SYS_LOG_DBG("SENSOR_ENABLE = 0x%x", reg);
	reg &= 0x05;
	// enable CS1-4
	ret = cap_write_reg(i2c_dev, CAP1106_REG_SENSOR_ENABLE, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write SENSOR ENABLE to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("write SENSOR_ENABLE = 0x%x", reg);

	ret = cap_read_reg(i2c_dev, CAP1106_REG_STANDBY_CHANNEL, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("STANDBY_CHANNEL failed %d", ret);
		return;
	}
	SYS_LOG_DBG("STANDBY_CHANNEL = 0x%x", reg);
	reg &= 0x05;
	// enable CS1-4 in standby mode
	ret = cap_write_reg(i2c_dev, CAP1106_REG_STANDBY_CHANNEL, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write STANDBY_CHANNEL to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("write STANDBY_CHANNEL = 0x%x", reg);

	// set sensitivity in standby mode
	reg = 0x02; // 0x01: 64x, 0x00: 128x, default 0x02: 32x
	ret = cap_write_reg(i2c_dev, CAP1106_REG_STANDBY_SENSITIVITY, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write STANDBY_SENSITIVITY to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("STANDBY_SENSITIVITY = 0x%x", reg);

	// set sensitivity, default 0x2f,
	reg = 0x45;
	ret = cap_write_reg(i2c_dev, CAP1106_REG_SENSITIVITY_CONTROL, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write SENSITIVITY_CONTROL to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("SENSITIVITY_CONTROL = 0x%x", reg);

	// set thresh hold in standby mode, default 0x40: 64,
	reg = 0x08; // thresh hold 32
	ret = cap_write_reg(i2c_dev, CAP1106_REG_STANDBY_THRESH, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write STANDBY_THRESH to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("STANDBY_THRESH = 0x%x", reg);

	ret = cap_read_reg(i2c_dev, CAP1106_REG_CONFIG, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("CONFIG failed %d", ret);
		return;
	}
	SYS_LOG_DBG("CONFIG = 0x%x", reg);
	reg |= 1 << 5;
	ret = cap_write_reg(i2c_dev, CAP1106_REG_CONFIG, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write CONFIG to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("write CONFIG = 0x%x", reg);

	// set repeat 525ms(1110), duration 7800ms(1100)
	reg = 0xCE;
	ret = cap_write_reg(i2c_dev, CAP1106_REG_SENSOR_CONFIG, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write SENSOR_CONFIG to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("SENSOR_CONFIG = 0x%x", reg);

	// set m press = 560ms
	reg = 0x0F;
	ret = cap_write_reg(i2c_dev, CAP1106_REG_SENSOR_CONFIG2, reg);
	if (ret != 0) {
		SYS_LOG_ERR("write SENSOR_CONFIG2 to 0x%x", reg);
		return;
	}
	SYS_LOG_DBG("SENSOR_CONFIG2 = 0x%x", reg);

	// Init ALERT GPIO pin

	SYS_LOG_INF("CAP1106 initialize DONE");
}

static int cap_write_reg(struct device *i2c_dev, u8_t addr, u8_t data)
{
	struct i2c_msg msgs[2];

	/* Setup I2C messages */

	/* Send the address to write to */
	msgs[0].buf = &addr;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Data to be written, and STOP after this. */
	msgs[1].buf = &data;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, CAP1106_SLAVE_ADDR);
}

static int cap_read_reg(struct device *i2c_dev, u8_t addr, u8_t *data)
{
	struct i2c_msg msgs[2];

	/* Setup I2C messages */

	/* Send the address to read from */
	msgs[0].buf = &addr;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	/* Read from device. STOP after this. */
	msgs[1].buf = data;
	msgs[1].len = 1;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, CAP1106_SLAVE_ADDR);
}

static int cap_read_touch(struct device *i2c_dev, int *latched, int *current)
{
	u8_t reg;
	int ret;

	ret = cap_read_reg(i2c_dev, CAP1106_REG_SENSOR_INPUT, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("read SENSOR_INPUT failed %d", reg);
		return -1;
	}
	if (latched) {
		*latched = reg;
		SYS_LOG_DBG("latched = 0x%x", reg);
	}

	ret = cap_reset_state(i2c_dev);
	if (ret != 0) {
		SYS_LOG_ERR("cap1106 reset state failed");
		return -2;
	}

	ret = cap_read_reg(i2c_dev, CAP1106_REG_SENSOR_INPUT, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("read SENSOR_INPUT failed %d", reg);
		return -3;
	}
	if (current) {
		*current = reg;
		SYS_LOG_DBG("TOUCH read 0x%x", reg);
	}

	return 0;
}


static int cap_reset_state(struct device *i2c_dev)
{
	u8_t reg;
	int ret = 0;

	ret = cap_read_reg(i2c_dev, CAP1106_REG_MAIN_CONTROL, &reg);
	if (ret != 0) {
		SYS_LOG_ERR("read MAIN_CONTROL failed %d", reg);
		return -1;
	}
	reg &= 0xFE;  // Clear the INT bit (bit 0)
	ret = cap_write_reg(i2c_dev, CAP1106_REG_MAIN_CONTROL, reg);
	if (0 != ret) {
		SYS_LOG_ERR("write MAIN_CONTROL to 0x%x", reg);
		return -2;
	}
	return 0;
}

static void led_set(struct device *gpio_dev, int val)
{
	gpio_pin_write(gpio_dev, LED1, (val & 0x01) ? 0 : 1);
	gpio_pin_write(gpio_dev, LED2, (val & 0x02) ? 0 : 1);
	gpio_pin_write(gpio_dev, LED3, (val & 0x04) ? 0 : 1);
}

static int scnt = 0;
void gpio_alert_hdl(struct device *port, struct gpio_callback *cb, u32_t pins)
{
	_test = 1;
	// touch_ret = cap_read_touch(i2c_dev, &latched, &current);
	if (scnt & 0x01) {
		led_set(gpio_dev, 0x7);
	} else {
		led_set(gpio_dev, 0x0);
	}
	scnt++;
	// led_set(gpio_dev, current);

	// SYS_LOG_INF("Now touch is 0x%x", current);
}