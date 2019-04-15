/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 15498 $
 * $Date: 2017-08-16 10:47:15 +0800 (周三, 16 八月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef _LINUX_NVT_TOUCH_H
#define _LINUX_NVT_TOUCH_H

#undef pr_fmt
#define pr_fmt(fmt)	"NVT: %s: " fmt, __func__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include "nt36xxx_mem_map.h"

#define NVT_ITO_TEST 0

/* ---GPIO number--- */
#define NVTTOUCH_INT_PIN 943
#define NVT_POWER_SOURCE_CUST_EN 1

#if NVT_POWER_SOURCE_CUST_EN
#define LCM_LAB_MIN_UV 6000000
#define LCM_LAB_MAX_UV 6000000
#define LCM_IBB_MIN_UV 6000000
#define LCM_IBB_MAX_UV 6000000
#endif

/* ---INT trigger mode--- */
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING

/* ---I2C driver info.--- */
#define NVT_I2C_NAME "NVT-ts"
#define I2C_BLDR_Address 0x01
#define I2C_FW_Address 0x01
#define I2C_HW_Address 0x62

/* ---Input device info.--- */
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"
#define HWINFO_NAME "tp_wake_switch"

#if NVT_ITO_TEST
/* ---add ito test --- */
extern int32_t ito_selftest_open(void);
#endif

/* ---Touch info.--- */
#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2160
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#define TOUCH_FORCE_NUM 1000

/* ---Customerized func.--- */
#define NVT_TOUCH_PROC 1
#define MT_PROTOCOL_B 1
#define WAKEUP_GESTURE 1

#ifdef CONFIG_TOUCHSCREEN_NT36xxx_EXT_PROC
#define NVT_TOUCH_EXT_PROC 1
#else
#define NVT_TOUCH_EXT_PROC 0
#endif

#ifdef CONFIG_TOUCHSCREEN_NT36xxx_MP
#define NVT_TOUCH_MP 1
#else
#define NVT_TOUCH_MP 0
#endif

#define BOOT_UPDATE_FIRMWARE 1
#define DJ_BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw_dj.bin"
#define TXD_BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw_txd.bin"
/* #define BOOT_UPDATE_FIRMWARE_NAME "novatek_ts_fw.bin" */

/* ---ESD Protect.--- */
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500 /* ms */

struct nvt_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	int8_t phys[32];
	struct notifier_block fb_notif;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	int32_t reset_gpio;
	uint32_t reset_flags;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint16_t nvt_pid;
#if NVT_POWER_SOURCE_CUST_EN
	struct regulator *lcm_lab;
	struct regulator *lcm_ibb;
	atomic_t lcm_lab_power;
	atomic_t lcm_ibb_power;
#endif
};

#if NVT_TOUCH_PROC
struct nvt_flash_data {
	rwlock_t lock;
	struct i2c_client *client;
};
#endif

typedef enum {
	RESET_STATE_INIT = 0xA0, /* IC reset */
	RESET_STATE_REK, /* ReK baseline */
	RESET_STATE_REK_FINISH, /* baseline is ready */
	RESET_STATE_NORMAL_RUN, /* normal run */
	RESET_STATE_MAX = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
	EVENT_MAP_HOST_CMD = 0x50,
	EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE = 0x51,
	EVENT_MAP_RESET_COMPLETE = 0x60,
	EVENT_MAP_FWINFO = 0x78,
	EVENT_MAP_PROJECTID = 0x9A,
} I2C_EVENT_MAP;

/* ---extern structures--- */
extern struct nvt_ts_data *ts;
extern int nvt_TestResultLen;
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#if WAKEUP_GESTURE
extern const uint16_t gesture_key_array[];
#endif

/* ---extern functions--- */
extern int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address,
			    uint8_t *buf, uint16_t len);
extern int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address,
			     uint8_t *buf, uint16_t len);
extern void nvt_bootloader_reset(void);
extern void nvt_sw_reset_idle(void);
extern int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
extern int32_t nvt_get_fw_info(void);
extern int32_t nvt_clear_fw_status(void);
extern int32_t nvt_check_fw_status(void);

#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#endif /* _LINUX_NVT_TOUCH_H */
