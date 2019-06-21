/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 16485 $
 * $Date: 2017-09-14 10:10:52 +0800 (周四, 14 九月 2017) $
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include "nt36xxx.h"

#if NVT_ITO_TEST
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/device.h>
#endif

#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer;
uint8_t esd_check;
uint8_t esd_retry;
uint8_t esd_retry_max = 5;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_POWER_SOURCE_CUST_EN

static int nvt_lcm_bias_power_init(struct nvt_ts_data *data)
{
	int ret;

	data->lcm_lab = regulator_get(&data->client->dev, "lcm_lab");
	if (IS_ERR(data->lcm_lab)) {
		ret = PTR_ERR(data->lcm_lab);
		pr_err("Regulator get failed lcm_lab ret=%d", ret);
		goto _end;
	}
	if (regulator_count_voltages(data->lcm_lab) > 0) {
		ret = regulator_set_voltage(data->lcm_lab, LCM_LAB_MIN_UV,
					    LCM_LAB_MAX_UV);
		if (ret) {
			pr_err("Regulator set_vtg failed lcm_lab ret=%d", ret);
			goto reg_lcm_lab_put;
		}
	}
	data->lcm_ibb = regulator_get(&data->client->dev, "lcm_ibb");
	if (IS_ERR(data->lcm_ibb)) {
		ret = PTR_ERR(data->lcm_ibb);
		pr_err("Regulator get failed lcm_ibb ret=%d", ret);
		goto reg_set_lcm_lab_vtg;
	}
	if (regulator_count_voltages(data->lcm_ibb) > 0) {
		ret = regulator_set_voltage(data->lcm_ibb, LCM_IBB_MIN_UV,
					    LCM_IBB_MAX_UV);
		if (ret) {
			pr_err("Regulator set_vtg failed lcm_lab ret=%d", ret);
			goto reg_lcm_ibb_put;
		}
	}
	return 0;
reg_lcm_ibb_put:
	if (regulator_count_voltages(data->lcm_ibb) > 0)
		regulator_set_voltage(data->lcm_ibb, 0, LCM_IBB_MAX_UV);
reg_set_lcm_lab_vtg:
	data->lcm_ibb = NULL;
	regulator_put(data->lcm_ibb);
reg_lcm_lab_put:
	if (regulator_count_voltages(data->lcm_lab) > 0)
		regulator_set_voltage(data->lcm_lab, 0, LCM_LAB_MAX_UV);
_end:
	data->lcm_lab = NULL;
	regulator_put(data->lcm_lab);
	return ret;
}

static int nvt_lcm_bias_power_deinit(struct nvt_ts_data *data)
{
	if (data->lcm_ibb != NULL) {
		if (regulator_count_voltages(data->lcm_ibb) > 0)
			regulator_set_voltage(data->lcm_ibb, 0, LCM_LAB_MAX_UV);

		regulator_put(data->lcm_ibb);
	}
	if (data->lcm_lab != NULL) {
		if (regulator_count_voltages(data->lcm_lab) > 0)
			regulator_set_voltage(data->lcm_lab, 0, LCM_LAB_MAX_UV);

		regulator_put(data->lcm_lab);
	}
	return 0;
}

static int nvt_lcm_power_source_ctrl(struct nvt_ts_data *data, int enable)
{
	int rc;

	if (data->lcm_lab != NULL && data->lcm_ibb != NULL) {
		if (enable) {
			if (atomic_inc_return(&(data->lcm_lab_power)) == 1) {
				rc = regulator_enable(data->lcm_lab);
				if (rc) {
					atomic_dec(&(data->lcm_lab_power));
					pr_err("Regulator lcm_lab enable failed rc=%d",
						rc);
				}
			} else {
				atomic_dec(&(data->lcm_lab_power));
			}
			if (atomic_inc_return(&(data->lcm_ibb_power)) == 1) {
				rc = regulator_enable(data->lcm_ibb);
				if (rc) {
					atomic_dec(&(data->lcm_ibb_power));
					pr_err("Regulator lcm_ibb enable failed rc=%d",
						rc);
				}
			} else {
				atomic_dec(&(data->lcm_ibb_power));
			}
		} else {
			if (atomic_dec_return(&(data->lcm_lab_power)) == 0) {
				rc = regulator_disable(data->lcm_lab);
				if (rc) {
					atomic_inc(&(data->lcm_lab_power));
					pr_err("Regulator lcm_lab disable failed rc=%d",
						rc);
				}
			} else {
				atomic_inc(&(data->lcm_lab_power));
			}
			if (atomic_dec_return(&(data->lcm_ibb_power)) == 0) {
				rc = regulator_disable(data->lcm_ibb);
				if (rc) {
					atomic_inc(&(data->lcm_ibb_power));
					pr_err("Regulator lcm_ibb disable failed rc=%d",
						rc);
				}
			} else {
				atomic_inc(&(data->lcm_ibb_power));
			}
		}
	} else
		pr_err("Regulator lcm_ibb or lcm_lab is invalid");
	return 0;
}

#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

static struct workqueue_struct *nvt_wq;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = { KEY_BACK, KEY_HOME,
						  KEY_MENU };
#endif

#if WAKEUP_GESTURE
#define GESTURE_EVENT_C KEY_TP_GESTURE_C
#define GESTURE_EVENT_E KEY_TP_GESTURE_E
#define GESTURE_EVENT_S KEY_TP_GESTURE_S
#define GESTURE_EVENT_V KEY_TP_GESTURE_V
#define GESTURE_EVENT_W KEY_TP_GESTURE_W
#define GESTURE_EVENT_Z KEY_TP_GESTURE_Z
#define GESTURE_EVENT_SWIPE_UP 0x2f6
#define GESTURE_EVENT_DOUBLE_CLICK 0x2f7

const uint16_t gesture_key_array[] = {
	GESTURE_EVENT_C, GESTURE_EVENT_W,
	GESTURE_EVENT_V, GESTURE_EVENT_DOUBLE_CLICK,
	GESTURE_EVENT_Z, KEY_M,
	KEY_O,		 GESTURE_EVENT_E,
	GESTURE_EVENT_S, GESTURE_EVENT_SWIPE_UP,
	KEY_POWER,       KEY_POWER,
	KEY_POWER,
};
#endif

static uint8_t bTouchIsAwake;

#if WAKEUP_GESTURE
#define NVT_GESTURE_MODE "tpd_gesture"

static long gesture_mode;

static ssize_t nvt_gesture_mode_get_proc(struct file *file, char __user *buffer,
					 size_t size, loff_t *ppos)
{
	char ptr[64] = {0};
	unsigned int len = 0;
	unsigned int ret = 0;

	if (gesture_mode == 0)
		len = sprintf(ptr, "0\n");
	else
		len = sprintf(ptr, "1\n");

	ret = simple_read_from_buffer(buffer, size, ppos, ptr, (size_t)len);
	return ret;
}

static ssize_t nvt_gesture_mode_set_proc(struct file *filp,
					 const char __user *buffer,
					 size_t count, loff_t *off)
{
	char msg[20] = {0};
	int ret = 0;

	if (!bTouchIsAwake) {
		pr_info("Touch is already sleep cant modify gesture node\n");
		return count;
	}

	ret = copy_from_user(msg, buffer, count);
	pr_info("msg = %s\n", msg);
	if (ret)
		return -EFAULT;

	ret = kstrtol(msg, 0, &gesture_mode);
	if (!ret) {
		if (gesture_mode == 0)
			gesture_mode = 0;
		else
			gesture_mode = 0x1FF;
	} else {
		pr_err("set gesture mode failed\n");
	}
	pr_info("gesture_mode = 0x%x\n", (unsigned int)gesture_mode);

	return count;
}

static struct proc_dir_entry *nvt_gesture_mode_proc;
static const struct file_operations gesture_mode_proc_ops = {
	.owner = THIS_MODULE,
	.read = nvt_gesture_mode_get_proc,
	.write = nvt_gesture_mode_set_proc,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf,
		     uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = address;
	msgs[0].len = 1;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = address;
	msgs[1].len = len - 1;
	msgs[1].buf = &buf[1];

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		pr_err("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf,
		      uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr = address;
	msg.len = len;
	msg.buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		pr_err("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	uint8_t buf[4] = { 0 };

	/* ---write i2c cmds to reset idle--- */
	buf[0] = 0x00;
	buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	usleep_range(14000, 15000);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = { 0 };

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	/* need 35ms delay after bootloader reset */
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = { 0 };
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		/* ---set xdata index to EVENT BUF ADDR--- */
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		/* ---clear fw status--- */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		/* ---read fw status--- */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 11000);
	}

	if (i >= retry) {
		pr_err("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = { 0 };
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		/* ---set xdata index to EVENT BUF ADDR--- */
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		/* ---read fw status--- */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 11000);
	}

	if (i >= retry) {
		pr_err("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = { 0 };
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		usleep_range(10000, 11000);

		/* ---read reset state--- */
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) &&
		    (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if (unlikely(retry > 100)) {
			pr_err("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = { 0 };
	int32_t ret = 0;

	/* ---set xdata index to EVENT BUF ADDR--- */
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	/* ---read project id--- */
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	pr_info("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = { 0 };
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	/* ---set xdata index to EVENT BUF ADDR--- */
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	/* ---read fw info--- */
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	/* ---clear x_num, y_num if fw info is broken--- */
	if ((buf[1] + buf[2]) != 0xFF) {
		pr_err("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n",
			buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts->max_button_num = TOUCH_KEY_NUM;

		if (retry_count < 3) {
			retry_count++;
			pr_err("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			pr_err("Set default fw_ver=%d, x_num=%d, y_num=%d, abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max,
					ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	/* ---Get Novatek PID--- */
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME "NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff,
			      size_t count, loff_t *offp)
{
	uint8_t str[68] = { 0 };
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

	if (count > sizeof(str)) {
		pr_err("error count=%zu\n", count);
		return -EFAULT;
	}

	if (copy_from_user(str, buff, count)) {
		pr_err("copy from user error\n");
		return -EFAULT;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ
	 * after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) { //I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F),
					    &str[2], str[1]);
			if (ret == 1)
				break;

			pr_err("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			pr_err("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) { /* I2C read */
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2],
					   str[1]);
			if (ret == 2)
				break;

			pr_err("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		/* copy buff to user if i2c transfer */
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			pr_err("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	}

	pr_err("Call error, str[0]=%d\n", str[0]);
	return -EFAULT;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		pr_err("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL, &nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		pr_err("Failed!\n");
		return -ENOMEM;
	}

	pr_info("Succeeded!\n");

	pr_info("===========================================\n");
	pr_info("Create /proc/NVTflash\n");
	pr_info("===========================================\n");

	return 0;
}
#endif

#if NVT_ITO_TEST
/**********add ito test mode function  *******************/
int nvt_TestResultLen;
static struct platform_device hwinfo_device = {
	.name = HWINFO_NAME,
	.id = -1,
};

static ssize_t ito_test_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int count;

	ito_selftest_open();
	count = snprintf(buf, sizeof(buf), "%d\n", nvt_TestResultLen);
	return count;
}

static ssize_t ito_test_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(factory_check, 0644, ito_test_show, ito_test_store);

static struct attribute *ito_test_attributes[] = {

	&dev_attr_factory_check.attr, NULL
};
static struct attribute_group ito_test_attribute_group = {

	.attrs = ito_test_attributes

};
int nvt_test_node_init(struct platform_device *tpinfo_device)
{
	int err = 0;

	err = sysfs_create_group(&tpinfo_device->dev.kobj,
				 &ito_test_attribute_group);
	if (err != 0) {
		pr_err("[nvt-ito] %s() - ERROR: sysfs_create_group() failed.",
		       __func__);
		sysfs_remove_group(&tpinfo_device->dev.kobj,
				   &ito_test_attribute_group);
		return -EIO;
	}

	pr_debug("[nvt-ito] %s() - sysfs_create_group() succeeded.", __func__);
	return err;
}
/*************************************************/
#endif

#if WAKEUP_GESTURE
#define ID_GESTURE_WORD_C 12
#define ID_GESTURE_WORD_W 13
#define ID_GESTURE_WORD_V 14
#define ID_GESTURE_DOUBLE_CLICK 15
#define ID_GESTURE_WORD_Z 16
/* #define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18 */
#define ID_GESTURE_WORD_e 19
#define ID_GESTURE_WORD_S 20
#define ID_GESTURE_SLIDE_UP 21
/* #define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24 */

/* customized gesture id */
#define DATA_PROTOCOL 30
/* function page definition */
#define FUNCPAGE_GESTURE 1

static struct wakeup_source gestrue_wakelock;

#define MASK_GESTURE_DOUBLE_CLICK 0x101
#define MASK_GESTURE_SLIDE_UP 0x102
#define MASK_GESTURE_V 0x104
#define MASK_GESTURE_Z 0x108
#define MASK_GESTURE_C 0x110
#define MASK_GESTURE_E 0x120
#define MASK_GESTURE_S 0x140
#define MASK_GESTURE_W 0x180

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		pr_err("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	pr_info("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
	case ID_GESTURE_WORD_C:
		if (gesture_mode & MASK_GESTURE_C) {
			pr_info("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
		}
		break;
	case ID_GESTURE_WORD_W:
		if (gesture_mode & MASK_GESTURE_W) {
			pr_info("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
		}
		break;
	case ID_GESTURE_WORD_V:
		if (gesture_mode & MASK_GESTURE_V) {
			pr_info("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
		}
		break;
	case ID_GESTURE_DOUBLE_CLICK:
		if (gesture_mode & MASK_GESTURE_DOUBLE_CLICK) {
			pr_info("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
		}
		break;
	case ID_GESTURE_WORD_Z:
		if (gesture_mode & MASK_GESTURE_Z) {
			pr_info("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
		}
		break;
	/* case GESTURE_WORD_M:
			pr_info("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			pr_info("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break; */
	case ID_GESTURE_WORD_e:
		if (gesture_mode & MASK_GESTURE_E) {
			pr_info("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
		}
		break;
	case ID_GESTURE_WORD_S:
		if (gesture_mode & MASK_GESTURE_W) {
			pr_info("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
		}
		break;
	case ID_GESTURE_SLIDE_UP:
		if (gesture_mode & MASK_GESTURE_SLIDE_UP) {
			pr_info("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
		}
		break;
		/* case GESTURE_SLIDE_DOWN:
			pr_info("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			pr_info("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			pr_info("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break; */
	default:
		break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static void nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0,
					       &ts->irq_flags);
	pr_info("novatek,irq-gpio=%d\n", ts->irq_gpio);
}
#else
static void nvt_parse_dt(struct device *dev)
{
	ts->irq_gpio = NVTTOUCH_INT_PIN;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			pr_err("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* enable/disable esd check flag */
	esd_check = enable;
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i = 1; i < 7; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	if (esd_retry >= esd_retry_max)
		nvt_esd_check_enable(false);

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		/* do esd recovery, bootloader reset */
		nvt_bootloader_reset();
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			   msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static void nvt_ts_work_func(struct work_struct *work)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = { 0 };
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = { 0 };
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;

	mutex_lock(&ts->lock);
	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data,
			   POINT_DATA_LEN + 1);
	if (ret < 0) {
		pr_err("CTP_I2C_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		input_id = (uint8_t)(point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		enable_irq(ts->client->irq);
		mutex_unlock(&ts->lock);
		return;
	}
#endif

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) ||
		    ((point_data[position] & 0x07) ==
		     0x02)) { /* finger down (enter & moving) */
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif
			input_x = (uint32_t)(point_data[position + 1] << 4) +
				  (uint32_t)(point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) +
				  (uint32_t)(point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max) ||
			    (input_y > ts->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) +
					  (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
					 input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					 input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, false);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i],
					 ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++)
			input_report_key(ts->input_dev, touch_key_array[i], 0);
	}
#endif

	input_sync(ts->input_dev);

XFER_ERROR:
	enable_irq(ts->client->irq);
	mutex_unlock(&ts->lock);
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	disable_irq_nosync(ts->client->irq);

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0)
		__pm_wakeup_event(&gestrue_wakelock, msecs_to_jiffies(5000));
#endif

	queue_work(nvt_wq, &ts->nvt_work);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = { 0 };
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nvt_bootloader_reset(); // NOT in retry loop

	/* ---Check for 5 times--- */
	for (retry = 5; retry > 0; retry--) {
		nvt_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		usleep_range(10000, 11000);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);

		pr_info("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		/* compare read chip id on supported list */
		for (list = 0; list < (sizeof(trim_id_table) /
				       sizeof(struct nvt_ts_trim_id_table));
		     list++) {
			found_nvt_chip = 0;

			/* compare each byte */
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] !=
					    trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX)
				found_nvt_chip = 1;

			if (found_nvt_chip) {
				pr_info("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system =
					trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		usleep_range(10000, 11000);
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif

	pr_info("start\n");

	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* ---parse dts--- */
	nvt_parse_dt(&client->dev);

#if NVT_POWER_SOURCE_CUST_EN
	atomic_set(&(ts->lcm_lab_power), 0);
	atomic_set(&(ts->lcm_ibb_power), 0);
	ret = nvt_lcm_bias_power_init(ts);

	if (ret) {
		pr_err("power resource init error!\n");
		goto err_power_resource_init_fail;
	}

	nvt_lcm_power_source_ctrl(ts, 1);
#endif

	/* ---request and config GPIOs--- */
	ret = nvt_gpio_config(ts);
	if (ret) {
		pr_err("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	/* ---check i2c func.--- */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed. (no I2C_FUNC_I2C)\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* need 10ms delay after POR(power on reset) */
	usleep_range(10000, 11000);

	/* ---check chip version trim--- */
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		pr_err("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->lock);

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();
	mutex_unlock(&ts->lock);

	/* ---create workqueue--- */
	nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		pr_err("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts->nvt_work, nvt_ts_work_func);

	/* ---allocate input device--- */
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		pr_err("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;

	/* ---set input device info.--- */
	ts->input_dev->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM,
			     0, 0); /* pressure = TOUCH_FORCE_NUM */

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0,
			     0); /* area = 255 */

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
#if MT_PROTOCOL_B
	/* no need to set ABS_MT_TRACKING_ID,
	 * input_mt_init_slots() already set it
	 */
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			     ts->max_touch_num, 0, 0);
#endif /* MT_PROTOCOL_B */
#endif /* TOUCH_MAX_FINGER_NUM > 1 */

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY,
				     touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0;
	     retry < (ARRAY_SIZE(gesture_key_array));
	     retry++) {
		input_set_capability(ts->input_dev, EV_KEY,
				     gesture_key_array[retry]);
	}
	wakeup_source_init(&gestrue_wakelock, "poll-wake-lock");
#endif

	snprintf(ts->phys, sizeof(ts->phys), "%s", "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	/* ---register input device--- */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		pr_err("register input device (%s) failed. ret=%d\n",
			ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	/* ---set int-pin & request irq--- */
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		pr_info("int_trigger_type=%d\n", ts->int_trigger_type);

#if WAKEUP_GESTURE
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				  ts->int_trigger_type | IRQF_NO_SUSPEND,
				  client->name, ts);
#else
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				  ts->int_trigger_type, client->name, ts);
#endif
		if (ret != 0) {
			pr_err("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			disable_irq(client->irq);
			pr_info("request irq %d succeed\n", client->irq);
		}
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		pr_err("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	/* please make sure boot update start after display reset(RESX) sequence */
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work,
			   msecs_to_jiffies(10000));
#endif
#if NVT_ITO_TEST
	/* ---add ito node--- */
	platform_device_register(&hwinfo_device);
	nvt_test_node_init(&hwinfo_device);
#endif

/********************add protect , 20170908***********************/
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			   msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	/* ---set device node--- */
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		pr_err("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		pr_err("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		pr_err("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if WAKEUP_GESTURE
	nvt_gesture_mode_proc = proc_create(NVT_GESTURE_MODE, 0666, NULL,
					    &gesture_mode_proc_ops);
	if (!nvt_gesture_mode_proc)
		pr_err("create proc tpd_gesture failed\n");
#endif

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret) {
		pr_err("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}

	bTouchIsAwake = 1;
	pr_info("end\n");
	enable_irq(client->irq);

	return 0;

err_register_fb_notif_failed:
#if (NVT_TOUCH_PROC || NVT_TOUCH_EXT_PROC || NVT_TOUCH_MP)
err_init_NVT_ts:
#endif
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_create_nvt_wq_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
err_check_functionality_failed:
	gpio_free(ts->irq_gpio);
err_gpio_config_failed:
	nvt_lcm_power_source_ctrl(ts, 0);
	nvt_lcm_bias_power_deinit(ts);
err_power_resource_init_fail:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	if (fb_unregister_client(&ts->fb_notif))
		pr_err("Error occurred while unregistering fb_notifier.\n");

	mutex_destroy(&ts->lock);

	pr_info("Removing driver...\n");

	free_irq(client->irq, ts);
	nvt_lcm_bias_power_deinit(ts);
	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	struct nvt_ts_data *data = dev_get_drvdata(dev);
	uint8_t buf[4] = { 0 };
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	if (!bTouchIsAwake) {
		pr_debug("Touch is already suspend\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	bTouchIsAwake = 0;

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE
	if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) {
		disable_irq(ts->client->irq);

		/* ---write i2c command to enter "deep sleep mode"--- */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	} else {
		/* ---write i2c command to enter "wakeup gesture mode"--- */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		enable_irq_wake(ts->client->irq);

		pr_info("Enter gesture mode\n");
	}
#else /* WAKEUP_GESTURE */
	disable_irq(ts->client->irq);

	/* ---write i2c command to enter "deep sleep mode"--- */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif /* WAKEUP_GESTURE */

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);

	mutex_unlock(&ts->lock);

#if NVT_POWER_SOURCE_CUST_EN
	if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0)) {
		nvt_lcm_power_source_ctrl(data, 0); //disable vsp/vsn
		pr_info("sleep suspend end disable vsp/vsn\n");
	} else {
		pr_info("gesture suspend end not disable vsp/vsn\n");
	}
#endif /* #if NVT_POWER_SOURCE_CUST_EN */

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	struct nvt_ts_data *data = dev_get_drvdata(dev);

	nvt_lcm_power_source_ctrl(data, 1); /* enable vsp/vsn */

	if (bTouchIsAwake) {
		pr_debug("Touch is already resume\n");
		return 0;
	}

	mutex_lock(&ts->lock);

	/* please make sure display reset(RESX) sequence and
	 * mipi dsi cmds sent before this
	 */
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_REK);

	if (((gesture_mode & 0x100) == 0) || ((gesture_mode & 0x0FF) == 0))
		enable_irq(ts->client->irq);

#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			   msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;
	mutex_unlock(&ts->lock);

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data) {
		blank = evdata->data;
		if (event == FB_EARLY_EVENT_BLANK && *blank == FB_BLANK_POWERDOWN)
			nvt_ts_suspend(&ts->client->dev);
		else if (event == FB_EVENT_BLANK && *blank == FB_BLANK_UNBLANK)
			nvt_ts_resume(&ts->client->dev);
	}

	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = { { NVT_I2C_NAME, 0 }, {} };

#ifdef CONFIG_OF
static const struct of_device_id nvt_match_table[] = {
	{
		.compatible = "novatek,NVT-ts",
	},
	{},
};
#endif

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	pr_info("start\n");
	//---add i2c driver---
	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		pr_err("%s: failed to add i2c driver", __func__);
		goto err_driver;
	}

	pr_info("finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);

	if (nvt_wq)
		destroy_workqueue(nvt_wq);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

/* late_initcall(nvt_driver_init); */
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
