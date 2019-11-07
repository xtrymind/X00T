#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/switch.h>
#include "fingerprint_common.h"

#define DEV_NAME "common,fingerprint"

static atomic_t irq_sync = ATOMIC_INIT(0);
static int irq_flag;

static struct fp_common_resource fp_g;

static struct of_device_id fp_common_table[] = {
	{ .compatible = "common,fingerprint" },
	{ .compatible = "commonfp" },
	{},
};

#if USE_COMMON_PINCTRL
static int pinctrl_select_pin(struct pinctrl *p, char *name)
{
	int i, ret = -EINVAL;

	if (!name)
		return ret;

	for (i = 0; i < ARRAY_SIZE(fp_g.states); i++) {
		const char *state = pinctrl_names[i];

		if (!strcmp(state, name)) {
			ret = pinctrl_select_state(p, fp_g.states[i]);
			if (ret)
				goto exit;
			break;
		}
	}

	if (i == ARRAY_SIZE(fp_g.states)) {
		pr_err("%s: Can not find state %s\n", __func__, name);
		goto exit;
	}

	return ret;
exit:
	pr_err("%s: Can not select state %s\n", __func__, name);
	return ret;
}
#endif

int commonfp_power_on(void)
{
	gpio_direction_output(fp_g.pwr_gpio, 1);
	pr_info("%s: OK", __func__);
	return 0;
}

int commonfp_power_off(void)
{
	pr_info("%s: OK\n", __func__);
	return 0;
}

int commonfp_hw_reset(int ms)
{
	int ret = 0;
#if USE_COMMON_PINCTRL
	ret = pinctrl_select_pin(fp_g.fp_pinctrl, "commonfp_reset_active");
	if (ret)
		goto exit;

	mdelay(ms);

	ret = pinctrl_select_pin(fp_g.fp_pinctrl, "commonfp_reset_reset");
	if (ret)
		goto exit;

	mdelay(ms);

	ret = pinctrl_select_pin(fp_g.fp_pinctrl, "commonfp_reset_active");
	if (ret)
		goto exit;

	mdelay(ms);
#else
	gpio_direction_output(fp_g.rst_gpio, 1);
	mdelay(ms);
	gpio_set_value(fp_g.rst_gpio, 0);
	mdelay(ms);
	gpio_set_value(fp_g.rst_gpio, 1);
	mdelay(ms);
#endif

	pr_debug("%s: hw reset success,ret:%d\n", __func__, ret);
	return ret;
exit:
	pr_err("%s:  failed,ret:%d\n", __func__, ret);
	return ret;
}

int commonfp_request_irq(irq_handler_t handler, irq_handler_t thread_fn,
			 unsigned long flags, const char *name, void *dev)
{
	int ret = -EINVAL;

	if (handler == NULL && thread_fn == NULL)
		return ret;
	if (irq_flag == 1) {
		ret = 0;
		pr_info("%s: irq has been requested\n", __func__);
		return ret;
	}

	ret = request_threaded_irq(fp_g.irq_num, handler, thread_fn, flags,
				   name, dev);
	if (ret) {
		pr_err("%s: failed,error number is %d,irq = %d\n", __func__,
		       ret, fp_g.irq_num);
	}

	irq_flag = 1;
	return ret;
}

void commonfp_free_irq(void *dev_id)
{
	if (irq_flag == 0) {
		pr_debug("%s: irq has been free\n", __func__);
		return;
	}
	free_irq(fp_g.irq_num, dev_id);
	irq_flag = 0;
	pr_debug("%s: success\n", __func__);
}

void commonfp_irq_enable(void)
{
	if (!atomic_cmpxchg(&irq_sync, 0, 1)) {
		enable_irq(fp_g.irq_num);
		enable_irq_wake(fp_g.irq_num);
	}
}
void commonfp_irq_disable(void)
{
	if (atomic_cmpxchg(&irq_sync, 1, 0)) {
		disable_irq_wake(fp_g.irq_num);
		disable_irq(fp_g.irq_num);
	}
}

/*
 * it has been checked in the probe follow
 * so if ust it,the value is right
 */
#if USE_COMMON_PINCTRL

#else
int get_irq_gpio_number(void)
{
	return fp_g.irq_gpio;
}

int get_reset_gpio_number(void)
{
	return fp_g.rst_gpio;
}
#endif

static int fp_common_probe(struct platform_device *pdev)
{
	int ret, i;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	fp_g.pdev = pdev;

	pr_info("%s: start", __func__);

#if USE_COMMON_PINCTRL
	fp_g.fp_pinctrl = devm_pinctrl_get(&fp_g.pdev->dev);
	if (!fp_g.fp_pinctrl) {
		pr_err("%s: Failed to get common fp pinctrl\n", __func__);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(fp_g.states); i++) {
		struct pinctrl_state *state;

		state = pinctrl_lookup_state(fp_g.fp_pinctrl,
					     pinctrl_names[i]);
		if (IS_ERR(state)) {
			pr_err("%s: commonfp:cannot find '%s'\n", __func__,
			       pinctrl_names[i]);
			ret = -EINVAL;
			return ret;
		}

		fp_g.states[i] = state;
		ret = pinctrl_select_state(fp_g.fp_pinctrl, fp_g.states[i]);
		if (ret) {
			pr_err("%s: can not select state %s\n", __func__,
			       pinctrl_names[i]);
		} else {
			pr_err("%s: select state %s OK !!!\n", __func__,
			       pinctrl_names[i]);
		}
	}

	fp_g.irq_num = irq_of_parse_and_map(np, 0);

	ret = pinctrl_select_pin(fp_g.fp_pinctrl, "commonfp_irq_active");
	if (ret) {
		pr_err("%s: can not select commonfp_irq_active state\n",
		       __func__);
	} else {
		pr_info("%s: select commonfp_irq_active state  OK !!!\n",
			__func__);
	}

	//add for 1650 power
	fp_g.pwr_gpio = of_get_named_gpio(dev->of_node, "common,gpio_vdd", 0);
	ret = gpio_request(fp_g.pwr_gpio, "fingerprint_common_power");
	if (ret) {
		pr_err("%s: Failed to request pm660l  GPIO(5). err= %d\n",
		       __func__, ret);
		return -1;
	}

#else
	fp_g.irq_gpio = of_get_named_gpio(np, "common,gpio_irq", 0);
	fp_g.rst_gpio = of_get_named_gpio(np, "common,gpio_reset", 0);

	if (!gpio_is_valid(fp_g.rst_gpio)) {
		pr_err("%s: get reset gpio is invalid!\n", __func__);
		return -1;
	}

	if (!gpio_is_valid(fp_g.rst_gpio)) {
		pr_err("%s: get irq gpio is invalid!\n", __func__);
		return -1;
	}

	ret = devm_gpio_request(dev, fp_g.irq_gpio, "commonfp,gpio_irq");
	if (ret) {
		pr_err("%s: failed to request irq gpio %d\n", __func__,
		       p_g.irq_gpio);
		return -1;
	}

	ret = devm_gpio_request(dev, fp_g.rst_gpio, "commonfp,gpio_rst");
	if (ret) {
		pr_err("%s: failed to request rst gpio %d\n", __func__,
		       fp_g.rst_gpio);
		goto error;
	}

	fp_g.irq_num = gpio_to_irq(fp_g.irq_gpio);
	gpio_direction_input(fp_g.irq_gpio);
#endif

	irq_flag = 0;

	pr_err("%s: probe OK\n", __func__);

	return 0;

#if USE_COMMON_PINCTRL

#else
error:
	devm_gpio_free(dev, fp_g.irq_gpio);
	return -1;
#endif
}

static int fp_common_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#if USE_COMMON_PINCTRL
	if (!fp_g.fp_pinctrl) {
		devm_pinctrl_put(fp_g.fp_pinctrl);

		dev_info(dev, "commonfp pinctrl release success");
	}
#else
	if (gpio_is_valid(fp_g.irq_gpio)) {
		devm_gpio_free(dev, fp_g.irq_gpio);
		dev_err(dev, "remove irq_gpio success\n");
	}

	if (gpio_is_valid(fp_g.rst_gpio)) {
		devm_gpio_free(dev, fp_g.rst_gpio);
		dev_err(dev, "remove irq_gpio success\n");
	}

	dev_info(dev, "release all gpio suscess!!!");
#endif

	return 0;
}

static struct platform_driver fingerprint_common = {
	.driver = {
		.name = "common_fp",
		.owner = THIS_MODULE,
		.of_match_table = fp_common_table,
	},
	.probe = fp_common_probe,
	.remove = fp_common_remove,
};

static int __init fingerprint_resource_init(void)
{
	int ret;

	ret = platform_driver_register(&fingerprint_common);
	if (ret) {
		pr_err("%s: commonfp register platform driver failed:%d\n",
		       __func__, ret);
		return ret;
	}
	pr_info("%s: commonfp init OK!!!,ret=%d\n", __func__, ret);

	return ret;
}

module_init(fingerprint_resource_init);

static void __exit fingerprint_resource_exit(void)
{
	platform_driver_unregister(&fingerprint_common);
}

module_exit(fingerprint_resource_exit);

MODULE_LICENSE("GPL");
