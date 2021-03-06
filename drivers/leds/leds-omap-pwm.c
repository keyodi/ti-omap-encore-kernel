/* drivers/leds/leds-omap_pwm.c
 *
 * Driver to blink LEDs using OMAP PWM timers
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Timo Teras
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/ctype.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/delay.h>
#include <plat/board.h>
#include <plat/dmtimer.h>

#define NO_LED_FULL /* to avoid led flickering, never set brightness to FULL */

struct omap_pwm_led {
	struct led_classdev cdev;
	struct work_struct work;
	struct delayed_work disable_int_work;
	struct omap_pwm_led_platform_data *pdata;
	struct omap_dm_timer *intensity_timer;
	struct omap_dm_timer *blink_timer;
	int powered;
	unsigned int on_period, off_period;
	enum led_brightness brightness;
	atomic_t cached_brightness;
};

static inline struct omap_pwm_led *pdev_to_omap_pwm_led(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}

static inline struct omap_pwm_led *cdev_to_omap_pwm_led(struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct omap_pwm_led, cdev);
}

static inline struct omap_pwm_led *work_to_omap_pwm_led(struct work_struct *work)
{
	return container_of(work, struct omap_pwm_led, work);
}

static void omap_pwm_led_set_blink(struct omap_pwm_led *led)
{
	int def_on = 1;

	if ( led->pdata )
		def_on = led->pdata->def_on;

	if (!led->powered)
		return;

	if (led->on_period != 0 && led->off_period != 0) {
		unsigned long load_reg, cmp_reg;

		load_reg = 32768 * (led->on_period + led->off_period) / 1000;
		cmp_reg = 32768 * led->on_period / 1000;

		omap_dm_timer_stop(led->blink_timer);
		omap_dm_timer_set_load(led->blink_timer, 1, -load_reg);
		omap_dm_timer_set_match(led->blink_timer, 1, -cmp_reg);
		omap_dm_timer_set_pwm(led->blink_timer, 
					  def_on, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_write_counter(led->blink_timer, -2);
		omap_dm_timer_start(led->blink_timer);
	} else {
		omap_dm_timer_set_pwm(led->blink_timer, 
					  def_on, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(led->blink_timer);
	}
}

/*
	Setting the timer match value when PWM is high (not yet toggled to low on MATCH)
	could cause the led flickering issue.  Therefore, to eliminate the flickering we
	set the new brightness (match) value after PWM goes down, not immediately upon
	receiving the brightness change request, that is, do it in the MATCH interrupt handler.
	
	The MATCH interrupt is disabled after the new match value is set to avoid performance hit.
	However interrupt cannot be disabled immediately in the interrupt handler as it will cause 
	flickering when the next change request comes in, for some unknown reasons.  Therefore, we
	disable match interrupt in a delayed (1/8s) work. So the match interrupt will last for 1/8s,
	which is about 15 ISR calls. Performance wise this shall be OK.
*/
static irqreturn_t intensity_timer_match_interrupt(int irq, void *arg)
{
	struct omap_pwm_led *led = (struct omap_pwm_led*) arg;
	struct omap_dm_timer* timer = (struct omap_dm_timer*)led->intensity_timer;

	/* clear interrupt status bit */
	omap_dm_timer_write_status(timer, OMAP_TIMER_INT_MATCH);

	/* set new match value */
	omap_dm_timer_set_match(timer, 1, (0xffffff00) | atomic_read(&led->cached_brightness));

	return IRQ_HANDLED;
}

static void omap_pwm_disable_int_work(struct work_struct *work)
{
    struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct omap_pwm_led* led =  container_of(dw, struct omap_pwm_led, disable_int_work);

	/* disable match interrupt*/
	omap_dm_timer_set_int_enable(led->intensity_timer, 0);	
}

static void omap_pwm_led_power_on(struct omap_pwm_led *led)
{
	int err;

	if (led->powered)
		return;
	led->powered = 1;

	/* Select clock */
	omap_dm_timer_enable(led->intensity_timer);
	omap_dm_timer_set_source(led->intensity_timer, OMAP_TIMER_SRC_32_KHZ);

	/* Turn voltage on */
	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 1);

	/* Enable PWM timers */
	if (led->blink_timer != NULL) {
		omap_dm_timer_enable(led->blink_timer);
		omap_dm_timer_set_source(led->blink_timer,
					 OMAP_TIMER_SRC_32_KHZ);
		omap_pwm_led_set_blink(led);
	}

	// register timer match interrupt
	err = request_irq(omap_dm_timer_get_irq(led->intensity_timer), intensity_timer_match_interrupt,
				IRQF_DISABLED, "led intensity timer", (void*)led);
	if (err) {
		printk(KERN_ERR "omap_pwm_led_power_on : unable to intensity gptimer IRQ\n");
	}

	omap_dm_timer_set_load(led->intensity_timer, 1, 0xffffff00);
}

static void omap_pwm_led_power_off(struct omap_pwm_led *led)
{
	int def_on = 1;

	if (!led->powered)
		return;
	led->powered = 0;

	/* cancel the pending disable interrupt work */
	cancel_delayed_work_sync(&led->disable_int_work);
	/* disable timer match interrupt */
	omap_dm_timer_set_int_enable(led->intensity_timer, 0);
	/* free irq, as corresponding to requst_irq() in omap_pwm_led_power_on*/
	free_irq(omap_dm_timer_get_irq(led->intensity_timer), (void*)led);
	/* mark the next brightness request as first request */
	atomic_set(&led->cached_brightness, -1);

	if ( led->pdata )
		def_on = led->pdata->def_on;

	/* Everything off */
	omap_dm_timer_set_pwm(led->intensity_timer, 
				  def_on ? 0 : 1, 1,
				  OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_stop(led->intensity_timer);
	omap_dm_timer_disable(led->intensity_timer);

	if (led->blink_timer != NULL) {
		omap_dm_timer_stop(led->blink_timer);
		omap_dm_timer_disable(led->blink_timer);
	}

	if (led->pdata->set_power != NULL)
		led->pdata->set_power(led->pdata, 0);
}

static void omap_pwm_led_set_pwm_cycle(struct omap_pwm_led *led, int cycle)
{
	unsigned int timerval;
	int def_on = 1;

	if ( led->pdata )
		def_on = led->pdata->def_on;

#ifdef NO_LED_FULL	   
	/* Reducing brightness starting from FULL (255) will result in
	   backlight flickering.  Therefore, we give up the LED_FULL value
	   and never set it to FULL as the brightness difference between 
	   255 (FULL) and 254 is ignorable.  As a result the next 
	   if (cycle == LED_FULL) block is actually disabled. */
	if (cycle == LED_FULL) {
		cycle = LED_FULL - 1;
	}
#endif

	if (cycle == LED_FULL || cycle == LED_OFF) {
		omap_dm_timer_set_pwm(led->intensity_timer, 
					  def_on ? 1 : 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(led->intensity_timer);
	} else {
		/* If this is the first time to set the brightness after led power on? */
		int first_request = (atomic_read(&led->cached_brightness) == -1)? 1 : 0;

		/* Cache the new brightness request and set it in the match interrupt handler afterwards. */
		atomic_set(&led->cached_brightness, cycle);

		omap_dm_timer_set_pwm(led->intensity_timer, 
					  def_on ? 0 : 1, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

		if (first_request) {
			/* if this is the first request after backlight is turned on, we have to set the match directly.  */
			omap_dm_timer_set_match(led->intensity_timer, 1,
						(0xffffff00) | cycle);
		} else {
			/* Don't set the new brightness here, but set it in the match interrupt handler. */

			/* Cancel the pending disable interrupt work */
			cancel_delayed_work_sync(&led->disable_int_work);

			/* At this point timer match interrupt must have been disabled, so enable it. */
			omap_dm_timer_set_int_enable(led->intensity_timer, OMAP_TIMER_INT_MATCH);

			/* Schedule to disable interrupt to avoid performance hit. 
			   Note that we cannot disable interrupt immediately in the interrupt handler 
			   because for some unknown reason it will cause flickering when the next brightness 
			   request comes in. We have to let the interrupt last for 1/8 s and then disalbe it
			   in the delayed work. */
			schedule_delayed_work(&led->disable_int_work, HZ >> 3);
		}

		/* ensure timer value is in range */
		timerval = omap_dm_timer_read_counter(led->intensity_timer);
		if (timerval < 0xffffff00)
			omap_dm_timer_write_counter(led->intensity_timer, -2);

		omap_dm_timer_start(led->intensity_timer);
	}
}

static void omap_pwm_led_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	if (value != led->brightness) {
		led->brightness = value;
		schedule_work(&led->work);
	}
}

static void omap_pwm_led_work(struct work_struct *work)
{
	struct omap_pwm_led *led = work_to_omap_pwm_led(work);

	if (led->brightness != LED_OFF) {
		omap_pwm_led_power_on(led);
		omap_pwm_led_set_pwm_cycle(led, led->brightness);
	} else {
		omap_pwm_led_set_pwm_cycle(led, led->brightness);
		omap_pwm_led_power_off(led);
	}
}

static ssize_t omap_pwm_led_on_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	return sprintf(buf, "%u\n", led->on_period) + 1;
}

static ssize_t omap_pwm_led_on_period_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		led->on_period = val;
		omap_pwm_led_set_blink(led);
		ret = count;
	}

	return ret;
}

static ssize_t omap_pwm_led_off_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);

	return sprintf(buf, "%u\n", led->off_period) + 1;
}

static ssize_t omap_pwm_led_off_period_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct omap_pwm_led *led = cdev_to_omap_pwm_led(led_cdev);
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		led->off_period = val;
		omap_pwm_led_set_blink(led);
		ret = count;
	}

	return ret;
}

static DEVICE_ATTR(on_period, 0644, omap_pwm_led_on_period_show,
				omap_pwm_led_on_period_store);
static DEVICE_ATTR(off_period, 0644, omap_pwm_led_off_period_show,
				omap_pwm_led_off_period_store);

static int omap_pwm_led_probe(struct platform_device *pdev)
{
	struct omap_pwm_led_platform_data *pdata = pdev->dev.platform_data;
	struct omap_pwm_led *led;
	int ret;

	led = kzalloc(sizeof(struct omap_pwm_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(&pdev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led);
	led->cdev.brightness_set = omap_pwm_led_set;
	led->cdev.default_trigger = NULL;
	led->cdev.name = pdata->name;
	led->pdata = pdata;
	led->brightness = pdata->def_brightness;
	atomic_set(&led->cached_brightness, -1);
	INIT_WORK(&led->work, omap_pwm_led_work);
	INIT_DELAYED_WORK(&led->disable_int_work, omap_pwm_disable_int_work);

	dev_info(&pdev->dev, "OMAP PWM LED (%s) at GP timer %d/%d\n",
		 pdata->name, pdata->intensity_timer, pdata->blink_timer);

	if (pdata->def_brightness) {
		led->cdev.brightness = pdata->def_brightness;
	}
	/* register our new led device */
	ret = led_classdev_register(&pdev->dev, &led->cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "led_classdev_register failed\n");
		goto error_classdev;
	}

	/* get related dm timers */
	led->intensity_timer = omap_dm_timer_request_specific(pdata->intensity_timer);
	if (led->intensity_timer == NULL) {
		dev_err(&pdev->dev, "failed to request intensity pwm timer\n");
		ret = -ENODEV;
		goto error_intensity;
	}
	omap_dm_timer_disable(led->intensity_timer);

	if (pdata->blink_timer != 0) {
		led->blink_timer = omap_dm_timer_request_specific(pdata->blink_timer);
		if (led->blink_timer == NULL) {
			dev_err(&pdev->dev, "failed to request blinking pwm timer\n");
			ret = -ENODEV;
			goto error_blink1;
		}
		omap_dm_timer_disable(led->blink_timer);

		ret = device_create_file(led->cdev.dev,
					       &dev_attr_on_period);
		if(ret)
			goto error_blink2;

		ret = device_create_file(led->cdev.dev,
					&dev_attr_off_period);
		if(ret)
			goto error_blink3;

	}
	if (led->brightness) {
		schedule_work(&led->work);
	} else {
		omap_pwm_led_power_on(led);
		omap_pwm_led_power_off(led);
	}
	return 0;

error_blink3:
	device_remove_file(led->cdev.dev,
				 &dev_attr_on_period);
error_blink2:
	dev_err(&pdev->dev, "failed to create device file(s)\n");
error_blink1:
	omap_dm_timer_free(led->intensity_timer);
error_intensity:
	led_classdev_unregister(&led->cdev);
error_classdev:
	kfree(led);
	return ret;
}

static int omap_pwm_led_remove(struct platform_device *pdev)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);


	device_remove_file(led->cdev.dev,
				 &dev_attr_on_period);
	device_remove_file(led->cdev.dev,
				 &dev_attr_off_period);
	led_classdev_unregister(&led->cdev);

	omap_pwm_led_set(&led->cdev, LED_OFF);
	if (led->blink_timer != NULL)
		omap_dm_timer_free(led->blink_timer);
	omap_dm_timer_free(led->intensity_timer);
	kfree(led);

	return 0;
}


#ifdef CONFIG_PM
static int omap_pwm_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

	led_classdev_suspend(&led->cdev);
	return 0;
}

static int omap_pwm_led_resume(struct platform_device *pdev)
{
	struct omap_pwm_led *led = pdev_to_omap_pwm_led(pdev);

	led_classdev_resume(&led->cdev);
	return 0;
}
#else
#define omap_pwm_led_suspend NULL
#define omap_pwm_led_resume NULL
#endif

static struct platform_driver omap_pwm_led_driver = {
	.probe		= omap_pwm_led_probe,
	.remove		= omap_pwm_led_remove,
	.suspend	= omap_pwm_led_suspend,
	.resume		= omap_pwm_led_resume,
	.driver		= {
		.name		= "omap_pwm_led",
		.owner		= THIS_MODULE,
	},
};

static int __init omap_pwm_led_init(void)
{
	return platform_driver_register(&omap_pwm_led_driver);
}

static void __exit omap_pwm_led_exit(void)
{
	platform_driver_unregister(&omap_pwm_led_driver);
}

module_init(omap_pwm_led_init);
module_exit(omap_pwm_led_exit);

MODULE_AUTHOR("Timo Teras");
MODULE_DESCRIPTION("OMAP PWM LED driver");
MODULE_LICENSE("GPL");
