/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/mfd/npm2100.h>
#include <zephyr/drivers/regulator.h>

#define FLASH_FAST_MS	100
#define FLASH_SLOW_MS	500
#define PRESS_SHORT_MS	1000
#define PRESS_MEDIUM_MS 2000

static int flash_time_ms = FLASH_SLOW_MS;
static bool shipmode;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm2100_pmic));
static const struct device *regulators = DEVICE_DT_GET(DT_NODELABEL(npm2100_regulators));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("User button pressed. Rebooting...");
	// Short delay to allow log to flush.
	k_msleep(100);
	sys_reboot(SYS_REBOOT_WARM);
}

static void event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t events)
{
	static int64_t press_t;

	if (events & BIT(NPM2100_EVENT_SYS_SHIPHOLD_FALL)) {
		press_t = k_uptime_get();
	}

	if (events & BIT(NPM2100_EVENT_SYS_SHIPHOLD_RISE)) {
		int64_t delta_t = k_uptime_get() - press_t;

		if (delta_t < PRESS_SHORT_MS) {
			LOG_INF("Short press");
			flash_time_ms = FLASH_FAST_MS;
		} else if (delta_t < PRESS_MEDIUM_MS) {
			LOG_INF("Medium press");
			flash_time_ms = FLASH_SLOW_MS;
		} else {
			// shipmode = true;
		}
	}
}

static bool configure_events(void)
{
	int ret;
	static struct gpio_callback event_cb;

	if (!device_is_ready(pmic)) {
		LOG_ERR("Error: PMIC device not ready.");
		return false;
	}

	if (!device_is_ready(regulators)) {
		LOG_ERR("Error: Regulator device not ready.");
		return false;
	}

	gpio_init_callback(&event_cb, event_callback,
			   BIT(NPM2100_EVENT_SYS_SHIPHOLD_FALL) |
				   BIT(NPM2100_EVENT_SYS_SHIPHOLD_RISE));

	ret = mfd_npm2100_add_callback(pmic, &event_cb);

	if (ret < 0) {
		LOG_ERR("Error: failed to add a PMIC event callback.");
		return false;
	}

	return true;
}

static bool configure_ui(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("Error: led device %s is not ready.", led.port->name);
		return false;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	if (ret < 0) {
		LOG_ERR("Error: failed to configure led device %s.", led.port->name);
		return false;
	}

	LOG_INF("Set up led at %s pin %d", led.port->name, led.pin);

	return true;
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);


	if (!configure_ui()) {
		return 0;
	}

	if (!configure_events()) {
		return 0;
	}

	LOG_INF("PMIC device ok");

	while (1) {
		if (shipmode) {
			shipmode = false;
			LOG_INF("Ship mode...");

			ret = regulator_parent_ship_mode(regulators);
			if (ret < 0) {
				LOG_ERR("Error: failed to enter ship mode. ret=%d", ret);
			}
		}
		(void)gpio_pin_toggle_dt(&led);
		k_msleep(flash_time_ms);
	}
}
