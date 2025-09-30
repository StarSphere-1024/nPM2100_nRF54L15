#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/*
 * Get button configuration from the device tree.
 */
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("User button pressed. Rebooting...");
	// Short delay to allow log to flush.
	k_msleep(100);
	sys_reboot(SYS_REBOOT_WARM);
}

int main(void)
{
	int ret;

	LOG_INF("Board %s started. You can now use the shell to test peripherals.", CONFIG_BOARD);
	LOG_INF("Press the user button to trigger a software reset.");

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
	return 0;
}

/* Custom GPIO commands */
struct gpio_pin {
	const struct device *port;
	uint8_t pin_num;
};

static const struct gpio_pin pins_to_control[] = {
	{DEVICE_DT_GET(DT_NODELABEL(gpio0)), 1}, {DEVICE_DT_GET(DT_NODELABEL(gpio0)), 2},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 2}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 3}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 4}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 5},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 7}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 10},
	{DEVICE_DT_GET(DT_NODELABEL(gpio1)), 13}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 14},
	{DEVICE_DT_GET(DT_NODELABEL(gpio2)), 0}, {DEVICE_DT_GET(DT_NODELABEL(gpio2)), 1}, {DEVICE_DT_GET(DT_NODELABEL(gpio2)), 2}, {DEVICE_DT_GET(DT_NODELABEL(gpio2)), 3},
	{DEVICE_DT_GET(DT_NODELABEL(gpio2)), 4}, {DEVICE_DT_GET(DT_NODELABEL(gpio2)), 5},

	// {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 8}, {DEVICE_DT_GET(DT_NODELABEL(gpio1)), 9},TX RX
};

static int cmd_gpios_init(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 0; i < ARRAY_SIZE(pins_to_control); i++) {
		if (!device_is_ready(pins_to_control[i].port)) {
			shell_error(sh, "Error: GPIO port not ready.");
			return -ENODEV;
		}
		int ret = gpio_pin_configure(pins_to_control[i].port, pins_to_control[i].pin_num, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			shell_error(sh, "Error configuring pin %d on port", pins_to_control[i].pin_num);
			return ret;
		}
	}
	shell_print(sh, "All specified GPIOs configured as output.");
	return 0;
}

static int cmd_gpios_set(const struct shell *sh, size_t argc, char **argv, int level)
{
	for (size_t i = 0; i < ARRAY_SIZE(pins_to_control); i++) {
		if (!device_is_ready(pins_to_control[i].port)) {
			shell_error(sh, "Error: GPIO port not ready.");
			return -ENODEV;
		}
		gpio_pin_set(pins_to_control[i].port, pins_to_control[i].pin_num, level);
	}
	shell_print(sh, "All specified GPIOs set to %d.", level);
	return 0;
}

static int cmd_gpios_high(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_gpios_set(sh, argc, argv, 1);
}

static int cmd_gpios_low(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_gpios_set(sh, argc, argv, 0);
}

static int cmd_gpios_toggle(const struct shell *sh, size_t argc, char **argv)
{
	for (size_t i = 0; i < ARRAY_SIZE(pins_to_control); i++) {
		if (!device_is_ready(pins_to_control[i].port)) {
			shell_error(sh, "Error: GPIO port not ready.");
			return -ENODEV;
		}
		gpio_pin_toggle(pins_to_control[i].port, pins_to_control[i].pin_num);
	}
	shell_print(sh, "All specified GPIOs toggled.");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_gpios,
	SHELL_CMD(init, NULL, "Configure all GPIOs as output.", cmd_gpios_init),
	SHELL_CMD(high, NULL, "Set all GPIOs to high.", cmd_gpios_high),
	SHELL_CMD(low, NULL, "Set all GPIOs to low.", cmd_gpios_low),
	SHELL_CMD(toggle, NULL, "Toggle all GPIOs.", cmd_gpios_toggle),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gpios, &sub_gpios, "Bulk GPIO control commands", NULL);
