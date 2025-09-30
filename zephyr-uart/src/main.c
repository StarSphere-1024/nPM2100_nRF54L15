/*
 * Copyright (c) 2023 Seeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/* Get the device from the device tree */
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(xiao_serial));

int main(void)
{
	uint8_t c;

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	printk("UART echo example started\n");

	while (1) {
		/* Poll for a character */
		if (uart_poll_in(uart_dev, &c) == 0) {
			/* Echo the character back */
			uart_poll_out(uart_dev, c);
		}
	}
	return 0;
}
