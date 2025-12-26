#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>

#include <zephyr/drivers/mfd/npm2100.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/drivers/i2c.h>

#include <stdlib.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/timeutil.h>

#define DEEP_SLEEP_TIME_S 5
#define MEASURE_TIME_S 10
#define MAX_SCANNED_DEVICES 10

LOG_MODULE_REGISTER(factory_test, LOG_LEVEL_INF);

enum factory_mode {
	FACTORY_MODE_WORK = 0,
	FACTORY_MODE_TEST = 1,
};

static atomic_t g_mode = ATOMIC_INIT(FACTORY_MODE_WORK);
static atomic_t g_bt_scan_running;
static atomic_t g_bt_adv_running;
static atomic_t g_nfc_running;
static atomic_t g_nfc_active;

struct scanned_device {
	bt_addr_le_t addr;
	int8_t rssi;
	uint8_t type;
};

static struct scanned_device devices[MAX_SCANNED_DEVICES];
static int device_count = 0;
static int device_index = 0;
static struct k_timer scan_print_timer;

static const struct device *pmic = DEVICE_DT_GET(DT_NODELABEL(npm2100_pmic));
static const struct device *regulators = DEVICE_DT_GET(DT_NODELABEL(npm2100_regulators));
static bool shipmode;
static bool system_off_flag;
static const struct device *i2c_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(npm2100_pmic)));
static uint8_t pmic_addr = DT_REG_ADDR(DT_NODELABEL(npm2100_pmic));

static const struct device *const sys_clock = DEVICE_DT_GET_ONE(nordic_nrf_clock);
static const struct device *const timer = DEVICE_DT_GET(DT_NODELABEL(timer21));
static bool lfxo_initialized;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec reset_btn = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback reset_btn_cb;

static const struct gpio_dt_spec pmic_int_pin = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(npm2100_pmic), host_int_gpios, 0);
static struct gpio_callback pmic_int_cb;
static struct k_work pmic_int_work;

static void reset_btn_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	LOG_WRN("Reset button pressed -> reboot");
	sys_reboot(SYS_REBOOT_COLD);
}

static void pmic_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	k_work_submit(&pmic_int_work);
}

static void pmic_int_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	// Read EVENTS_GPIO_SET (0x02)
	uint8_t events;
	int ret = i2c_reg_read_byte(i2c_dev, pmic_addr, 0x02, &events);
	if (ret) {
		LOG_ERR("Failed to read EVENTS_GPIO_SET: %d", ret);
		return;
	}

	if (events & 0x01) { // GPIO0FALL
		LOG_INF("PMIC GPIO0 falling edge detected");
	}
	if (events & 0x02) { // GPIO0RISE
		LOG_INF("PMIC GPIO0 rising edge detected");
	}

	// Clear events by writing to EVENTS_GPIO_CLR (0x07)
	ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x07, events & 0x03);
	if (ret) {
		LOG_ERR("Failed to clear events: %d", ret);
	}
}

static int reset_button_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&reset_btn)) {
		LOG_ERR("Reset button GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&reset_btn, GPIO_INPUT);
	if (err) {
		LOG_ERR("Reset button configure failed: %d", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&reset_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("Reset button interrupt config failed: %d", err);
		return err;
	}

	gpio_init_callback(&reset_btn_cb, reset_btn_isr, BIT(reset_btn.pin));
	err = gpio_add_callback(reset_btn.port, &reset_btn_cb);
	if (err) {
		LOG_ERR("Reset button add callback failed: %d", err);
		return err;
	}

	return 0;
}

static int pmic_int_init(void)
{
	int err;

	k_work_init(&pmic_int_work, pmic_int_work_handler);

	if (!gpio_is_ready_dt(&pmic_int_pin)) {
		LOG_ERR("PMIC INT GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&pmic_int_pin, GPIO_INPUT);
	if (err) {
		LOG_ERR("PMIC INT GPIO configure failed: %d", err);
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&pmic_int_pin, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("PMIC INT GPIO interrupt config failed: %d", err);
		return err;
	}

	gpio_init_callback(&pmic_int_cb, pmic_int_isr, BIT(pmic_int_pin.pin));
	err = gpio_add_callback(pmic_int_pin.port, &pmic_int_cb);
	if (err) {
		LOG_ERR("PMIC INT GPIO add callback failed: %d", err);
		return err;
	}

	return 0;
}

static int led_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		LOG_WRN("LED not ready; skipping LED blink");
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
}

static void led_set(bool on)
{
	if (!gpio_is_ready_dt(&led)) {
		return;
	}
	(void)gpio_pin_set_dt(&led, on ? 1 : 0);
}

static void led_toggle(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return;
	}
	(void)gpio_pin_toggle_dt(&led);
}

static void led_worker_thread(void)
{
	while (true) {
		if (atomic_get(&g_mode) == FACTORY_MODE_WORK) {
			led_toggle();
			k_sleep(K_MSEC(500));
		} else {
			if (!atomic_get(&g_nfc_active)) {
				led_set(true);
			}
			k_sleep(K_MSEC(200));
		}
	}
}

K_THREAD_DEFINE(led_thread_id, 768, led_worker_thread, NULL, NULL, NULL, 5, 0, 0);

static void scan_print_timer_handler(struct k_timer *timer)
{
	if (device_count == 0) {
		return;
	}

	printk("Scanned devices (last %d):\n", device_count);
	int start = (device_index - device_count + MAX_SCANNED_DEVICES) % MAX_SCANNED_DEVICES;
	for (int i = 0; i < device_count; i++) {
		int idx = (start + i) % MAX_SCANNED_DEVICES;
		char addr_str[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(&devices[idx].addr, addr_str, sizeof(addr_str));
		printk("  %s RSSI %d dBm type 0x%02x\n", addr_str, devices[idx].rssi, devices[idx].type);
	}
	// Clear the list after printing
	device_count = 0;
	device_index = 0;
}

static void bt_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *ad)
{
	ARG_UNUSED(ad);

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		return;
	}

	devices[device_index].addr = *addr;
	devices[device_index].rssi = rssi;
	devices[device_index].type = type;
	device_index = (device_index + 1) % MAX_SCANNED_DEVICES;
	if (device_count < MAX_SCANNED_DEVICES) {
		device_count++;
	}
}

static int bt_stack_ensure_ready(void)
{
	static bool inited;
	static int init_err;

	if (inited) {
		return init_err;
	}

	init_err = bt_enable(NULL);
	if (init_err) {
		LOG_ERR("Bluetooth init failed: %d", init_err);
	} else {
		LOG_INF("Bluetooth initialized");
	}
	inited = true;
	return init_err;
}

static int bt_scan_start(void)
{
	int err = bt_stack_ensure_ready();
	if (err) {
		return err;
	}

	if (atomic_get(&g_bt_scan_running)) {
		return 0;
	}

	struct bt_le_scan_param scan_param = {
		.type = BT_HCI_LE_SCAN_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, bt_device_found);
	if (!err) {
		atomic_set(&g_bt_scan_running, 1);
		device_count = 0;
		device_index = 0;
		k_timer_init(&scan_print_timer, scan_print_timer_handler, NULL);
		k_timer_start(&scan_print_timer, K_SECONDS(5), K_SECONDS(5));
		LOG_INF("BT scan started");
	}
	return err;
}

static int bt_scan_stop(void)
{
	if (!atomic_get(&g_bt_scan_running)) {
		return 0;
	}

	int err = bt_le_scan_stop();
	if (!err) {
		atomic_set(&g_bt_scan_running, 0);
		k_timer_stop(&scan_print_timer);
		device_count = 0;
		device_index = 0;
		LOG_INF("BT scan stopped");
	}
	return err;
}

static const struct bt_data adv_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int bt_adv_start(void)
{
	int err = bt_stack_ensure_ready();
	if (err) {
		return err;
	}

	if (atomic_get(&g_bt_adv_running)) {
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
	if (!err) {
		atomic_set(&g_bt_adv_running, 1);
		LOG_INF("BT advertising started");
	}

	return err;
}

static int bt_adv_stop(void)
{
	if (!atomic_get(&g_bt_adv_running)) {
		return 0;
	}

	int err = bt_le_adv_stop();
	if (!err) {
		atomic_set(&g_bt_adv_running, 0);
		LOG_INF("BT advertising stopped");
	}
	return err;
}

#define NDEF_MSG_BUF_SIZE 128
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t len)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		if (atomic_get(&g_nfc_active)) {
			led_set(true);
		}
		printk("NFC field on\n");
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		if (atomic_get(&g_nfc_active)) {
			led_set(false);
		}
		printk("NFC field off\n");
		break;
	default:
		break;
	}
}

static int nfc_start(void)
{
	if (atomic_get(&g_nfc_running)) {
		return 0;
	}

	int err = nfc_t2t_setup(nfc_callback, NULL);
	if (err) {
		LOG_ERR("NFC setup failed: %d", err);
		return err;
	}

	static const uint8_t payload[] = { 'F','A','C','T','O','R','Y','_','T','E','S','T' };
	static const uint8_t lang[] = { 'e', 'n' };

	NFC_NDEF_TEXT_RECORD_DESC_DEF(text_rec, UTF_8, lang, sizeof(lang), payload, sizeof(payload));
	NFC_NDEF_MSG_DEF(msg, 1);

	err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(msg), &NFC_NDEF_TEXT_RECORD_DESC(text_rec));
	if (err) {
		LOG_ERR("NFC record add failed: %d", err);
		return err;
	}

	uint32_t ndef_len = sizeof(ndef_msg_buf);
	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(msg), ndef_msg_buf, &ndef_len);
	if (err) {
		LOG_ERR("NFC msg encode failed: %d", err);
		return err;
	}

	err = nfc_t2t_payload_set(ndef_msg_buf, ndef_len);
	if (err) {
		LOG_ERR("NFC payload set failed: %d", err);
		return err;
	}

	err = nfc_t2t_emulation_start();
	if (err) {
		LOG_ERR("NFC emulation start failed: %d", err);
		return err;
	}

	atomic_set(&g_nfc_running, 1);
	// Flash LED once and turn off
	led_toggle();
	k_sleep(K_MSEC(100));
	led_set(false);
	atomic_set(&g_nfc_active, 1);
	LOG_INF("NFC emulation started");
	return 0;
}

static int nfc_stop(void)
{
	if (!atomic_get(&g_nfc_running)) {
		return 0;
	}

	int err = nfc_t2t_emulation_stop();
	if (!err) {
		atomic_set(&g_nfc_running, 0);
		atomic_set(&g_nfc_active, 0);
		led_set(true);  // Restore LED to on in TEST mode
		LOG_INF("NFC emulation stopped");
	}
	return err;
}

static int cmd_lfxo_test(const struct shell *sh, size_t argc, char **argv)
{
	int ret;
	uint32_t top;
	bool measured = false;
	struct timeutil_sync_config sync_config = { 0};
	struct timeutil_sync_state sync_state = { 0 };
	uint64_t counter_ref = 0U;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!lfxo_initialized) {
		shell_error(sh, "LFXO module not initialized");
		return -EPERM;
	}

	ret = clock_control_on(sys_clock, CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (ret < 0) {
		shell_error(sh, "Failed to start HF clock: %d\n", ret);
		return ret;
	}

	sync_config.ref_Hz = counter_get_frequency(timer);
	if (sync_config.ref_Hz == 0) {
		shell_error(sh, "Timer has no fixed frequency");
		return -ENOTSUP;
	}

	top = counter_get_top_value(timer);
	if (top != UINT32_MAX) {
		shell_error(sh, "Timer wraps at %u not at 32 bits", top);
		return -ENOTSUP;
	}

	ret = counter_start(timer);
	if (ret < 0) {
		shell_error(sh, "Failed to start timer: %d\n", ret);
		return ret;
	}

	sync_config.local_Hz = CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	sync_state.cfg = &sync_config;

	shell_print(sh, "Checking timer at %u Hz against ticks at %u Hz\n", sync_config.ref_Hz,
		    sync_config.local_Hz);

	while (!measured) {
		uint32_t ctr;
		float skew;
		struct timeutil_sync_instant inst;

		ret = counter_get_value(timer, &ctr);
		if (ret < 0) {
			shell_error(sh, "Failed to get timer value: %d\n", ret);
			return ret;
		}

		counter_ref += ctr - (uint32_t)counter_ref;
		inst.ref = counter_ref;
		inst.local = k_uptime_ticks();

		ret = timeutil_sync_state_update(&sync_state, &inst);
		if (ret < 0) {
			shell_error(sh, "Sync update error: %d\n", ret);
			return ret;
		}

		if (ret == 0) {
			k_sleep(K_SECONDS(MEASURE_TIME_S));
			continue;
		}

		skew = timeutil_sync_estimate_skew(&sync_state);
		shell_print(sh, "Skew %f ; err %d ppb", (double)skew,
			    timeutil_sync_skew_to_ppb(skew));
		measured = true;
	}

	ret = counter_stop(timer);
	if (ret < 0) {
		shell_error(sh, "Failed to stop timer: %d\n", ret);
		return ret;
	}

	ret = clock_control_off(sys_clock, CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (ret < 0) {
		shell_error(sh, "Failed to stop HF clock: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lfxo_init(void)
{
	if (!device_is_ready(sys_clock) || !device_is_ready(timer)) {
		return -ENODEV;
	}

	lfxo_initialized = true;

	return 0;
}

static int cmd_factory_mode(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (!strcmp(argv[1], "work")) {
		(void)bt_scan_stop();
		(void)bt_adv_stop();
		(void)nfc_stop();
		atomic_set(&g_mode, FACTORY_MODE_WORK);
		shell_print(shell, "Mode: WORK (LED blink)");
		return 0;
	}
	if (!strcmp(argv[1], "test")) {
		atomic_set(&g_mode, FACTORY_MODE_TEST);
		shell_print(shell, "Mode: TEST");
		return 0;
	}

	shell_error(shell, "Usage: factory mode <work|test>");
	return -EINVAL;
}

static int cmd_factory_bt_scan(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	int err;
	if (!strcmp(argv[1], "start")) {
		err = bt_scan_start();
		shell_print(shell, "bt scan start: %d", err);
		return err;
	}
	if (!strcmp(argv[1], "stop")) {
		err = bt_scan_stop();
		shell_print(shell, "bt scan stop: %d", err);
		return err;
	}

	shell_error(shell, "Usage: factory bt_scan <start|stop>");
	return -EINVAL;
}

static int cmd_factory_bt_adv(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	int err;
	if (!strcmp(argv[1], "start")) {
		err = bt_adv_start();
		shell_print(shell, "bt adv start: %d", err);
		return err;
	}
	if (!strcmp(argv[1], "stop")) {
		err = bt_adv_stop();
		shell_print(shell, "bt adv stop: %d", err);
		return err;
	}

	shell_error(shell, "Usage: factory bt_adv <start|stop>");
	return -EINVAL;
}

static int cmd_factory_nfc(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	int err;
	if (!strcmp(argv[1], "start")) {
		err = nfc_start();
		shell_print(shell, "nfc start: %d", err);
		return err;
	}
	if (!strcmp(argv[1], "stop")) {
		err = nfc_stop();
		shell_print(shell, "nfc stop: %d", err);
		return err;
	}

	shell_error(shell, "Usage: factory nfc <start|stop>");
	return -EINVAL;
}

static int cmd_factory_shipmode(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(pmic)) {
		shell_error(shell, "PMIC not ready");
		return -ENODEV;
	}

	if (!device_is_ready(regulators)) {
		shell_error(shell, "Regulators not ready");
		return -ENODEV;
	}

	shipmode = true;
	shell_print(shell, "Entering shipping mode...");
	return 0;
}

static int cmd_factory_system_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	system_off_flag = true;
	shell_print(shell, "Entering system off...");
	return 0;
}

static int cmd_factory_pmic_gpio0(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: factory pmic_gpio0 <high|low>");
		return -EINVAL;
	}

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	if (!device_is_ready(pmic)) {
		shell_error(shell, "PMIC not ready");
		return -ENODEV;
	}

	if (!device_is_ready(i2c_dev)) {
		shell_error(shell, "I2C device not ready");
		return -ENODEV;
	}

	// Set USAGE0 SEL=0 for GPIO mode
	int ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x83, 0x00);
	if (ret) {
		shell_error(shell, "Failed to set USAGE0: %d", ret);
		return ret;
	}

	// Set CONFIG0 OUTPUT=1 to enable output buffer
	ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x80, 0x02);
	if (ret) {
		shell_error(shell, "Failed to set CONFIG0: %d", ret);
		return ret;
	}

	// Set OUTPUT0 VALUE based on argument
	uint8_t value = strcmp(argv[1], "high") == 0 ? 1 : 0;
	ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x86, value);
	if (ret) {
		shell_error(shell, "Failed to set OUTPUT0: %d", ret);
		return ret;
	}

	shell_print(shell, "PMIC GPIO0 set to %s", argv[1]);
	return 0;
}

static int cmd_factory_pmic_gpio_int_test(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 1) {
		shell_error(shell, "Usage: factory pmic_gpio_int_test");
		return -EINVAL;
	}

	if (atomic_get(&g_mode) != FACTORY_MODE_TEST) {
		shell_error(shell, "Not in TEST mode");
		return -EPERM;
	}

	if (!device_is_ready(pmic)) {
		shell_error(shell, "PMIC not ready");
		return -ENODEV;
	}

	if (!device_is_ready(i2c_dev)) {
		shell_error(shell, "I2C device not ready");
		return -ENODEV;
	}

	// Step 1: Set USAGE0 SEL=0 for GPIO mode
	int ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x83, 0x00);
	if (ret) {
		shell_error(shell, "Failed to set USAGE0: %d", ret);
		return ret;
	}

	// Step 2: Set CONFIG0 INPUT=1, enable pull-up and debounce
	// Bit 0: INPUT=1, Bit 4: PULLUP=1, Bit 6: DEBOUNCE=1
	ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x80, 0x51);
	if (ret) {
		shell_error(shell, "Failed to set CONFIG0: %d", ret);
		return ret;
	}

	// Step 3: Enable interrupts for both edges on GPIO0
	// GPIO0FALL (Bit 0) and GPIO0RISE (Bit 1)
	ret = i2c_reg_write_byte(i2c_dev, pmic_addr, 0x0C, 0x03);
	if (ret) {
		shell_error(shell, "Failed to set INTEN_GPIO_SET: %d", ret);
		return ret;
	}

	shell_print(shell, "PMIC GPIO0 interrupt test configured. Toggle GPIO0 to trigger interrupts.");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(factory_cmds,
	SHELL_CMD_ARG(mode, NULL, "Switch mode: work|test", cmd_factory_mode, 2, 0),
	SHELL_CMD_ARG(bt_scan, NULL, "Bluetooth scan: start|stop", cmd_factory_bt_scan, 2, 0),
	SHELL_CMD_ARG(bt_adv, NULL, "Bluetooth advertising: start|stop", cmd_factory_bt_adv, 2, 0),
	SHELL_CMD_ARG(nfc, NULL, "NFC emulation: start|stop", cmd_factory_nfc, 2, 0),
	SHELL_CMD_ARG(shipmode, NULL, "Enter shipping mode", cmd_factory_shipmode, 1, 0),
	SHELL_CMD_ARG(system_off, NULL, "Enter system off mode", cmd_factory_system_off, 1, 0),
	SHELL_CMD_ARG(pmic_gpio0, NULL, "Control PMIC GPIO0: high|low", cmd_factory_pmic_gpio0, 2, 0),
	SHELL_CMD(pmic_gpio_int_test, NULL, "Test PMIC GPIO0 interrupt", cmd_factory_pmic_gpio_int_test),
	SHELL_CMD(lfxo_test, NULL, "Test LFXO accuracy", cmd_lfxo_test),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(factory, &factory_cmds, "Factory test commands", NULL);

int main(void)
{
	int err;

	printk("Factory test firmware start\n");
	printk("Default mode: WORK (LED blink)\n");
	printk("Type 'factory mode test' to enter TEST mode\n");

	err = led_init();
	if (err && err != -ENODEV) {
		LOG_ERR("LED init failed: %d", err);
	}

	err = reset_button_init();
	if (err) {
		LOG_ERR("Reset button init failed: %d", err);
	}

	err = pmic_int_init();
	if (err) {
		LOG_ERR("PMIC INT init failed: %d", err);
	}

	err = lfxo_init();
	if (err && err != -ENODEV) {
		LOG_ERR("LFXO init failed: %d", err);
	}

	while (1) {
		if (shipmode) {
			shipmode = false;
			LOG_INF("Ship mode...");
			int ret = regulator_parent_ship_mode(regulators);
			if (ret < 0) {
				LOG_ERR("Failed to enter ship mode: %d", ret);
			}
		}
		if (system_off_flag) {
			system_off_flag = false;
			led_set(false);  // Turn off LED before system off
			int err = z_nrf_grtc_wakeup_prepare(DEEP_SLEEP_TIME_S * USEC_PER_SEC);
			if (err < 0) {
				LOG_ERR("Unable to prepare GRTC as a wake up source (err = %d)", err);
			} else {
				LOG_INF("System off; wait %u seconds to restart", DEEP_SLEEP_TIME_S);
				sys_poweroff();
			}
		}
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
