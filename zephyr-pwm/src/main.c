#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

// Register log module
LOG_MODULE_REGISTER(pwm_shell_example, CONFIG_LOG_DEFAULT_LEVEL);

// Define default PWM period as 1 millisecond (1,000,000 nanoseconds)
// This corresponds to a 1 kHz PWM frequency
static uint32_t pwm_period_ns = 1000000UL; // Use UL to ensure unsigned long
static uint32_t pwm_duty_ns = 500000UL;   // Default to 50% duty cycle

// Get the PWM LED device defined in the device tree
static const struct pwm_dt_spec led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led));

static int cmd_set_freq(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(sh, "Usage: set_freq <frequency_hz>");
        return -EINVAL;
    }

    uint32_t freq_hz = atoi(argv[1]);
    if (freq_hz == 0) {
        shell_print(sh, "Invalid frequency value.");
        return -EINVAL;
    }

    pwm_period_ns = 1000000000UL / freq_hz;
    int ret = pwm_set_dt(&led, pwm_period_ns, pwm_duty_ns);
    if (ret < 0) {
        shell_error(sh, "Error %d: failed to set PWM period", ret);
        return ret;
    }

    shell_print(sh, "Set PWM frequency to %u Hz (period: %u ns)", freq_hz, pwm_period_ns);
    return 0;
}

static int cmd_set_duty(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(sh, "Usage: set_duty <duty_cycle_percent>");
        return -EINVAL;
    }

    uint32_t duty_percent = atoi(argv[1]);
    if (duty_percent > 100) {
        shell_print(sh, "Duty cycle cannot be more than 100%%.");
        return -EINVAL;
    }

    pwm_duty_ns = (pwm_period_ns * duty_percent) / 100U;
    int ret = pwm_set_dt(&led, pwm_period_ns, pwm_duty_ns);
    if (ret < 0) {
        shell_error(sh, "Error %d: failed to set PWM duty cycle", ret);
        return ret;
    }

    shell_print(sh, "Set PWM duty cycle to %u%% (%u ns)", duty_percent, pwm_duty_ns);
    return 0;
}

SHELL_CMD_REGISTER(set_freq, NULL, "Set PWM frequency in Hz", cmd_set_freq);
SHELL_CMD_REGISTER(set_duty, NULL, "Set PWM duty cycle in percent (0-100)", cmd_set_duty);

int main(void)
{
    LOG_INF("Starting Zephyr PWM Shell example...");

    // Check if PWM device is ready
    if (!device_is_ready(led.dev)) {
        LOG_ERR("Error: PWM device %s is not ready", led.dev->name);
        return 0;
    }

    // Set initial PWM signal
    int ret = pwm_set_dt(&led, pwm_period_ns, pwm_duty_ns);
    if (ret < 0) {
        LOG_ERR("Error %d: failed to set initial PWM duty cycle", ret);
        return 0;
    }

    LOG_INF("Initial PWM frequency: 1kHz, duty cycle: 50%%");
    LOG_INF("Use 'set_freq' and 'set_duty' commands to control the PWM.");

    return 0;
}