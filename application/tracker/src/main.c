#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/pm/pm.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/display/cfb.h>
#include <zephyr/sys/poweroff.h>

#include <cfb_custom_font.h>

#include "tracker.h"
#include "display.h"
#include "sensors.h"
#include "communication.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_MAIN_LOG_LEVEL);

tracker_t tracker_god = {0};

struct gpio_callback button_callback_data;

void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    const struct gpio_dt_spec button_sw = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
    static uint32_t pressed_at = 0;

    bool state = gpio_pin_get_dt(&button_sw);
    // LOG_DBG("button pressed at %d %d", k_uptime_get_32(), state);

    // driver doesn't debounce
    if (k_uptime_get_32() - pressed_at < 50) {
        return;
    }

    if (state == true) {
        pressed_at = k_uptime_get_32();
    }

    if (state == false) {
        uint32_t delta = k_uptime_get_32() - pressed_at;
        if (delta > 1000) {
            send_screen_command(&tracker_god);
        } else  {
            next_frame(&tracker_god);
        }
    }
}

int main(void)
{
    #ifdef CONFIG_DELAYED_START
	const int delay = 10;
	for (int i = 0; i < delay; i++) {
		printk("%d\n", delay - i);
		k_msleep(1000);
	}
	#endif
    init_display(&tracker_god);
    init_communication(&tracker_god);
    init_sensors(&tracker_god);

    int ret;
    const struct gpio_dt_spec gpio_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    const struct gpio_dt_spec back_led = GPIO_DT_SPEC_GET(DT_ALIAS(led_back), gpios);
    const struct gpio_dt_spec button_sw = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

    ret = gpio_pin_configure_dt(&gpio_led, GPIO_OUTPUT_ACTIVE);

    ret = gpio_pin_configure_dt(&back_led, GPIO_OUTPUT_ACTIVE);
    ret = gpio_pin_set_dt(&back_led, 1);

    ret = gpio_pin_configure_dt(&button_sw, GPIO_INPUT);
    ret = gpio_pin_interrupt_configure_dt(&button_sw, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&button_callback_data, button_callback, BIT(button_sw.pin));
    ret = gpio_add_callback_dt(&button_sw, &button_callback_data);

    while (1) {
        ret = gpio_pin_toggle_dt(&gpio_led);
        if (ret < 0) {
            return 0;
        }

        k_msleep(100);
    }
    return 0;
}

#ifdef CONFIG_BOARD_T_ECHO
#include <hal/nrf_gpio.h>

void power_thread(void *p1, void *p2, void *p3);

K_THREAD_DEFINE(power_thread_id, 512, power_thread, NULL, NULL, NULL, 10, 0, 0);

void power_off(const struct gpio_dt_spec *gpio_pwr_en) {
    gpio_pin_set_dt(gpio_pwr_en, 0);
    sys_poweroff();
    k_msleep(1000);
    // the line above kind of powers off the board.
    // If it somehow doesn't then enable external power again to notify the user
    gpio_pin_set_dt(gpio_pwr_en, 1);
}

void power_thread(void *p1, void *p2, void *p3) {
    const struct gpio_dt_spec gpio_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
    const struct gpio_dt_spec gpio_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(pwr_en), gpios);
    const struct gpio_dt_spec gpio_led = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

    gpio_pin_configure_dt(&gpio_button, GPIO_INPUT);
    gpio_pin_configure_dt(&gpio_pwr_en, GPIO_OUTPUT);
    gpio_pin_configure_dt(&gpio_led, GPIO_OUTPUT);

    int pressed_for = 0;
    while (true) {
        if (gpio_pin_get_dt(&gpio_button)) {
            pressed_for++;
            if (pressed_for > 100) {
                power_off(&gpio_pwr_en);
            }
        } else {
            pressed_for = 0;
        }
        k_msleep(100);
    }
}

int early_power_on() {
    uint32_t psel = NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(pwr_en), gpios);
    gpio_dt_flags_t flags = DT_GPIO_FLAGS(DT_ALIAS(pwr_en), gpios);

    if (flags & GPIO_ACTIVE_LOW) {
        nrf_gpio_pin_clear(psel);
    } else {
        nrf_gpio_pin_set(psel);
    }
    nrf_gpio_cfg_output(psel);

    psel = NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(led2), gpios);
    flags = DT_GPIO_FLAGS(DT_ALIAS(led2), gpios);

    if (flags & GPIO_ACTIVE_LOW) {
        nrf_gpio_pin_clear(psel);
    } else {
        nrf_gpio_pin_set(psel);
    }
    nrf_gpio_cfg_output(psel);

    return 0;
}
SYS_INIT(early_power_on, PRE_KERNEL_1, 0);
#endif