#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include <stdlib.h>

#include "fjalar.h"
#include "melodies.h"

LOG_MODULE_REGISTER(actuation, CONFIG_APP_ACTUATION_LOG_LEVEL);

#define LED_THREAD_PRIORITY 7
#define LED_THREAD_STACK_SIZE 2048

#define BUZZER_THREAD_PRIORITY 7
#define BUZZER_THREAD_STACK_SIZE 2048

#define PYRO_THREAD_PRIORITY 7
#define PYRO_THREAD_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
struct k_thread led_thread_data;
k_tid_t led_thread_id;
void led_thread(fjalar_t *fjalar, void *, void *);

K_THREAD_STACK_DEFINE(buzzer_thread_stack, BUZZER_THREAD_STACK_SIZE);
struct k_thread buzzer_thread_data;
k_tid_t buzzer_thread_id;
void buzzer_thread(fjalar_t *fjalar, void *, void *);

K_THREAD_STACK_DEFINE(pyro_thread_stack, PYRO_THREAD_STACK_SIZE);
struct k_thread pyro_thread_data;
k_tid_t pyro_thread_id;
void pyro_thread(fjalar_t *fjalar, void *, void *);

volatile bool terminate_actuation =false;

void init_actuation(fjalar_t *fjalar) {
    terminate_actuation = false;
    led_thread_id = k_thread_create(
		&led_thread_data,
		led_thread_stack,
		K_THREAD_STACK_SIZEOF(led_thread_stack),
		(k_thread_entry_t) led_thread,
		fjalar, NULL, NULL,
		LED_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(led_thread_id, "led");

    pyro_thread_id = k_thread_create(
		&pyro_thread_data,
		pyro_thread_stack,
		K_THREAD_STACK_SIZEOF(pyro_thread_stack),
		(k_thread_entry_t) pyro_thread,
		fjalar, NULL, NULL,
		PYRO_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(pyro_thread_id, "pyro");

	#if DT_ALIAS_EXISTS(buzzer)
    buzzer_thread_id = k_thread_create(
		&buzzer_thread_data,
		buzzer_thread_stack,
		K_THREAD_STACK_SIZEOF(buzzer_thread_stack),
		(k_thread_entry_t) buzzer_thread,
		fjalar, NULL, NULL,
		BUZZER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(buzzer_thread_id, "buzzer");
	#endif
}

int deinit_actuation() {
    terminate_actuation = true;
    int e;
    e = k_thread_join(&led_thread_data, K_MSEC(1000));
    e |= k_thread_join(&pyro_thread_data, K_MSEC(1000));
    e |= k_thread_join(&buzzer_thread_data, K_MSEC(1000));
}

void led_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct gpio_dt_spec io_led_dt = GPIO_DT_SPEC_GET(DT_ALIAS(io_led), gpios);
    int ret = 0;
    ret |= gpio_pin_configure_dt(&io_led_dt, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        LOG_ERR("Could not configure the IO-led");
    }

    while (true) {
        gpio_pin_toggle_dt(&io_led_dt);
        k_msleep(1000);
    }
}

void set_pyro(fjalar_t *fjalar, int pyro, bool state) {
    switch (pyro) {
        case 1:
            const struct gpio_dt_spec pyro1_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1), gpios);
            gpio_pin_set_dt(&pyro1_dt, state);
            break;

        case 2:
            const struct gpio_dt_spec pyro2_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2), gpios);
            gpio_pin_set_dt(&pyro2_dt, state);
            break;

        case 3:
            const struct gpio_dt_spec pyro3_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3), gpios);
            gpio_pin_set_dt(&pyro3_dt, state);
            break;
        default:
            LOG_ERR("tried to enable invalid pyro %d", pyro);
    }
}

void pyro_thread(fjalar_t *fjalar, void *p2, void *p3) {
    const struct gpio_dt_spec pyro1_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1), gpios);
    const struct gpio_dt_spec pyro2_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2), gpios);
    const struct gpio_dt_spec pyro3_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3), gpios);
    const struct gpio_dt_spec pyro1_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1_sense), gpios);
    const struct gpio_dt_spec pyro2_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2_sense), gpios);
    const struct gpio_dt_spec pyro3_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3_sense), gpios);

    int ret = 0;
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);
    ret |= gpio_pin_set_dt(&pyro3_dt, 0);
    ret |= gpio_pin_configure_dt(&pyro1_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&pyro2_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&pyro3_dt, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);
    ret |= gpio_pin_set_dt(&pyro3_dt, 0);

    ret |= gpio_pin_configure_dt(&pyro1_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro2_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro3_sense_dt, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Could not configure pyro pins");
    }


    while (true) {
        fjalar->pyro1_sense = gpio_pin_get_dt(&pyro1_sense_dt);
        fjalar->pyro2_sense = gpio_pin_get_dt(&pyro2_sense_dt);
        fjalar->pyro3_sense = gpio_pin_get_dt(&pyro3_sense_dt);
        LOG_INF("pyros connected p1:%d p2:%d p3:%d", fjalar->pyro1_sense, fjalar->pyro2_sense, fjalar->pyro3_sense);
        k_msleep(500);
    }
}


void play_song(const struct pwm_dt_spec *buzzer_dt, int melody[], int size, int tempo) {
        int notes=size/sizeof(melody[0])/2;
        int wholenote = (60000 * 4) / tempo;

        int divider = 0, noteDuration = 0;

        for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
            divider = melody[thisNote + 1];
            if (divider > 0) {
            noteDuration = (wholenote) / divider;
            } else if (divider < 0) {
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5; // increases the duration in half for dotted notes
            }

            int period = 1e9 / melody[thisNote];
            pwm_set_dt(buzzer_dt, period, period / 2);
            k_msleep(noteDuration * 0.9);
            pwm_set_dt(buzzer_dt, period, 0);
            k_msleep(noteDuration * 0.1);
        }
        k_msleep(1000);
}

#if DT_ALIAS_EXISTS(buzzer)
void buzzer_thread(fjalar_t *fjalar, void *p2, void *p3) {
    #if !CONFIG_BUZZER_ENABLED
    LOG_INF("Buzzer disabled");
    return;
    #endif
    int err;
    const struct pwm_dt_spec buzzer_dt = PWM_DT_SPEC_GET(DT_ALIAS(buzzer));
    if (!device_is_ready(buzzer_dt.dev)) {
        LOG_ERR("buzzer is not ready");
        return;
    }

    uint32_t center_period = 1e9 / 2000;
    err = pwm_set_dt(&buzzer_dt, center_period, 0);
    if (err) {
        LOG_ERR("Could not set buzzer");
    }

    while (true) {
        if (fjalar->sudo) {
                play_song(&buzzer_dt, doom_melody, sizeof(doom_melody), doom_tempo);
                k_msleep(1000);
                continue;
        }
        switch (fjalar->flight_state) {
            case STATE_LANDED:
                play_song(&buzzer_dt, nokia_melody, sizeof(nokia_melody), nokia_tempo);
                k_msleep(1000);
                break;

            case STATE_IDLE:
                play_song(&buzzer_dt, greensleeves_melody, sizeof(greensleeves_melody), greensleeves_tempo);
                k_msleep(1000);
                break;

            case STATE_LAUNCHPAD:
                play_song(&buzzer_dt, keyboardcat_melody, sizeof(keyboardcat_melody), keyboardcat_tempo);
                k_msleep(1000);
                break;

            default:
                err = pwm_set_dt(&buzzer_dt, center_period, center_period / 2);
                k_msleep(100);
                err = pwm_set_dt(&buzzer_dt, center_period, 0);
                k_msleep(100);
                err = pwm_set_dt(&buzzer_dt, center_period, center_period / 2);
                k_msleep(100);
                err = pwm_set_dt(&buzzer_dt, center_period, 0);
                k_msleep(1500);
        }
    }
}
#endif