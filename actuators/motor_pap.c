#include "motor_pap.h"

#include "periph/gpio.h"
#include "xtimer.h"

/*
 * Pin mux definition
 */

#define GPIO_STEPPER_EN     GPIO_PIN(PORT_C, 11)
#define GPIO_STEPPER_DIR    GPIO_PIN(PORT_A, 10)
#define GPIO_STEPPER_STEP   GPIO_PIN(PORT_A, 11)

#define GPIO_SHARP_SENSOR1  GPIO_PIN(PORT_A, 7)

#define HALF_PERIOD_WAIT_US (20UL * US_PER_MS)

#define STEP_MAX_BEFORE_BLOCKED 50

#define STEP_MIN_STEPS      4

/* If following is defined, use sharp measurement on each stepper motor steps */
#define MEASUREMENT_ON_EACH_STEP
/* Number of step we consider the Sharp input value for end of turn */
#define MEASUREMENT_GRANULARITY 10

/*
 * internal helpers functions
 */

/* stepper controller */
static inline void _stepper_en(uint8_t en)
{
    gpio_write(GPIO_STEPPER_EN, !en); /* we do the logic inversion here */
}

static inline void _stepper_dir_set_cw(void)
{
    gpio_write(GPIO_STEPPER_DIR, 1);
}

static inline void _stepper_dir_set_ccw(void)
{
    gpio_write(GPIO_STEPPER_DIR, 0);
}

/* wheel detection */
static inline uint8_t _sharp_detects_hole(void)
{
    return gpio_read(GPIO_SHARP_SENSOR1);
}

/* some sequences */
static inline void _bitbanging_do_one_step(void)
{
    gpio_write(GPIO_STEPPER_STEP, 1);
    xtimer_usleep(HALF_PERIOD_WAIT_US);

    gpio_write(GPIO_STEPPER_STEP, 0);
    xtimer_usleep(HALF_PERIOD_WAIT_US);
}

#if defined(MODULE_CALIBRATION)
static void _turn_unconditionally(uint16_t nb_steps)
{
    for (uint16_t steps = 0; steps < nb_steps; steps++) {
        _bitbanging_do_one_step();
    }
}
#endif

/*
 * Public functions
 */

void motor_pap_init(void)
{
    gpio_init(GPIO_STEPPER_EN, GPIO_OUT);
    gpio_init(GPIO_STEPPER_DIR, GPIO_OUT);
    gpio_init(GPIO_STEPPER_STEP, GPIO_OUT);

    _stepper_en(FALSE);
}

uint8_t motor_pap_turn_next_storage(void)
{
    uint8_t i = 0;
    uint8_t retval = 0;
    uint8_t sharp_initial = FALSE;

    _stepper_en(TRUE);
    _stepper_dir_set_ccw();

    _bitbanging_do_one_step();
    _bitbanging_do_one_step();

    sharp_initial = _sharp_detects_hole();

    /* First ensure the value is logic 1 */
    if (!sharp_initial) {
        printf("PAP: started while sharp at 0, going to 1 first !\n");

        for (i = 0; i < STEP_MAX_BEFORE_BLOCKED; i++) {
#if defined(MEASUREMENT_ON_EACH_STEP)
            _bitbanging_do_one_step();
#else
            _turn_unconditionally(MEASUREMENT_GRANULARITY);
#endif

            printf("[%03d]\tTOR = %d\n", i, _sharp_detects_hole());

            if (i < STEP_MIN_STEPS) {
                continue;
            }

            if (_sharp_detects_hole() != sharp_initial) {
                break;
            }
        }
    }

    /* PAP is blocked, mechanical issue, should invalidate all actions ! */
    if (i == STEP_MAX_BEFORE_BLOCKED) {
        retval = 1;
        goto exit_point;
    }

    sharp_initial = _sharp_detects_hole();

    /* move till next storage location */
    for (i = 0; i < STEP_MAX_BEFORE_BLOCKED; i++) {
#if defined(MEASUREMENT_ON_EACH_STEP)
        _bitbanging_do_one_step();
#else
        _turn_unconditionally(MEASUREMENT_GRANULARITY);
#endif

        printf("[%03d]\tTOR = %d\n", i, _sharp_detects_hole());

        if (i < STEP_MIN_STEPS) {
            continue;
        }

        if (_sharp_detects_hole() != sharp_initial) {
            break;
        }
    }

    /* PAP is blocked, mechanical issue, should invalidate all actions ! */
    if (i == STEP_MAX_BEFORE_BLOCKED) {
        retval = 1;
    }

exit_point:
    xtimer_usleep(200 * US_PER_MS); // Wait 200ms before power off (wheel stop)
    _stepper_en(FALSE);

    return retval;
}

#if defined(MODULE_CALIBRATION)
static void motor_pap_calibration_usage(void)
{
    cons_printf("\n>>> Entering motor_pap calibration\n\n");

    cons_printf("\t'n' to move wheel to next position\n");
    cons_printf("\t't' to test max angle before BLOCKED state\n");
    cons_printf("\n");
    cons_printf("\t'h' to display this help\n");
    cons_printf("\t'q' to quit\n");
    cons_printf("\n");
}


void motor_pap_calib(void)
{
    int c;
    uint8_t quit = 0;

    motor_pap_calibration_usage();

    while (!quit) {
        /* display prompt */
        cons_printf("$ ");

        /* wait for command */
        c = cons_getchar();
        cons_printf("%c\n", c);

        switch (c) {
            case 'n':
                motor_pap_turn_next_storage();
                break;
            case 't':
                cons_printf("Turn %d steps\n", STEP_MAX_BEFORE_BLOCKED);
                _stepper_en(TRUE);
                _turn_unconditionally(STEP_MAX_BEFORE_BLOCKED);
                _stepper_en(FALSE);
                cons_printf("Done\n");
                break;
            case 'h':
                motor_pap_calibration_usage();
                break;
            case 'q':
                quit = 1;
                break;
            default:
                cons_printf("\n");
                break;
        }
    }
}
#endif
