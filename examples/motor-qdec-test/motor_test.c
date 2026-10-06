/*
 * PWM + QDEC bench test logic for cogip-board-h5: identification, per-motor
 * characterization, report, config generation, and validation.
 */

#include <stdio.h>
#include <stdlib.h>

#include "ztimer.h"
#include "periph/gpio.h"
#include "periph/qdec.h"
#include "motor_driver.h"

#include "console.h"
#include "motor_test.h"

#define PWM_RESOLUTION  (500U)
#define DUTY_PERCENT    (20)
#define DUTY            ((int32_t)(PWM_RESOLUTION * DUTY_PERCENT / 100)) /* 100 */
#define RUN_MS          (3000U)
#define PAUSE_MS        (1000U)

/* below this many X4 pulses over a run the result is treated as "no/low
 * movement" rather than a wiring verdict */
#define MIN_PULSES      (50)

/* cogip-board-h5 motion motors (mirrors motion_motors_params for the h5 pin map:
 * brake PB3/PB2, dir PC6/PB10, enable PA10/PB1, PWM TIM1 CH1/CH2). */
const motor_driver_params_t motors_params = {
    .mode = MOTOR_DRIVER_1_DIR_BRAKE,
    .pwm_dev = 0,
    .pwm_mode = PWM_LEFT,
    .pwm_frequency = 20000U,
    .pwm_resolution = PWM_RESOLUTION,
    .brake_inverted = true,
    .enable_inverted = false,
    .nb_motors = 2,
    .motors =
        {
            {
                .pwm_channel = 0,
                .gpio_enable = GPIO_PIN(PORT_A, 10),
                .gpio_dir0 = GPIO_PIN(PORT_C, 6),
                .gpio_brake = GPIO_PIN(PORT_B, 3),
                .gpio_dir_reverse = 1,
            },
            {
                .pwm_channel = 1,
                .gpio_enable = GPIO_PIN(PORT_B, 1),
                .gpio_dir0 = GPIO_PIN(PORT_B, 10),
                .gpio_brake = GPIO_PIN(PORT_B, 2),
                .gpio_dir_reverse = 0,
            },
        },
    .motor_set_post_cb = NULL,
};

/* Positive PWM is expected to give a positive (increasing) QDEC count and
 * negative PWM a negative count. Print and return the correction constant to
 * apply in the motion firmware (+1 ok, -1 inverted, 0 inconclusive). */
static int verdict(int32_t fwd, int32_t rev)
{
    if (fwd > MIN_PULSES && rev < -MIN_PULSES) {
        puts("  A/B wiring : OK            -> QDEC correction = +1");
        return +1;
    }
    if (fwd < -MIN_PULSES && rev > MIN_PULSES) {
        puts("  A/B wiring : INVERTED      -> QDEC correction = -1  (swap A/B)");
        return -1;
    }
    printf("  A/B wiring : INCONCLUSIVE  (fwd=%ld rev=%ld, motor barely moved)\n",
           (long)fwd, (long)rev);
    return 0;
}

char identify(motor_driver_t *md)
{
    clear_screen();
    hr();
    puts(" STEP 1: identify motor id 0");
    hr();
    puts(" Look at the motors.");
    wait_key(" Press any key to spin motor id 0 briefly...\n");

    qdec_read_and_reset(QDEC_DEV(0));
    qdec_read_and_reset(QDEC_DEV(1));
    motor_set(md, 0, DUTY);
    ztimer_sleep(ZTIMER_MSEC, 1500U);
    motor_set(md, 0, 0);
    int32_t d0 = qdec_read_and_reset(QDEC_DEV(0));
    int32_t d1 = qdec_read_and_reset(QDEC_DEV(1));
    printf(" enc L(TIM4,DEV0)=%ld  enc R(TIM3,DEV1)=%ld\n", (long)d0, (long)d1);

    if (labs(d0) < MIN_PULSES && labs(d1) < MIN_PULSES) {
        puts(" !! no QDEC signal, check encoder wiring/power");
    }
    else if (labs(d1) > labs(d0)) {
        puts(" !! CROSS-WIRED: motor 0 driven but encoder DEV1 (R/TIM3) moved");
        puts("    PWM and encoder are on different motors, swap encoder connectors");
    }

    return (char)ask_char(" which wheel turned? (l=left / r=right): ", "lr");
}

void test_motor(motor_driver_t *md, uint8_t id, const char *label, char side,
                motor_result_t *res)
{
    clear_screen();
    hr();
    printf(" STEP 2: %s wheel  (motor id %u)\n", label, id);
    hr();
    printf(" Look at the %s motor.\n", label);
    wait_key(" Press any key to spin it...\n");

    qdec_read_and_reset(QDEC_DEV(0));
    qdec_read_and_reset(QDEC_DEV(1));
    printf("\n forward  (PWM +20%%, %u ms)\n", (unsigned)RUN_MS);
    motor_set(md, id, DUTY);
    ztimer_sleep(ZTIMER_MSEC, RUN_MS);
    motor_set(md, id, 0);
    int32_t l_fwd = qdec_read_and_reset(QDEC_DEV(0));
    int32_t r_fwd = qdec_read_and_reset(QDEC_DEV(1));
    printf("   DEV0 enc (TIM4) = %7ld\n", (long)l_fwd);
    printf("   DEV1 enc (TIM3) = %7ld\n", (long)r_fwd);

    ztimer_sleep(ZTIMER_MSEC, PAUSE_MS);

    printf("\n reverse  (PWM -20%%, %u ms)\n", (unsigned)RUN_MS);
    motor_set(md, id, -DUTY);
    ztimer_sleep(ZTIMER_MSEC, RUN_MS);
    motor_set(md, id, 0);
    int32_t l_rev = qdec_read_and_reset(QDEC_DEV(0));
    int32_t r_rev = qdec_read_and_reset(QDEC_DEV(1));
    printf("   DEV0 enc (TIM4) = %7ld\n", (long)l_rev);
    printf("   DEV1 enc (TIM3) = %7ld\n", (long)r_rev);

    /* the encoder coupled to this motor is the one that moved most */
    bool left_active = (labs(l_fwd) + labs(l_rev)) >= (labs(r_fwd) + labs(r_rev));
    const char *enc = left_active ? "DEV0 (TIM4)" : "DEV1 (TIM3)";
    int32_t fwd = left_active ? l_fwd : r_fwd;
    int32_t rev = left_active ? l_rev : r_rev;

    putchar('\n');
    printf("  coupled enc: %s\n", enc);
    int corr = verdict(fwd, rev);

    /* cross-wire check at test time: driving motor <id> should move its paired
     * encoder QDEC_DEV(id); the other responding means different motors. */
    unsigned coupled_dev = left_active ? 0 : 1;
    bool cross_wired = (corr != 0) && (coupled_dev != id);
    if (cross_wired) {
        printf("  !! CROSS-WIRED: motor %u driven but encoder %s (DEV%u) moved\n",
               id, enc, coupled_dev);
        puts("     PWM and encoder are on different motors, swap encoder connectors");
    }
    hr();

    int fdir = ask_char(" on 'forward', did the wheel go FORWARD?  (y/n): ", "yn");
    int rdir = ask_char(" on 'reverse', did the wheel go BACKWARD? (y/n): ", "yn");
    bool dir_bad = (fdir != rdir); /* consistent motor flips: y/y or n/n */

    res->label      = label;
    res->moved      = (corr != 0);
    res->side       = side;
    res->dir_ok     = (fdir == 'y');
    res->dir_bad    = dir_bad;
    res->correction = corr;
    res->left_enc   = left_active;
    res->enc_mismatch = cross_wired;
    res->fwd        = fwd;
    res->rev        = rev;

    if (dir_bad) {
        puts("  !! INCONSISTENT: motor did not reverse direction");
        puts("     (same way both times) check the direction pin / driver");
    }

    putchar('\n');
    printf(" => motor %u is the %s wheel, forward direction %s, encoder %s\n",
           id, (side == 'l') ? "LEFT" : "RIGHT",
           (fdir == 'y') ? "CORRECT" : "REVERSED", enc);
}

void report(const motor_result_t *r, unsigned n)
{
    clear_screen();
    hr();
    puts(" FINAL REPORT");
    hr();
    for (unsigned i = 0; i < n; i++) {
        printf(" motor %u (\"%s\"): wheel=%-5s  forward=%-8s  QDEC corr=%+d  enc=%s\n",
               i, r[i].label,
               (r[i].side == 'l') ? "LEFT" : "RIGHT",
               r[i].dir_ok ? "CORRECT" : "REVERSED",
               r[i].correction,
               r[i].left_enc ? "DEV0(TIM4)" : "DEV1(TIM3)");
        if (!r[i].moved) {
            puts("          (warning: inconclusive, motor barely moved)");
        }
        if (r[i].enc_mismatch) {
            puts("          (!! CROSS-WIRED: encoder is on the other motor, swap connectors)");
        }
        if (r[i].dir_bad) {
            puts("          (!! direction inconsistent: motor did not reverse, check dir pin)");
        }
    }
    hr();
    puts(" QDEC recap (coupled encoder, X4). corrected = raw * polarity:");
    puts("   motor        PWM      raw    corrected   enc");
    for (unsigned i = 0; i < n; i++) {
        int pol = r[i].correction * (r[i].dir_ok ? 1 : -1);
        const char *e = r[i].left_enc ? "DEV0(TIM4)" : "DEV1(TIM3)";
        printf("   %u (%-5s)  +20%%  %7ld   %7ld   %s\n",
               i, r[i].label, (long)r[i].fwd, (long)(r[i].fwd * pol), e);
        printf("   %u (%-5s)  -20%%  %7ld   %7ld   %s\n",
               i, r[i].label, (long)r[i].rev, (long)(r[i].rev * pol), e);
    }
    hr();
    if (n >= 2 && r[0].side == r[1].side) {
        puts(" WARNING: both motors reported the SAME wheel, re-check.");
    }
    else if (n >= 2) {
        printf(" id->wheel: 0=%s 1=%s\n",
               (r[0].side == 'l') ? "LEFT" : "RIGHT",
               (r[1].side == 'l') ? "LEFT" : "RIGHT");
        if (r[0].side == 'l') {
            puts(" params order OK: id 0 = left, id 1 = right");
        }
        else {
            puts(" params order SWAPPED: give id 0 to the right motor, id 1 to the left");
        }
        printf(" wheels: %s = id %u + enc DEV%u,  %s = id %u + enc DEV%u\n",
               "LEFT",  (r[0].side == 'l') ? 0 : 1, (r[0].side == 'l') ? 0 : 1,
               "RIGHT", (r[0].side == 'r') ? 0 : 1, (r[0].side == 'r') ? 0 : 1);
        puts(" (TIM4/TIM3 are fixed timers; each motor id uses encoder DEV<id>,");
        puts("  so the left/right swap moves the motor AND its encoder together)");
    }
    hr();
}

bool print_conf(const motor_result_t *r, unsigned n)
{
    hr();
    puts(" SUGGESTED CONFIG");
    hr();

    bool ok = true;
    for (unsigned i = 0; i < n; i++) {
        if (!r[i].moved) {
            printf(" motor %u: inconclusive, rerun\n", i);
            ok = false;
        }
        if (r[i].enc_mismatch) {
            printf(" motor %u: cross-wired, recable encoder first\n", i);
            ok = false;
        }
        if (r[i].dir_bad) {
            printf(" motor %u: direction inconsistent (no reverse), check dir pin/driver\n", i);
            ok = false;
        }
    }
    if (n < 2 || r[0].side == r[1].side) {
        puts(" wheel sides ambiguous (same answer twice?), cannot map ids");
        ok = false;
    }
    if (!ok) {
        puts(" -> fix the above, then rerun to get the config");
        hr();
        return false;
    }

    unsigned left_id  = (r[0].side == 'l') ? 0 : 1;
    unsigned right_id = (r[0].side == 'r') ? 0 : 1;
    int pol_l = r[left_id].correction  * (r[left_id].dir_ok  ? 1 : -1);
    int pol_r = r[right_id].correction * (r[right_id].dir_ok ? 1 : -1);

    puts(" // applications/robot-motion-control/include/robotX_conf.hpp");
    printf(" #define MOTOR_LEFT  %u\n", left_id);
    printf(" #define MOTOR_RIGHT %u\n", right_id);
    /* RIOT's default printf has no %f (no printf_float); polarity is +-1, so
     * print it as an int with a .0 suffix to get a valid float literal. */
    printf(" constexpr float default_qdec_left_polarity  = %d.0;\n", pol_l);
    printf(" constexpr float default_qdec_right_polarity = %d.0;\n", pol_r);
    putchar('\n');
    puts(" // platforms/pf-robot-motion-control/include/motion_motors_params.hpp");
    for (unsigned i = 0; i < n; i++) {
        int rec = motors_params.motors[i].gpio_dir_reverse ^ (r[i].dir_ok ? 0 : 1);
        printf("   motors[%u].gpio_dir_reverse = %d;\n", i, rec);
    }
    hr();
    return true;
}

void validate(const motor_result_t *r)
{
    motor_driver_params_t p = motors_params;
    int pol[2];
    for (unsigned i = 0; i < 2; i++) {
        p.motors[i].gpio_dir_reverse =
            motors_params.motors[i].gpio_dir_reverse ^ (r[i].dir_ok ? 0 : 1);
        pol[i] = r[i].correction * (r[i].dir_ok ? 1 : -1);
    }

    motor_driver_t vd;
    motor_driver_init(&vd, &p);
    motor_enable(&vd, 0);
    motor_enable(&vd, 1);
    motor_set(&vd, 0, 0);
    motor_set(&vd, 1, 0);

    unsigned left_id  = (r[0].side == 'l') ? 0 : 1;
    unsigned order[2] = { left_id, (left_id ^ 1u) };
    const char *lbl[2] = { "left", "right" };

    clear_screen();
    hr();
    puts(" VALIDATION (corrections applied)");
    hr();

    bool all_ok = true;
    for (unsigned k = 0; k < 2; k++) {
        unsigned id = order[k];
        printf("\n %s wheel (motor id %u)\n", lbl[k], id);

        wait_key(" watch it, press any key to drive FORWARD...\n");
        qdec_read_and_reset(QDEC_DEV(id));
        motor_set(&vd, id, DUTY);
        ztimer_sleep(ZTIMER_MSEC, RUN_MS);
        motor_set(&vd, id, 0);
        int32_t cf = qdec_read_and_reset(QDEC_DEV(id)) * pol[id];
        int fdir = ask_char("   wheel went FORWARD?  (y/n): ", "yn");

        ztimer_sleep(ZTIMER_MSEC, PAUSE_MS);

        wait_key(" press any key to drive REVERSE...\n");
        qdec_read_and_reset(QDEC_DEV(id));
        motor_set(&vd, id, -DUTY);
        ztimer_sleep(ZTIMER_MSEC, RUN_MS);
        motor_set(&vd, id, 0);
        int32_t cr = qdec_read_and_reset(QDEC_DEV(id)) * pol[id];
        int rdir = ask_char("   wheel went BACKWARD? (y/n): ", "yn");

        printf("   corrected: forward=%ld  reverse=%ld  (polarity %+d)\n",
               (long)cf, (long)cr, pol[id]);

        /* a correctly wired encoder must reverse the count sign with direction.
         * in X1, or with an open A/B channel, it may not, keeping the same sign
         * both ways. require clear, opposite-sign counts; stop otherwise. */
        bool discriminates = (labs(cf) > MIN_PULSES) && (labs(cr) > MIN_PULSES)
                             && ((cf > 0) != (cr > 0));
        if (!discriminates) {
            puts("   !! BAD WIRING: encoder count does not reverse with direction");
            puts("      (X1 mode or open A/B channel?) check encoder wiring");
            motor_set(&vd, 0, 0);
            motor_set(&vd, 1, 0);
            motor_brake(&vd, 0);
            motor_brake(&vd, 1);
            hr();
            puts(" VALIDATION ABORTED (bad wiring)");
            hr();
            return;
        }

        bool pass = (cf > MIN_PULSES) && (cr < -MIN_PULSES)
                    && (fdir == 'y') && (rdir == 'y');
        printf("   => %s\n", pass ? "PASS" : "FAIL");
        if (!pass) {
            all_ok = false;
        }
    }

    motor_set(&vd, 0, 0);
    motor_set(&vd, 1, 0);
    motor_brake(&vd, 0);
    motor_brake(&vd, 1);

    hr();
    puts(all_ok ? " VALIDATION PASSED" : " VALIDATION FAILED, re-check wiring/config");
    hr();
}
