/*
 * PWM + QDEC bench test for cogip-board-h5.
 *
 * STEP 1 identifies which physical wheel is motor id 0, STEP 2 characterizes
 * each motor (direction + QDEC A/B), then the tool prints a report with a QDEC
 * recap table, generates the ready-to-paste robot config, and optionally
 * re-runs with the corrections applied to validate the wiring in both
 * directions. See console.c / motor_test.c.
 */

#include <stdio.h>

#include "periph/qdec.h"
#include "motor_driver.h"

#include "console.h"
#include "motor_test.h"

int main(void)
{
    motor_driver_t md;
    motor_result_t res[2];

    puts("=== cogip-board-h5 PWM + QDEC bench test ===");

    if (motor_driver_init(&md, &motors_params) != 0) {
        puts("motor_driver_init FAILED");
        return 1;
    }
    /* QDEC_DEV(0) = left (TIM4), QDEC_DEV(1) = right (TIM3) */
    qdec_init(QDEC_DEV(0), QDEC_X4, NULL, NULL);
    qdec_init(QDEC_DEV(1), QDEC_X4, NULL, NULL);

    motor_enable(&md, 0);
    motor_enable(&md, 1);
    motor_set(&md, 0, 0);
    motor_set(&md, 1, 0);

    for (;;) {
        char s0 = identify(&md);
        unsigned left_id  = (s0 == 'l') ? 0 : 1;
        unsigned right_id = (s0 == 'l') ? 1 : 0;
        if (s0 != 'l') {
            puts(" motor id 0 is the RIGHT wheel -> running the test with corrected IDs");
            wait_key(" press any key to continue...\n");
        }

        test_motor(&md, left_id,  "left",  'l', &res[left_id]);
        test_motor(&md, right_id, "right", 'r', &res[right_id]);
        report(res, 2);
        bool conf_ok = print_conf(res, 2);

        /* requires the stdin module (see Makefile) so getchar()/ask_char block. */
        if (conf_ok) {
            int c = ask_char("\n [v]alidate with corrections, or [r]erun from scratch? (v/r): ",
                             "vr");
            if (c == 'v') {
                validate(res);
                puts("\npress ENTER to rerun from scratch...\n");
                int k;
                do {
                    k = getchar();
                } while (k != '\n' && k != '\r');
            }
        }
        else {
            puts("\npress ENTER to run the test again...\n");
            int c;
            do {
                c = getchar();
            } while (c != '\n' && c != '\r');
        }
    }

    return 0;
}
