#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"

/* Per-motor result kept for the end-of-run report and the config generator. */
typedef struct {
    const char *label;  /* role label ("left"/"right") */
    bool  moved;        /* enough movement to judge */
    char  side;         /* physical wheel: 'l' or 'r' */
    bool  dir_ok;       /* forward command really drove forward */
    bool  dir_bad;      /* forward/reverse answers inconsistent (motor won't reverse) */
    int   correction;   /* QDEC A/B correction: +1, -1, or 0 if inconclusive */
    bool  left_enc;     /* coupled encoder: true = L (TIM4), false = R (TIM3) */
    bool  enc_mismatch; /* driven motor id != encoder that responded */
    int32_t fwd;        /* coupled encoder pulses on the forward run */
    int32_t rev;        /* coupled encoder pulses on the reverse run */
} motor_result_t;

/* cogip-board-h5 motion motors (mirrors motion_motors_params). */
extern const motor_driver_params_t motors_params;

char identify(motor_driver_t *md);
void test_motor(motor_driver_t *md, uint8_t id, const char *label, char side,
                motor_result_t *res);
void report(const motor_result_t *r, unsigned n);
bool print_conf(const motor_result_t *r, unsigned n);
void validate(const motor_result_t *r);
