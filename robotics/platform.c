#include <stdio.h>
#include <stdarg.h>

//#include "clksys.h"
//#include "gpio.h"
//#include "usart.h"
//
//#include "console.h"
//#include "kos.h"
//#include "msched.h"
#include "platform.h"
#include "platform_task.h"
#include <periph/qdec.h>
#include <motor_driver.h>
#include "analog_sensor.h"
#include <periph/adc.h>

#include "actuators/sd21.h"
#if defined(CONFIG_MOTOR_PAP)
#include "actuators/motor_pap.h"
#endif
#include <thread.h>

#include "xtimer.h"

/**
 * PORTA : ANA input
 *	PA0 - PA7 : IR
 * PORTB : JTAG + ANA inputs
 * PORTC : Communication
 *	PC0 (SDA) : I2C RGB Sensor Left + SD21
 *	PC1 (SCL) : I2C RGB Sensor Left + SD21
 *	PC2 (RX) : UART Debug communication
 *	PC3 (TX) : UART Debug communication
 *	PC4 (GPIO) : RGB LED Enable Left
 *	PC7 (GPIO) : RGB LED Enable Right
 * PORTD : Communication + PWM
 *	PD0 (SDA) : I2C RGB Sensor right
 *	PD1 (SCL) : I2C RGB Sensor right
 *	PD2 (OC0A) : PWM right motor
 *	PD3 (OCBA) : PWM left motor
 *	PD4 : DIR right motor
 *	PD5 : DIR left motor
 * PORTE : Timer + PWM
 *	PE4 : encoder A1 left wheel
 *	PE5 : encoder B1
 * PORTF : Timer decoder quadrature
 *	PF0 : encoder A2 right wheel
 *	PF1 : encoder B2
 *	PF2 : color switch
 *	PF3 : starter switch
 *
 * use TCC0 as general timer
 * use TCE0 timer to generate PWM signal
 * use TCE1, TCF0 and TCF1 timers to decode quadrature
 */

/* TODO: To activate when included in RIOT */
//#ifdef CONFIG_ANALOG_SENSORS
analog_sensors_t ana_sensors = {
	.sensors_nb = 6,
	.sensor_index = 0,
	.sensors = {
		/* Front: [10...80] cm - GP2Y0A21 - cal done */
		[0] = {
			.adc = 0,

			.pos_str = "F",

			.coeff_volts = 0.022,
			.const_volts = 0.010,
			.const_dist = -5.0,
			.dist_cm_max = 100,

			.dist_robot_offset_cm = 14,
			.angle_robot_offset = 0,
		},
		/* Rear: [10...80] cm - GP2Y0A21 - cal done */
		[1] = {
			.adc = 1,

			.pos_str = "R",

			.coeff_volts = 0.022,
			.const_volts = 0.010,
			.const_dist = -5.0,
			.dist_cm_max = 100,

			.dist_robot_offset_cm = 14,
			.angle_robot_offset = 180,
		},
		/* Front Side left: [4...30] cm - GP2YD120X - cal done */
		[2] = {
			.adc = 2,

			.pos_str = "FL",

			.coeff_volts = 0.052,
			.const_volts = 0.007,
			.const_dist = 0,
			.dist_cm_max = 40,

			.dist_robot_offset_cm = 16,
			.angle_robot_offset = 45,
		},
		/* Front Side right: [4...30] cm - GP2YD120X - cal done */
		[3] = {
			.adc = 3,

			.pos_str = "FR",

			.coeff_volts = 0.052,
			.const_volts = 0.007,
			.const_dist = 0,
			.dist_cm_max = 40,

			.dist_robot_offset_cm = 16,
			.angle_robot_offset = 315,
		},
		/* Rear Side left: [4...30] cm - GP2YD120X - cal done */
		[4] = {
			.adc = 4,

			.pos_str = "RL",

			.coeff_volts = 0.052,
			.const_volts = 0.007,
			.const_dist = 0,
			.dist_cm_max = 40,

			.dist_robot_offset_cm = 16,
			.angle_robot_offset = 135,
		},
		/* Rear Side right: [4...30] cm - GP2YD120X - cal done */
		[5] = {
			.adc = 5,

			.pos_str = "RR",

			.coeff_volts = 0.052,
			.const_volts = 0.007,
			.const_dist = 0,
			.dist_cm_max = 40,

			.dist_robot_offset_cm = 16,
			.angle_robot_offset = 225,
		},
//		/* -- Spare one (keep for calib value): [10...60] cm - GP2D12 - cal done */
//		[X] = {
//			.adc = X,
//
//			.coeff_volts = 0.045,
//			.const_volts = 0.027,
//			.const_dist = 5.0,
//			.dist_cm_max = 50,
//
//			.dist_robot_offset_cm = ,
//			.angle_robot_offset = ,
//		},
	}
};
//#endif /* CONFIG_ANALOG_SENSORS */

#if 0
qdec_t encoders[] = {
	{
		/* left motor */
		.pin_port = &PORTE,
		.pin_qdph0 = PIN4_bp,
		.pin_qdph90 = PIN5_bp,
		.event_channel = TC_EVSEL_CH0_gc,
		.tc = &TCE1,
		.line_count = WHEELS_ENCODER_RESOLUTION / 4,
		.polarity = 1,
	},
	{
		/* right motor */
		.pin_port = &PORTF,
		.pin_qdph0 = PIN0_bp,
		.pin_qdph90 = PIN1_bp,
		.event_channel = TC_EVSEL_CH2_gc,
		.tc = &TCF0,
		.line_count = WHEELS_ENCODER_RESOLUTION / 4,
		.polarity = 1,
	},
};
#endif

#if defined(CONFIG_SD21)
sd21_t sd21 = {
#ifdef BOARD_NATIVE
	.bus_id = 0,
#else
	.bus_id = I2C_0, /* I2C3 bus (cf. board.h) */
#endif
	.twi_speed_khz = I2C_SPEED_FAST,

	.servos_nb = SERVO_COUNT,
	.servos = {
		[SERVO_ID_VALVE_LAUNCHER] = {
			.value_init = 1350,
			.value_open = 2400,
			.value_close = 1350,
		},

		[SERVO_ID_VALVE_RECYCLER] = {
			.value_init = 1050,
			.value_open = 1925,
			.value_close = 1050,
		},

		[SERVO_ID_RECYCLER] = {
			.value_init = 1750,
			.value_open = 1750,
			.value_close = 1200,
		},

		[SERVO_ID_BEE_L] = {
			.value_init = 2125,
			.value_open = 1225,
			.value_close = 2125,
		},

		[SERVO_ID_BEE_R] = {
			.value_init = 875,
			.value_open = 1700,
			.value_close = 875,
		},

	},
};
#endif /* CONFIG_SD21 */

/* TCE0 ClkIn == ClkPer / 8 == 4000 KHz */
/* Counter set to 200 for 20KHz output */
#define TC_MOTOR_PRESCALER		TC_CLKSEL_DIV8_gc
#define TC_MOTOR_PER_VALUE		200

#if 0
hbridge_t hbridges = {
	.tc = &TCD0,
	.period = TC_MOTOR_PER_VALUE,
	.prescaler = TC_MOTOR_PRESCALER,

	.pwm_port = &PORTD, /* TODO: can be 'guessed' from timer ref above */

	.engine_nb = 2,
	.engines = {
		[HBRIDGE_MOTOR_LEFT] = {
			/* left motor */
			.direction_pin_port = &PORTD,
			.direction_pin_id = PIN5_bp,
			.direction_inverse_polarity = FALSE,
			.pwm_channel = PIN3_bp,
			.offset = 0,
		},
		[HBRIDGE_MOTOR_RIGHT] = {
			/* right motor */
			.direction_pin_port = &PORTD,
			.direction_pin_id = PIN4_bp,
			.direction_inverse_polarity = TRUE,
			.pwm_channel = PIN2_bp,
			.offset = 0,
		},
	},
};
#endif

controller_mode_t controller_modes[] = {
	{ "STOP", ctrl_state_stop_cb, },		/* CTRL_STATE_STOP */
	{ "IDLE", ctrl_state_idle_cb, },		/* CTRL_STATE_IDLE */
	{ "BLOCKED", ctrl_state_stop_cb, },		/* CTRL_STATE_BLOCKED */
	{ "INGAME", ctrl_state_ingame_cb, },		/* CTRL_STATE_INGAME */
#if defined(MODULE_CALIBRATION)
	{ "CALIB1", ctrl_state_calib_mode1_cb, },	/* CTRL_STATE_CALIB_MODE1 */
	{ "CALIB2", ctrl_state_calib_mode2_cb, },	/* CTRL_STATE_CALIB_MODE2 */
	{ "CALIB3", ctrl_state_calib_mode3_cb, },	/* CTRL_STATE_CALIB_MODE3 */
#endif /* MODULE_CALIBRATION */
};

controller_t controller = {
#ifdef BOARD_NATIVE
	.linear_speed_pid = {
		.kp = 5,
		.ki = 0.02,
		.kd = 0,
	},
	.angular_speed_pid = {
		.kp = 5,
		.ki = 0.02,
		.kd = 0,
	},
	.linear_pose_pid = {
		.kp = 0.5,
		.ki = 0.,
		.kd = 0,
	},
	.angular_pose_pid = {
		.kp = 0.5,
		.ki = 0.,
		.kd = 0.,
	},
#else
	.linear_speed_pid = {
		.kp = 15.,
		.ki = 2.,
		.kd = 0.,
	},
	.angular_speed_pid = {
		.kp = 15.,
		.ki = 2.,
		.kd = 0.,
	},
	.linear_pose_pid = {
		.kp = 0.05,
		.ki = 0.,
		.kd = 0,
	},
	.angular_pose_pid = {
		.kp = 0.1,
		.ki = 0.,
		.kd = 0.,
	},
#endif

	//.min_distance_for_angular_switch = 500,
	.min_distance_for_angular_switch = 30,
	.min_angle_for_pose_reached = 100,
	.regul = CTRL_REGUL_POSE_DIST,
	.allow_reverse = TRUE,
	.mode = &controller_modes[CTRL_STATE_INGAME],
};

/* This global object contains all numerical logs references (vectors, etc.) */
datalog_t datalog;

static void mach_post_ctrl_loop_func(void)
{
	//analog_sensor_refresh_all(&ana_sensors);
}

inline func_cb_t mach_get_ctrl_loop_pre_pfn(void)
{
	return NULL;
}

inline func_cb_t mach_get_ctrl_loop_post_pfn(void)
{
	return mach_post_ctrl_loop_func;
}

inline func_cb_t mach_get_end_of_game_pfn(void)
{
	return NULL;
}

path_t * mach_get_path(void)
{
	return &robot_path;
}

/* TODO: To activate when included in RIOT */
//uint8_t mach_is_zone_obscured(analog_sensor_zone_t zone)
//{
//#ifdef CONFIG_ANALOG_SENSORS
//   return analog_sensor_detect_obstacle(&ana_sensors, zone);
//#else
//   (void)zone;
//   return 0;
//#endif
//}
//FIXME: stub to remove
#define gpio_get_input(...) 0
#define gpio_set_direction(...)
#define gpio_set_output(...)
#define hbridge_setup(...)
#define msched_init(...)
#define kos_run(...)

uint8_t mach_is_game_launched(void)
{
	/* Starter switch */
#if defined(CONFIG_USE_STARTER) && !defined(BOARD_NATIVE)
	/* read 4 when the bar is NOT yet removed */
	return gpio_read(GPIO_PIN(PORT_B, 2)) ? 0 : 1;
#else
	return 1;
#endif
}

uint8_t mach_is_camp_left(void)
{
	/* Color switch for coords translations */
	gpio_init(GPIO_PIN(PORT_B, 10), GPIO_IN);

	xtimer_usleep(100 * US_PER_MS); // Wait 100ms (debounce)

	return gpio_read(GPIO_PIN(PORT_B, 10)) ? 1 : 0;
}

static void mach_pinmux_setup(void)
{
#if defined(__AVR__)
	/* analog to digital conversion */
	PORTA.DIR = 0x00; /*!< PORTA as input pin */
	PORTA.OUT = 0x00;
	PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

	/* Port B - Jtag disable (fuse bit required) */
	MCU_MCUCR = MCU_JTAGD_bm; /* Fuse4 bit0 to set to 1 with flasher */

	/* twi configuration pin */
	PORTC.DIRSET = PIN1_bm; /*!< PC1 (SCL) as output pin */
	/* usart configuration pin */
	PORTC.DIRCLR = PIN2_bm; /*!< PC2 (RDX0) as input pin */
	PORTC.DIRSET = PIN3_bm; /*!< PC3 (TXD0) as output pin */
#endif

	/* Pumps, outputs all off. */
	gpio_set_direction(&PORTB, PIN0_bp, GPIO_DIR_OUT);
	gpio_set_direction(&PORTB, PIN1_bp, GPIO_DIR_OUT);
	gpio_set_direction(&PORTB, PIN2_bp, GPIO_DIR_OUT);
	gpio_set_direction(&PORTB, PIN3_bp, GPIO_DIR_OUT);
	gpio_set_direction(&PORTB, PIN4_bp, GPIO_DIR_OUT);
	gpio_set_direction(&PORTB, PIN5_bp, GPIO_DIR_OUT);

	gpio_set_output(&PORTB, GPIO_ID_PUMP_FR, 0); /* Front Right Pump */
	gpio_set_output(&PORTB, GPIO_ID_PUMP_RR, 0); /* Rear Right Pump */
	gpio_set_output(&PORTB, PIN2_bp, 0); /* Not Connected */
	gpio_set_output(&PORTB, PIN3_bp, 0); /* Not Connected */
	gpio_set_output(&PORTB, GPIO_ID_PUMP_FL, 0); /* Front Left Pump */
	gpio_set_output(&PORTB, GPIO_ID_PUMP_RL, 0); /* Rear Left Pump */

	/* Starter switch */
	gpio_set_direction(&PORTF, PIN3_bp, GPIO_DIR_IN);
	/* Color switch */
	gpio_set_direction(&PORTF, PIN2_bp, GPIO_DIR_IN);
}

void mach_sched_init(void)
{
	msched_init(10/*ms*/, &TCC0);
}

void mach_sched_run(void)
{
	kos_run();
}

//console_t usartc0_console = {
//	.usart = &USART_CONSOLE,
//	.speed = 115200,
//};

void mach_setup(void)
{
#if F_CPU == 32000000UL
	clksys_intrc_32MHz_setup();
#endif
	mach_pinmux_setup();

#ifdef CONFIG_ENABLE_LOGGING
	/* setup logs through usart communication */
	console_init(&usartc0_console);
#endif

	analog_sensor_setup(&ana_sensors);

#if defined(CONFIG_SD21)
	/* setup TWI communication with SD21 */
	sd21_setup(&sd21);
#endif /* CONFIG_SD21 */

	//action_setup(); /* TODO: commenter pour debug */

	motor_driver_init(0);
#if defined(CONFIG_MOTOR_PAP)
	motor_pap_init();
#endif

	/* Starter, orig 1 & 2, color */
	gpio_init(GPIO_PIN(PORT_A, 0), GPIO_IN_PU);
	gpio_init(GPIO_PIN(PORT_A, 1), GPIO_IN_PU);
	gpio_init(GPIO_PIN(PORT_B, 2), GPIO_IN_PU);
	gpio_init(GPIO_PIN(PORT_B, 10), GPIO_IN);

	action_init();

	/* setup qdec */
//#ifdef BOARD_NATIVE
	int error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_LEFT), QDEC_MODE, NULL, NULL);
	if (error)
		printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_LEFT, error);
	error = qdec_init(QDEC_DEV(HBRIDGE_MOTOR_RIGHT), QDEC_MODE, NULL, NULL);
	if (error)
		printf("QDEC %u not initialized, error=%d !!!\n", HBRIDGE_MOTOR_RIGHT, error);

	/* controller setup */
	odometry_setup(WHEELS_DISTANCE);

#if defined(__AVR__)
	/* Programmable Multilevel Interrupt Controller */
	PMIC.CTRL |= PMIC_LOLVLEN_bm; /* Low-level Interrupt Enable */

	/* global interrupt enable */
	sei();
#endif

	log_vect_init(&datalog, NULL, /*400,*/
			COL_INT32, "left_speed",
			COL_INT32, "right_speed",
			COL_INT16, "left_command",
			COL_INT16, "right_command",
			COL_DOUBLE, "robot_speed.distance",
			//COL_DOUBLE, "robot_speed.angle",
			COL_DOUBLE, "speed_order.distance",
			//COL_DOUBLE, "speed_order.angle",
			COL_END);
}

void ctrl_state_stop_cb(pose_t *robot_pose, polar_t *motor_command)
{
	(void)robot_pose;
	/* final position */
	motor_command->distance = 0;
	motor_command->angle = 0;
}

void ctrl_state_idle_cb(pose_t *robot_pose, polar_t *motor_command)
{
	(void)motor_command;
	(void)robot_pose;

	//FIXME: read encoders
}

void ctrl_state_ingame_cb(pose_t *robot_pose, polar_t *motor_command)
{
	polar_t	robot_speed		= { 0, 0 };

	/* catch speed */
	encoder_read(&robot_speed);

	/* convert to position */
	odometry_update(robot_pose, &robot_speed, SEGMENT);

	/* convert pulse to degree */
	robot_pose->O /= PULSE_PER_DEGREE;

	/* PID / feedback control */
	*motor_command = controller_update(&controller,
					  robot_pose,
					  robot_speed);

	/* convert degree to pulse */
	robot_pose->O *= PULSE_PER_DEGREE;
}

int encoder_read(polar_t *robot_speed)
{
	int32_t left_speed = qdec_read_and_reset(HBRIDGE_MOTOR_LEFT) * QDEC_LEFT_POLARITY;
	int32_t right_speed = qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT) * QDEC_RIGHT_POLARITY;

	/* update speed */
	robot_speed->distance = (right_speed + left_speed) / 2.0;
	robot_speed->angle = right_speed - left_speed;

	log_vect_setvalue(&datalog, LOG_IDX_SPEED_L, (void *) &left_speed);
	log_vect_setvalue(&datalog, LOG_IDX_SPEED_R, (void *) &right_speed);

	return 0;
}

void encoder_reset(void)
{
	qdec_read_and_reset(HBRIDGE_MOTOR_LEFT);
	qdec_read_and_reset(HBRIDGE_MOTOR_RIGHT);
}

/* FIXME: put following func elsewhere */
/* Note: requires "LINKFLAGS += -u _scanf_float" */
int custom_scanf(const char *format, ...)
{
	va_list args;

	int c, retval;
	char buffer[100];
	uint8_t idx = 0;

	while ((c = cons_getchar()) != '\r' && idx < 100) {
		printf("%c", c); fflush(stdout);
		buffer[idx++] = c;
	}
	buffer[idx] = '\0';

	va_start (args, format);
	retval = vsscanf(buffer, format, args);
	va_end (args);

	return retval;
}
