#include <stdio.h>

//#include "clksys.h"
//#include "gpio.h"
//#include "usart.h"
//
//#include "console.h"
//#include "kos.h"
//#include "msched.h"
#include "platform.h"
#include "platform_task.h"

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
//analog_sensors_t ana_sensors = {
//	.adc = &ADCA,
//
//	.sensors_nb = 8,
//	.sensors = {
//		/* Rear right: [10...80] cm - GP2Y0A21 - cal done */
//		[0] = {
//			.pin_id = PIN0_bp,
//
//			.coeff_volts = 0.022,
//			.const_volts = 0.010,
//			.const_dist = -5.0,
//			.dist_cm_max = 50,
//
//			.zone = (AS_ZONE_REAR | AS_ZONE_RIGHT),
//		},
//		/* Front left: [10...80] cm - GP2Y0A21 - cal done */
//		[1] = {
//			.pin_id = PIN1_bp,
//
//			.coeff_volts = 0.03,
//			.const_volts = 0.009,
//			.const_dist = 1.0,
//			.dist_cm_max = 80,
//
//			.zone = (AS_ZONE_FRONT | AS_ZONE_LEFT),
//		},
//		/* Front right: [10...60] cm - GP2D12 - cal done*/
//		[2] = {
//			.pin_id = PIN2_bp,
//
//			.coeff_volts = 0.045,
//			.const_volts = 0.027,
//			.const_dist = 5.0,
//			.dist_cm_max = 50,
//
//			.zone = (AS_ZONE_FRONT | AS_ZONE_RIGHT),
//		},
//		/* *************** Front Side left: [4...30] cm - GP2YD120X - cal NOT done */
//		[3] = {
//			.pin_id = PIN3_bp,
//
//			.coeff_volts = 0.052,
//			.const_volts = 0.007,
//			.const_dist = 0,
//			.dist_cm_max = 40,
//
//			.zone = (AS_ZONE_REAR | AS_ZONE_LEFT),
//		},
//		/* *************** Front Side right: [4...30] cm - GP2YD120X - cal NOT done */
//		[4] = {
//			.pin_id = PIN4_bp,
//
//			.coeff_volts = 0.052,
//			.const_volts = 0.007,
//			.const_dist = 0,
//			.dist_cm_max = 40,
//
//			.zone = (AS_ZONE_REAR | AS_ZONE_RIGHT),
//		},
//		/* Front Side left: [4...30] cm - GP2YD120X - cal done */
//		[5] = {
//			.pin_id = PIN5_bp,
//
//			.coeff_volts = 0.052,
//			.const_volts = 0.007,
//			.const_dist = 0,
//			.dist_cm_max = 40,
//
//			.zone = (AS_ZONE_FRONT | AS_ZONE_LEFT),
//		},
//		/* Front Side right: [4...30] cm - GP2YD120X - cal done */
//		[6] = {
//			.pin_id = PIN6_bp,
//
//			.coeff_volts = 0.052,
//			.const_volts = 0.007,
//			.const_dist = 0,
//			.dist_cm_max = 40,
//
//			.zone = (AS_ZONE_FRONT | AS_ZONE_RIGHT),
//		},
//		/* Rear left: [10...80] cm - GP2Y0A21 - cal done  */
//		[7] = {
//			.pin_id = PIN7_bp,
//
//			.coeff_volts = 0.03,
//			.const_volts = 0.009,
//			.const_dist = 1.0,
//			.dist_cm_max = 80,
//
//			.zone = (AS_ZONE_REAR | AS_ZONE_LEFT),
//		},
//	}
//};
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
	.twi = &TWIC,
	.twi_speed_khz = 100,

	.servos_nb = 8,
	.servos = {
		/* Front-Left */
		[0] = {
			.value_init = 550/*1500*/,
			.value_open = 1850,
			.value_close = 550,
		},

		/* Bottom-Left */
		[1] = {
			.value_init = 2400/*1500*/,
			.value_open = 1075,
			.value_close = 2400,
		},

		/* Bottom-Right */
		[2] = {
			.value_init = 600/*1500*/,
			.value_open = 1925,
			.value_close = 600,
		},

		/* Front-Right */
		[3] = {
			.value_init = 2500/*1500*/,
			.value_open = 1150,
			.value_close = 2500,
		},

		/* Glasses right arm */
		[4] = {
			.value_init = 0,//1500,//600,
			.value_open = 600,
			.value_close = 2450,
		},

		/* Glasses left arm */
		[5] = {
			.value_init = 0,//1500,//2600,
			.value_open = 2600,
			.value_close = 800,
		},


		/* Left arm */
		[6] = {
			.value_init = 1850,
			.value_open = 800,
			.value_close = 1850,
		},

		/* Right arm */
		[7] = {
			.value_init = 950,
			.value_open = 1975,
			.value_close = 950,
		},

		///* Clamp */
		//[8] = {
		//	.value_init = 1700,
		//	.value_open = 2120,
		//	.value_close = 1700,
		//},
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
	{ "STOP", ctrl_state_stop_cb, },	/* CTRL_STATE_STOP */
	{ "IDLE", ctrl_state_idle_cb, },	/* CTRL_STATE_IDLE */
	{ "INGAME", ctrl_state_ingame_cb, },	/* CTRL_STATE_INGAME */
};

controller_t controller = {
#if defined(__AVR__)
	.linear_speed_pid = {
		.kp = 2.0,
		.ki = 0.1,
		.kd = 0,
	},
	.angular_speed_pid = {
		.kp = 2.0,
		.ki = 0.1,
		.kd = 0,
	},
	.linear_pose_pid = {
		.kp = 0.050,
		.ki = 0,
		.kd = 0.4,
	},
	.angular_pose_pid = {
		.kp = 0.050,
		.ki = 0,
		.kd = 0.4,
	},
#else
	.linear_speed_pid = {
		.kp = 2.9,
		.ki = 0.15,
		.kd = 0,
	},
	.angular_speed_pid = {
		.kp = 2,
		.ki = 0.05,
		.kd = 0,
	},
	.linear_pose_pid = {
		.kp = 0.050,
		.ki = 0,
		.kd = 0.4,
	},
	.angular_pose_pid = {
		.kp = 0.050,
		.ki = 0,
		.kd = 0.4,
	},
#endif

	//.min_distance_for_angular_switch = 500,
	.min_distance_for_angular_switch = 100,
	.min_angle_for_pose_reached = 100,
	.regul = CTRL_REGUL_POSE_DIST,
	.allow_reverse = TRUE,
	.mode = &controller_modes[CTRL_STATE_INGAME],
};

/* This global object contains all numerical logs references (vectors, etc.) */
datalog_t datalog;

static void mach_post_ctrl_loop_func(void)
{
#ifdef CONFIG_ANALOG_SENSORS
    /* TODO: To activate when included in RIOT */
	//analog_sensor_refresh_all(&ana_sensors);
#endif
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
#define qdec_setup(...)
#define msched_init(...)
#define log_vect_init(...)
#define kos_run(...)

uint8_t mach_is_game_launched(void)
{
	/* Starter switch */
	return 1;//return gpio_get_input(&PORTF, PIN3_bp);
}

uint8_t mach_is_camp_yellow(void)
{
	/* Color switch for coords translations */
	return gpio_get_input(&PORTF, PIN2_bp);
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

#ifdef CONFIG_ANALOG_SENSORS
	/* setup analog conversion */
    /* TODO: To activate when included in RIOT */
	/* analog_sensor_setup(&ana_sensors); */
#endif

#if defined(CONFIG_SD21)
	/* setup TWI communication with SD21 */
	sd21_setup(&sd21);
#endif /* CONFIG_SD21 */

	//action_setup(); /* TODO: commenter pour debug */

	hbridge_setup(&hbridges);

	/* setup qdec */
	qdec_setup(&encoders[0]);
	qdec_setup(&encoders[1]);

	/* controller setup */
	odometry_setup(WHEELS_DISTANCE);

#if defined(__AVR__)
	/* Programmable Multilevel Interrupt Controller */
	PMIC.CTRL |= PMIC_LOLVLEN_bm; /* Low-level Interrupt Enable */

	/* global interrupt enable */
	sei();
#endif

	log_vect_init(&datalog, NULL, /*400,*/
			COL_INT16, "left_speed",
			COL_INT16, "right_speed",
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
	//FIXME! robot_speed = encoder_read();

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

