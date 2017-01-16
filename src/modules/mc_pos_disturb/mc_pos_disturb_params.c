/**
 * @file mc_pos_disturb_params.c
 * Multicopter motion controller with disturbance rejection.
 *
 * @author Hui Xie <hui.xie.sheffield@gmail.com>
 */
 
 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_EPS1, 5.00f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 20.00
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_EPS2, 0.50f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.02
 * @max 30.0
 * @decimal 3
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_K1, 0.119f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.02
 * @max 30.0
 * @decimal 3
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_K2, 1.681f);

 /**
 * Minimum thrust in auto thrust control
 *W
 * @unit norm
 * @min 0.02
 * @max 30.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_L1, 10.0f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 30.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_L2, 10.0f);




/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_MANTHR_MIN, 0.08f);

/**
 * Maximum manual thrust
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_MANTHR_MAX, 0.8f);


/**
 * Minimum thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_THR_MIN, 0.12f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to ALTCTL mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * @unit norm
 * @min 0.2
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_THR_HOVER, 0.5f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust. Setting a value of one can put
 * the system into actuator saturation as no spread between
 * the motors is possible any more. A value of 0.8 - 0.9
 * is recommended.
 *
 * @unit norm
 * @min 0.0
 * @max 0.95
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_THR_MAX, 0.6f);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @max 1.5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_P, 0.1f);

/**
 * Proportional gain for vertical velocity error
 *
 * @min 0.1
 * @max 0.4
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_P, 0.1f);

/**
 * Integral gain for vertical velocity error
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.01
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_I, 0.02f);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_D, 0.2f);

/**
 * Maximum vertical ascent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_MAX_UP, 0.5f);

/**
 * Maximum vertical descent velocity
 *
 * Maximum vertical velocity in AUTO mode and endpoint for stabilized modes (ALTCTRL, POSCTRL).
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_MAX, 1.0f);

/**
 * Transitional support, do not change / use
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_VEL_MAX_DN, 1.0f);

/**
 * Vertical velocity feed forward
 *
 * Feed forward weight for altitude control in stabilized modes (ALTCTRL, POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_Z_FF, 0.5f);


/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_P, 0.11f);

/**
 * Proportional gain for horizontal velocity error
 *
 * @min 0.06
 * @max 0.15
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_VEL_P, 0.12f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_VEL_I, 0.015f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.005
 * @max 0.3
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_VEL_D, 0.18f);


/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_VEL_MAX, 1.0f);

/**
 * Horizontal velocity feed forward
 *
 * Feed forward weight for position control in position control mode (POSCTRL). 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_XY_FF, 0.5f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_VELD_LP, 5.0f);

/**
 * Max manual roll
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_MAN_R_MAX, 20.0f);

/**
 * Max manual pitch
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_MAN_P_MAX, 20.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_MAN_Y_MAX, 200.0f);

/**
 * Maximum horizonal acceleration in velocity controlled modes
 *
 * @unit m/s/s
 * @min 0.5
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_ACC_HOR_MAX, 1.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_TILTMAX_AIR, 12.0f);

/**
 * Reference Trajectory Radius
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit meter
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_CIR_RAD, 1.0f);

/**
 * Reference Trajectory Radius
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit rad per second
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MCD_CIR_OMG, 0.314159f);




