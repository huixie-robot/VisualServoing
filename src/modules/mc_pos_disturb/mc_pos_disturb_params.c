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
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_EPS1, 0.12f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_EPS2, 0.12f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_K1, 0.12f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_K2, 0.12f);

 /**
 * Minimum thrust in auto thrust control
 *W
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_L1, 0.12f);

 /**
 * Minimum thrust in auto thrust control
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Position Control with Disturbance
 */
PARAM_DEFINE_FLOAT(MCD_NE_L2, 0.12f);



