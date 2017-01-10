/**
 * @file vicon_main.cpp
 *
 * @author Hui Xie
 */

#include <fcntl.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <poll.h>

#include <px4_posix.h>

#include <platforms/px4_defines.h>


#include <drivers/drv_hrt.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_image_attitude_setpoint.h>

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_pos_disturb_main(int argc, char *argv[]);

const float PI = 3.14159f;

class ViconPosControl:public control::SuperBlock
{
public:
    /* Constructor */
    ViconPosControl();
    /* Destructor */
    virtual ~ViconPosControl();
    int start();
    void status();
//    void update();
private:
    bool _task_should_exit;
    int _control_task;
    static void task_main_trampoline(int argc, char *argv[]);

    /* Subscription descriptors */
    int _local_pos_sub;
    int _vehicle_status_sub;
    int _params_sub;
    int _arming_sub;
    int _ctrl_state_sub;
    int	_manual_sub;
    int _control_mode_sub;
    int _vehicle_land_detected_sub;
    int _vehicle_img_att_sp_sub;

    /* Subscribe structures */
    struct vehicle_status_s _vehicle_status;
    struct vehicle_local_position_s _local_pos;
    struct actuator_armed_s _arming;
    struct control_state_s _ctrl_state;
    struct manual_control_setpoint_s _manual;
    struct vehicle_control_mode_s _control_mode;
    struct vehicle_land_detected_s _vehicle_land_detected;
    struct vehicle_image_attitude_setpoint_s _img_att_sp;



    void poll_subscriptions();

    struct{
        param_t thr_min;
        param_t thr_max;
        param_t thr_hover;

        param_t z_p;
        param_t z_vel_p;
        param_t z_vel_i;
        param_t z_vel_d;
        param_t z_vel_max_up;
        param_t z_vel_max_down;
        param_t z_ff;

        param_t xy_p;
        param_t xy_vel_p;
        param_t xy_vel_i;
        param_t xy_vel_d;
        param_t xy_vel_max;
        param_t xy_ff;

        param_t man_roll_max;
        param_t man_pitch_max;
        param_t man_yaw_max;
        param_t global_yaw_max;
        param_t mc_att_yaw_p;

        param_t acc_hor_max;
        param_t tilt_max_air;

        param_t ibvs_enable;
    } _params_handles;

    struct{
        float thr_min;
        float thr_max;
        float thr_hover;
        float vel_max_up;
        float vel_max_down;

        float man_roll_max;
        float man_pitch_max;
        float man_yaw_max;
        float global_yaw_max;
        float mc_att_yaw_p;

        math::Vector<3> pos_p;
        math::Vector<3> vel_p;
        math::Vector<3> vel_i;
        math::Vector<3> vel_d;
        math::Vector<3> vel_ff;
        math::Vector<3> vel_max;

        float acc_hor_max;
        float tilt_max_air;
        uint8_t ibvs_enable;
    } _params;

    int parameters_update(bool force);

    /* publication topics */
    orb_advert_t _mavlink_log_pub;
    orb_advert_t _att_sp_pub;
    orb_advert_t _local_pos_sp_pub;


    /*Publish structure*/
    struct vehicle_attitude_setpoint_s _att_sp;
    struct vehicle_local_position_setpoint_s _local_pos_sp;


    control::BlockParamFloat _manual_thr_min;
    control::BlockParamFloat _manual_thr_max;

    control::BlockDerivative _vel_x_deriv;
    control::BlockDerivative _vel_y_deriv;
    control::BlockDerivative _vel_z_deriv;

    control::BlockDerivative _pos_x_deriv;
    control::BlockDerivative _pos_y_deriv;
    control::BlockDerivative _pos_z_deriv;


    math::Matrix<3, 3> _R;
    math::Vector<3> _pos;
    math::Vector<3> _pos_sp;
    math::Vector<3> _vel;
    math::Vector<3> _vel_prev;
    math::Vector<3> _vel_sp;
    math::Vector<3> _acc_sp;
    math::Vector<3> _vel_sp_prev;
    math::Vector<3> _vel_ff;
//    math::Vector<3> _vel_err_d;
    math::Vector<3> _pos_err_d;
    math::Vector<3> _thrust_int;


    float _yaw;
    void automatic_3(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
    void automatic_disturb_rej(float dt);
    float sign_f(float var);

    static float throttle_curve(float ctl, float ctr);
    void task_main();
    void manual(bool &reset_yaw_sp, float dt);
};

namespace Vicon_Control
{
    ViconPosControl *g_control;
}

ViconPosControl::ViconPosControl():
    SuperBlock(NULL, "VCN"),
    _task_should_exit(false),
    _control_task(-1),

    /* subscriptions */
    _local_pos_sub(-1),
    _vehicle_status_sub(-1),
    _params_sub(-1),
    _arming_sub(-1),
    _ctrl_state_sub(-1),
    _manual_sub(-1),
    _control_mode_sub(-1),
    _vehicle_land_detected_sub(-1),
    _vehicle_img_att_sp_sub(-1),
    /*subscribed structure*/
    _vehicle_status{},
    _local_pos{},
    _arming{},
    _ctrl_state{},
    _manual{},
    _control_mode{},
    _vehicle_land_detected{},
    _img_att_sp{},
    /*publication*/
    _mavlink_log_pub(nullptr),
    _att_sp_pub(nullptr),
    _local_pos_sp_pub(nullptr),
    /*Publish structure*/
    _att_sp{},
    _local_pos_sp{},

    _manual_thr_min(this, "MANTHR_MIN"),
    _manual_thr_max(this, "MANTHR_MAX"),
    _vel_x_deriv(this, "VELD"),
    _vel_y_deriv(this, "VELD"),
    _vel_z_deriv(this, "VELD"),

    _pos_x_deriv(this, "VELD"),
    _pos_y_deriv(this, "VELD"),
    _pos_z_deriv(this, "VELD"),
    _yaw(0.0f)
{
    // Make the quaternion valid for control state
    _ctrl_state.q[0] = 1.0f;


    _params.pos_p.zero();
    _params.vel_p.zero();
    _params.vel_i.zero();
    _params.vel_d.zero();
    _params.vel_max.zero();
    _params.vel_ff.zero();

    _R.identity();
    _pos.zero();
    _pos_sp.zero();
    _vel.zero();
    _vel_prev.zero();
    _vel_sp.zero();
    _acc_sp.zero();
    _vel_sp_prev.zero();
    _vel_ff.zero();
//    _vel_err_d.zero();
    _pos_err_d.zero();
    _thrust_int.zero();


    _params_handles.thr_min		= param_find("VCN_THR_MIN");
    _params_handles.thr_max		= param_find("VCN_THR_MAX");
    _params_handles.thr_hover	= param_find("VCN_THR_HOVER");

    _params_handles.z_p		= param_find("VCN_Z_P");
    _params_handles.z_vel_p		= param_find("VCN_Z_VEL_P");
    _params_handles.z_vel_i		= param_find("VCN_Z_VEL_I");
    _params_handles.z_vel_d		= param_find("VCN_Z_VEL_D");
    _params_handles.z_vel_max_up	= param_find("VCN_Z_VEL_MAX_UP");
    _params_handles.z_vel_max_down	= param_find("VCN_Z_VEL_MAX");
    _params_handles.z_ff		= param_find("VCN_Z_FF");

    _params_handles.xy_p		= param_find("VCN_XY_P");
    _params_handles.xy_vel_p	= param_find("VCN_XY_VEL_P");
    _params_handles.xy_vel_i	= param_find("VCN_XY_VEL_I");
    _params_handles.xy_vel_d	= param_find("VCN_XY_VEL_D");
    _params_handles.xy_vel_max	= param_find("VCN_XY_VEL_MAX");
    _params_handles.xy_ff		= param_find("VCN_XY_FF");

    _params_handles.man_roll_max = param_find("VCN_MAN_R_MAX");
    _params_handles.man_pitch_max = param_find("VCN_MAN_P_MAX");
    _params_handles.man_yaw_max = param_find("VCN_MAN_Y_MAX");
    _params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
    _params_handles.mc_att_yaw_p = param_find("MC_YAW_P");

    _params_handles.acc_hor_max = param_find("VCN_ACC_HOR_MAX");
    _params_handles.tilt_max_air = param_find("VCN_TILTMAX_AIR");
    _params_handles.ibvs_enable = param_find("IBVS_ENABLE");


    /* fetch initial parameter values */
    parameters_update(true);
}


ViconPosControl::~ViconPosControl(){
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }
    Vicon_Control::g_control = nullptr;
}

void ViconPosControl::status(){
   warnx("Refference: Roll:%.5f Pitch:%.5f Yaw:%.5f Thrust:%.5f",
         (double)_att_sp.roll_body, (double)_att_sp.pitch_body,(double)_att_sp.yaw_body,(double)_att_sp.thrust);
}

float ViconPosControl::throttle_curve(float ctl, float ctr)
{
    /* piecewise linear mapping: 0:ctr -> 0:0.5
     * and ctr:1 -> 0.5:1 */
    if (ctl < 0.5f) {
        return 2 * ctl * ctr;

    } else {
//        return ctr + 2.0 * (ctl - 0.5f) * (1.0f - ctr);
        return ctr + 1.6f * (ctl - 0.5f) * (1.0f - ctr);
    }
}

void ViconPosControl::poll_subscriptions()
{
    bool updated;

    orb_check(_vehicle_status_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
    }

    orb_check(_local_pos_sub,&updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }

    orb_check(_arming_sub,&updated);
    if(updated){
        orb_copy(ORB_ID(actuator_armed),_arming_sub,&_arming);
    }

    orb_check(_ctrl_state_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

        /* get current rotation matrix and euler angles from control state quaternions */
        math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
        _R = q_att.to_dcm();
        math::Vector<3> euler_angles;
        euler_angles = _R.to_euler();
        _yaw = euler_angles(2);
    }

    orb_check(_manual_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
    }

    orb_check(_control_mode_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
    }

    orb_check(_vehicle_land_detected_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
    }

    orb_check(_vehicle_img_att_sp_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_image_attitude_setpoint),_vehicle_img_att_sp_sub,&_img_att_sp);
    }
}

int ViconPosControl::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(_params_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
    }

    if (updated || force) {

        param_get(_params_handles.thr_min, &_params.thr_min);
        param_get(_params_handles.thr_max, &_params.thr_max);
        param_get(_params_handles.thr_hover, &_params.thr_hover);

        float v;
        param_get(_params_handles.xy_p, &v);
        _params.pos_p(0) = v;
        _params.pos_p(1) = v;
        param_get(_params_handles.z_p, &v);
        _params.pos_p(2) = v;
        param_get(_params_handles.xy_vel_p, &v);
        _params.vel_p(0) = v;
        _params.vel_p(1) = v;
        param_get(_params_handles.z_vel_p, &v);
        _params.vel_p(2) = v;
        param_get(_params_handles.xy_vel_i, &v);
        _params.vel_i(0) = v;
        _params.vel_i(1) = v;
        param_get(_params_handles.z_vel_i, &v);
        _params.vel_i(2) = v;
        param_get(_params_handles.xy_vel_d, &v);
        _params.vel_d(0) = v;
        _params.vel_d(1) = v;
        param_get(_params_handles.z_vel_d, &v);
        _params.vel_d(2) = v;
        param_get(_params_handles.xy_vel_max, &v);
        _params.vel_max(0) = v;
        _params.vel_max(1) = v;
        param_get(_params_handles.z_vel_max_up, &v);
        _params.vel_max_up = v;
        _params.vel_max(2) = v;
        param_get(_params_handles.z_vel_max_down, &v);
        _params.vel_max_down = v;

        param_get(_params_handles.xy_ff, &v);
        v = math::constrain(v, 0.0f, 1.0f);
        _params.vel_ff(0) = v;
        _params.vel_ff(1) = v;
        param_get(_params_handles.z_ff, &v);
        v = math::constrain(v, 0.0f, 1.0f);
        _params.vel_ff(2) = v;

        param_get(_params_handles.man_roll_max, &_params.man_roll_max);
        param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
        param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
        param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
        _params.man_roll_max = math::radians(_params.man_roll_max);
        _params.man_pitch_max = math::radians(_params.man_pitch_max);
        _params.man_yaw_max = math::radians(_params.man_yaw_max);
        _params.global_yaw_max = math::radians(_params.global_yaw_max);
        param_get(_params_handles.mc_att_yaw_p, &_params.mc_att_yaw_p);
        param_get(_params_handles.acc_hor_max, &_params.acc_hor_max);

        param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
        _params.tilt_max_air = math::radians(_params.tilt_max_air);

        param_get(_params_handles.ibvs_enable,&_params.ibvs_enable);
        warnx("param updated: _params.ibvs_enable: %d", _params.ibvs_enable);
    }

    return OK;
}

void ViconPosControl::automatic_3(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual){
//    cal_vel_sp(dt,reset_int_xy,reset_int_z,reset_int_z_manual);
    // reset integrals
    if (reset_int_z) {
        reset_int_z = false;
        float i = _params.thr_min;

        if (reset_int_z_manual) {
            warnx("reset_int_z_manual");
            i = _params.thr_hover;

            if (i < _params.thr_min) {
                i = _params.thr_min;

            } else if (i > _params.thr_max) {
                i = _params.thr_max;
            }
        }

        _thrust_int(2) = -i;
    }
    if (reset_int_xy) {
        reset_int_xy = false;
        _thrust_int(0) = 0.0f;
        _thrust_int(1) = 0.0f;
    }

    math::Vector<3> pos_err = _pos_sp - _pos;
    math::Vector<3> force_nav;
    math::Vector<3> force_body;

    force_nav = pos_err.emult(_params.vel_p) + _pos_err_d.emult(_params.vel_d)+_thrust_int;
//    math::Vector<3> vel_err = _vel_sp - _vel;
//    force_nav = pos_err.emult(_params.vel_p) + vel_err.emult(_params.vel_d) + _thrust_int;

    float roll_sp, pitch_sp, thrust_sp;

    force_body(0) = cosf(-_yaw)*force_nav(0) - sinf(-_yaw)*force_nav(1);
    force_body(1) = sinf(-_yaw)*force_nav(0) + cosf(-_yaw)*force_nav(1);
    force_body(2) = force_nav(2);
    /* limit thrust vector and check for saturation */


    bool saturation_x = false;
    bool saturation_y = false;
    bool saturation_z = false;

    float thr_min = _params.thr_min;
    float tilt_max = _params.tilt_max_air;
    float thr_max = _params.thr_max;

    pitch_sp = - force_body(0);
    roll_sp = force_body(1);
    thrust_sp = - force_body(2);

//    warnx("CurPos: %.4f PosSP %.4f, Thrust SP: %.4f",(double)_pos(2),(double) _pos_sp(2),(double) thrust_sp);
    if(pitch_sp>tilt_max)
    {
        pitch_sp = tilt_max;
        saturation_x = true;
    }
    else if(pitch_sp < - tilt_max)
    {
        pitch_sp = - tilt_max;
        saturation_x = true;
    }

    if(roll_sp>tilt_max)
    {
        roll_sp = tilt_max;
        saturation_y = true;
    }
    else if(roll_sp < - tilt_max)
    {
        roll_sp = - tilt_max;
        saturation_y = true;
    }

    if(thrust_sp<thr_min)
    {
        thrust_sp = thr_min;
        saturation_z = true;
    }
    if(thrust_sp>thr_max)
    {
        thrust_sp = thr_max;
        saturation_z =  true;
    }

    /* update integrals */
    if (!saturation_x) {
        _thrust_int(0) += pos_err(0) * _params.vel_i(0) * dt;
    }

    /* update integrals */
    if (!saturation_y) {
        _thrust_int(0) += pos_err(0) * _params.vel_i(0) * dt;
    }

    if (!saturation_z) {
        _thrust_int(2) += pos_err(2) * _params.vel_i(2) * dt;

        /* protection against flipping on ground when landing */
        if (_thrust_int(2) > 0.0f) {
            _thrust_int(2) = 0.0f;
        }
    }
    _att_sp.roll_body = roll_sp;
    _att_sp.pitch_body = pitch_sp;
    _att_sp.yaw_body = _wrap_pi(_att_sp.yaw_body);
    _att_sp.thrust = thrust_sp;
}

void ViconPosControl::automatic_disturb_rej(float dt){

    //TODO add those constant to parameters
    const float K1_n = 0.1f;
    const float K2_n = 0.1f;
    const float epsilon1_n = 0.1f;
    const float epsilon2_n = 0.1f;
    const float L1_n = 0.1f;
    const float L2_n = 0.1f;
    //TODO add state to the class
    float hatd_n = 0.0f;
    float z2_n = 0.0f;
    float rhos21_n = 0.0f;
    float rhos22_n = 0.0f;
    float Isd2_n = 0.0f;


    float hatd_n_dot;
    float z2_n_dot;
    float rhos21_n_dot;
    float rhos22_n_dot;
    float Isd2_n_dot;

    math::Vector<3> force_nav;


    float e1_n = _pos(0) - _pos_sp(0);
//    float e1_e = _pos(1) - _pos_sp(1);
//    float e1_d = _pos(2) - _pos_sp(2);

    float eta2c_n       = _vel_sp(0) - K1_n*e1_n;
    float e2_n          = _vel(0) -  eta2c_n;
    float eta2cdot_n    = _acc_sp(0) - K1_n*(e2_n + eta2c_n - _vel_sp(0));

    force_nav(0) = -K2_n*e2_n - hatd_n - e1_n + eta2cdot_n;


    float s2_n = z2_n - _vel(0);

    z2_n_dot = force_nav(0)+hatd_n;

    float rhos21_minus_s2 = rhos21_n - s2_n;
    float sqrt_temp = sqrt(fabs(rhos21_minus_s2));
//    rhos21_n_dot = rhos22_n - epsilon1_n*sqrt(fabs(rhos21_minus_s2))*sign_f(rhos21_minus_s2);
    rhos21_n_dot = rhos22_n - epsilon1_n*sqrt_temp*sign_f(rhos21_minus_s2);
    rhos22_n_dot = -epsilon2_n*sign_f(rhos21_minus_s2);
    float sd2_n = s2_n + rhos21_n;
    sqrt_temp = sqrt(fabs(sd2_n));
    hatd_n_dot = -rhos21_n_dot - L1_n*sqrt_temp*sign_f(sd2_n) - L2_n*Isd2_n;
    Isd2_n_dot = sign_f(sd2_n);

    z2_n += z2_n_dot*dt;
    rhos21_n += rhos21_n_dot*dt;
    rhos22_n += rhos22_n_dot*dt;
    hatd_n += hatd_n_dot*dt;
    Isd2_n += Isd2_n_dot*dt;

    _att_sp.roll_body = force_nav(0);

}

float ViconPosControl::sign_f(float var){
    float sign;
    if(var>0.0f)
        sign = 1.0f;
    else
        sign = -1.0f;
    return sign;
}


void ViconPosControl::manual(bool &reset_yaw_sp, float dt){
    /* reset yaw setpoint to current position if needed */
    if (reset_yaw_sp) {
        reset_yaw_sp = false;
        _att_sp.yaw_body = _yaw;
    }
    else if (!_vehicle_land_detected.landed &&_manual.z > 0.1f)
    {
    }

    _att_sp.roll_body = _manual.y * _params.man_roll_max;
    _att_sp.pitch_body = -_manual.x * _params.man_pitch_max;
    float thr_val = throttle_curve(_manual.z, _params.thr_hover);
    _att_sp.thrust = math::min(thr_val, _manual_thr_max.get());
    /* enforce minimum throttle if not landed */
     if (!_vehicle_land_detected.landed) {
         _att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
     }

    math::Matrix<3, 3> R_sp;

    // construct attitude setpoint rotation matrix. modify the setpoints for roll
    // and pitch such that they reflect the user's intention even if a yaw error
    // (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
    // from the pure euler angle setpoints will lead to unexpected attitude behaviour from
    // the user's view as the euler angle sequence uses the  yaw setpoint and not the current
    // heading of the vehicle.

    // calculate our current yaw error
    float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);
    // compute the vector obtained by rotating a z unit vector by the rotation
    // given by the roll and pitch commands of the user
    math::Vector<3> zB = {0, 0, 1};
    math::Matrix<3,3> R_sp_roll_pitch;
    R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
    math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;
    // transform the vector into a new frame which is rotated around the z axis
    // by the current yaw error. this vector defines the desired tilt when we look
    // into the direction of the desired heading
    math::Matrix<3,3> R_yaw_correction;
    R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
    z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;
    // use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
    // to calculate the new desired roll and pitch angles
    // R_tilt can be written as a function of the new desired roll and pitch
    // angles. we get three equations and have to solve for 2 unknowns
    float pitch_new = asinf(z_roll_pitch_sp(0));
    float roll_new = -atan2f(z_roll_pitch_sp(1), z_roll_pitch_sp(2));
    R_sp.from_euler(roll_new, pitch_new, _att_sp.yaw_body);
    memcpy(&_att_sp.R_body[0], R_sp.data, sizeof(_att_sp.R_body));
    /* copy quaternion setpoint to attitude setpoint topic */
    math::Quaternion q_sp;
    q_sp.from_dcm(R_sp);
    memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));
    _att_sp.timestamp = hrt_absolute_time();
}

void ViconPosControl::task_main(){

    /*Do subscription */
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _arming_sub = orb_subscribe(ORB_ID(actuator_armed));
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    _vehicle_img_att_sp_sub = orb_subscribe(ORB_ID(vehicle_image_attitude_setpoint));

    parameters_update(true);
    /* initialize values of critical structs until first regular update */
    _arming.armed = false;

    poll_subscriptions();

    bool reset_yaw_sp = true;
    bool reset_int_z = true;
    bool reset_int_z_manual = false;
    bool reset_int_xy = true;

    bool was_armed = false;

    hrt_abstime t_prev = hrt_absolute_time();

    hrt_abstime t0_circle = t_prev;
    math::Vector<2> p0_circle;
    const float radius_circle = 1.0f;
    const float omega_circle = 2.0f*PI/20.0f;


    px4_pollfd_struct_t fds[1];
    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;

    uint8_t prev_nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

    while (!_task_should_exit)
    {
        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }
        /* this is undesirable but not much we can do */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

        poll_subscriptions();
        parameters_update(false);

        hrt_abstime t = hrt_absolute_time();
        float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
        t_prev = t;
        setDt(dt);


        /* Update velocity derivative,
         * independent of the current flight mode
         */
        if (_local_pos.timestamp > 0) {
            if (PX4_ISFINITE(_local_pos.x) &&
                    PX4_ISFINITE(_local_pos.y) &&
                    PX4_ISFINITE(_local_pos.z)) {
                _pos(0) = _local_pos.x;
                _pos(1) = _local_pos.y;
                _pos(2) = _local_pos.z;
            }

            if (PX4_ISFINITE(_local_pos.vx) &&
                    PX4_ISFINITE(_local_pos.vy) &&
                    PX4_ISFINITE(_local_pos.vz)) {
                _vel(0) = _local_pos.vx;
                _vel(1) = _local_pos.vy;
                _vel(2) = _local_pos.vz;
            }
        }

        if (_control_mode.flag_armed && !was_armed) {
            reset_int_z = true;
            reset_int_xy = true;
            reset_yaw_sp = true;
            _pos_sp(0) = _pos(0);
            _pos_sp(1) = _pos(1);
            _pos_sp(2) = _pos(2);
        }
        //Update previous arming state
        was_armed = _control_mode.flag_armed;
        _att_sp.disable_mc_yaw_control = false;

        if((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)||
                (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)||
                (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER))
        {
            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
            {
                if(prev_nav_state!=vehicle_status_s::NAVIGATION_STATE_ALTCTL)
                {
                    _att_sp.yaw_body = _yaw;
                    _pos_sp(0) = _pos(0);
                    _pos_sp(1) = _pos(1);
                    _vel_sp(0) = 0.0f;
                    _vel_sp(1) = 0.0f;
                    _acc_sp(0) = 0.0f;
                    _acc_sp(1) = 0.0f;
                }
            }

            if((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)||
                    (_vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER))
            {
//                warnx("3D automatic control");
                if((prev_nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)&&
                        (prev_nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER))
                {
                    _pos_sp(2) = _pos(2);
                    p0_circle(0) = _pos(0);
                    p0_circle(1) = _pos(1);
                    t0_circle = hrt_absolute_time();
                    reset_int_z = true;
                    reset_int_xy = true;
                    if(_control_mode.flag_armed)
                        reset_int_z_manual = true;
                }
                hrt_abstime time = hrt_absolute_time();
                float dt0 = (time - t0_circle) * 0.000001f;
                _pos_sp(0) = p0_circle(0) + radius_circle*sinf(omega_circle*dt0);
                _pos_sp(1) = p0_circle(1) + radius_circle*(1.0f-cosf(omega_circle*dt0));
                _vel_sp(0) = radius_circle*omega_circle*cosf(omega_circle*dt0);
                _vel_sp(1) = radius_circle*omega_circle*sinf(omega_circle*dt0);
                _acc_sp(0) = -radius_circle*omega_circle*omega_circle*sinf(omega_circle*dt0);
                _acc_sp(1) = radius_circle*omega_circle*omega_circle*cosf(omega_circle*dt0);

            }
            if (_local_pos.timestamp > 0) {
                _pos_err_d(0) = _pos_x_deriv.update(_pos_sp(0)-_pos(0));
                _pos_err_d(1) = _pos_y_deriv.update(_pos_sp(1)-_pos(1));
                _pos_err_d(2) = _pos_z_deriv.update(_pos_sp(2)-_pos(2));
            }

            automatic_3(dt,reset_int_xy,reset_int_z,reset_int_z_manual);

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
            {
                float thr_val = throttle_curve(_manual.z, _params.thr_hover);
                _att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

                if (!_vehicle_land_detected.landed) {
                    _att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
                }
            }

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
                automatic_disturb_rej(dt);

            _att_sp.timestamp = hrt_absolute_time();
            math::Matrix<3,3> R_sp;
            R_sp.from_euler(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
            memcpy(&_att_sp.R_body[0], R_sp.data, sizeof(_att_sp.R_body));
            _att_sp.R_valid = true;

            /* copy quaternion setpoint to attitude setpoint topic */
            math::Quaternion q_sp;
            q_sp.from_dcm(R_sp);
            memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));


            /* fill local position, velocity and thrust setpoint */
            _local_pos_sp.timestamp = hrt_absolute_time();
            _local_pos_sp.x = _pos_sp(0);
            _local_pos_sp.y = _pos_sp(1);
            _local_pos_sp.z = _pos_sp(2);
            _local_pos_sp.yaw = _att_sp.yaw_body;
            _local_pos_sp.vx = _vel_sp(0);
            _local_pos_sp.vy = _vel_sp(1);
            _local_pos_sp.vz = _vel_sp(2);

            /* publish local position setpoint */
            if (_local_pos_sp_pub != nullptr) {
                orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

            } else {
                _local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
            }
        }

        /* generate attitude setpoint from manual controls */
        if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
//            warnx("manual mode");
            manual(reset_yaw_sp,dt);
        } else {
            //TODO check whether need to reset yaw sp
            reset_yaw_sp = true;
        }



        /* update previous velocity for velocity controller D part */
        _vel_prev = _vel;

        if(_att_sp_pub != nullptr){
            orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);
        }
        else{
            _att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint),&_att_sp);
        }

        prev_nav_state = _vehicle_status.nav_state;
    }

    _control_task = -1;
}

void ViconPosControl::task_main_trampoline(int argc, char *argv[])
{
    Vicon_Control::g_control->task_main();
}

int ViconPosControl::start(){
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("mc_pos_disturb",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1900,
                       (px4_main_t)&ViconPosControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

int mc_pos_disturb_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: mc_pos_disturb {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (Vicon_Control::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        Vicon_Control::g_control = new ViconPosControl;

        if (Vicon_Control::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != Vicon_Control::g_control->start()) {
            delete Vicon_Control::g_control;
            Vicon_Control::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (Vicon_Control::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete Vicon_Control::g_control;
        Vicon_Control::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (Vicon_Control::g_control) {
            Vicon_Control::g_control->status();

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
