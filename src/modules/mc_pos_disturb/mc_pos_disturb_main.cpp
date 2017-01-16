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
#include <uORB/topics/ibvs_state.h>

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

    /* Subscribe structures */
    struct vehicle_status_s _vehicle_status;
    struct vehicle_local_position_s _local_pos;
    struct actuator_armed_s _arming;
    struct control_state_s _ctrl_state;
    struct manual_control_setpoint_s _manual;
    struct vehicle_control_mode_s _control_mode;
    struct vehicle_land_detected_s _vehicle_land_detected;


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

//        param_t ibvs_enable;

        param_t K1_ne;
        param_t K2_ne;
        param_t epsilon1_ne;
        param_t epsilon2_ne;
        param_t L1_ne;
        param_t L2_ne;

        param_t radius_circle;
        param_t omega_circle;

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
//        uint8_t ibvs_enable;

        float K1_ne;
        float K2_ne;
        float epsilon1_ne;
        float epsilon2_ne;
        float L1_ne;
        float L2_ne;

        float radius_circle;
        float omega_circle;
    } _params;

    int parameters_update(bool force);

    /* publication topics */
    orb_advert_t    _att_sp_pub;
    orb_advert_t    _local_pos_sp_pub;
    orb_advert_t    _att_sp_ibvs_pub_fd;
    orb_advert_t    _ibvs_state_pub_fd;



    /*Publish structure*/
    struct vehicle_attitude_setpoint_s _att_sp;
    struct vehicle_local_position_setpoint_s _local_pos_sp;
    struct vehicle_image_attitude_setpoint_s _img_att_sp;
    struct ibvs_state_s _ibvs_state;


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
    math::Vector<3> _vel_sp;
    math::Vector<3> _acc_sp;
    math::Vector<3> _vel_sp_prev;
    math::Vector<3> _pos_err_d;
    math::Vector<3> _thrust_int;

    float hatd_n;
    float z2_n;
    float rhos21_n;
    float rhos22_n;
    float Isd2_n;

    float hatd_e;
    float z2_e;
    float rhos21_e;
    float rhos22_e;
    float Isd2_e;

//    float hatd_d;
//    float z2_d;
//    float rhos21_d;
//    float rhos22_d;
//    float Isd2_d;


    float _roll;
    float _pitch;
    float _yaw;
    void automatic_3(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
    void automatic_disturb_rej(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
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
    SuperBlock(NULL, "MCD"),
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
    /*subscribed structure*/
    _vehicle_status{},
    _local_pos{},
    _arming{},
    _ctrl_state{},
    _manual{},
    _control_mode{},
    _vehicle_land_detected{},
    /*publication*/
    _att_sp_pub(nullptr),
    _local_pos_sp_pub(nullptr),
    _att_sp_ibvs_pub_fd(nullptr),
    _ibvs_state_pub_fd(nullptr),
    /*Publish structure*/
    _att_sp{},
    _local_pos_sp{},
    _img_att_sp{},
    _ibvs_state{},

    _manual_thr_min(this, "MANTHR_MIN"),
    _manual_thr_max(this, "MANTHR_MAX"),
    _vel_x_deriv(this, "VELD"),
    _vel_y_deriv(this, "VELD"),
    _vel_z_deriv(this, "VELD"),
    _pos_x_deriv(this, "VELD"),
    _pos_y_deriv(this, "VELD"),
    _pos_z_deriv(this, "VELD"),

    hatd_n(0.0f),
    z2_n(0.0f),
    rhos21_n(0.0f),
    rhos22_n(0.0f),
    Isd2_n(0.0f),

    hatd_e(0.0f),
    z2_e(0.0f),
    rhos21_e(0.0f),
    rhos22_e(0.0f),
    Isd2_e(0.0f),

//    hatd_d(0.0f),
//    z2_d(0.0f),
//    rhos21_d(0.0f),
//    rhos22_d(0.0f),
//    Isd2_d(0.0f),

    _roll(0.0f),
    _pitch(0.0f),
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
    _vel_sp.zero();
    _acc_sp.zero();
    _vel_sp_prev.zero();
//    _vel_err_d.zero();
    _pos_err_d.zero();
    _thrust_int.zero();


    _params_handles.thr_min		= param_find("MCD_THR_MIN");
    _params_handles.thr_max		= param_find("MCD_THR_MAX");
    _params_handles.thr_hover	= param_find("MCD_THR_HOVER");

    _params_handles.z_p		= param_find("MCD_Z_P");
    _params_handles.z_vel_p		= param_find("MCD_Z_VEL_P");
    _params_handles.z_vel_i		= param_find("MCD_Z_VEL_I");
    _params_handles.z_vel_d		= param_find("MCD_Z_VEL_D");
    _params_handles.z_vel_max_up	= param_find("MCD_Z_VEL_MAX_UP");
    _params_handles.z_vel_max_down	= param_find("MCD_Z_VEL_MAX");
    _params_handles.z_ff		= param_find("MCD_Z_FF");

    _params_handles.xy_p		= param_find("MCD_XY_P");
    _params_handles.xy_vel_p	= param_find("MCD_XY_VEL_P");
    _params_handles.xy_vel_i	= param_find("MCD_XY_VEL_I");
    _params_handles.xy_vel_d	= param_find("MCD_XY_VEL_D");
    _params_handles.xy_vel_max	= param_find("MCD_XY_VEL_MAX");
    _params_handles.xy_ff		= param_find("MCD_XY_FF");

    _params_handles.man_roll_max = param_find("MCD_MAN_R_MAX");
    _params_handles.man_pitch_max = param_find("MCD_MAN_P_MAX");
    _params_handles.man_yaw_max = param_find("MCD_MAN_Y_MAX");
    _params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
    _params_handles.mc_att_yaw_p = param_find("MC_YAW_P");

    _params_handles.acc_hor_max = param_find("MCD_ACC_HOR_MAX");
    _params_handles.tilt_max_air = param_find("MCD_TILTMAX_AIR");
//    _params_handles.ibvs_enable = param_find("IBVS_ENABLE");

    _params_handles.epsilon1_ne = param_find("MCD_NE_EPS1");
    _params_handles.epsilon2_ne = param_find("MCD_NE_EPS2");
    _params_handles.L1_ne = param_find("MCD_NE_L1");
    _params_handles.L2_ne = param_find("MCD_NE_L2");
    _params_handles.K1_ne = param_find("MCD_NE_K1");
    _params_handles.K2_ne = param_find("MCD_NE_K2");

    _params_handles.radius_circle = param_find("MCD_CIR_RAD");
    _params_handles.omega_circle = param_find("MCD_CIR_OMG");

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
        _roll = euler_angles(0);
        _pitch = euler_angles(1);
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

//        param_get(_params_handles.ibvs_enable,&_params.ibvs_enable);

        param_get(_params_handles.epsilon1_ne,&_params.epsilon1_ne);
        param_get(_params_handles.epsilon2_ne,&_params.epsilon2_ne);
        param_get(_params_handles.L1_ne,&_params.L1_ne);
        param_get(_params_handles.L2_ne,&_params.L2_ne);
        param_get(_params_handles.K1_ne,&_params.K1_ne);
        param_get(_params_handles.K2_ne,&_params.K2_ne);

        param_get(_params_handles.radius_circle,&_params.radius_circle);
        param_get(_params_handles.omega_circle,&_params.omega_circle);
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

//    force_nav = pos_err.emult(_params.vel_p) + _pos_err_d.emult(_params.vel_d)+_thrust_int;
    math::Vector<3> vel_err = _vel_sp - _vel;
    force_nav = pos_err.emult(_params.vel_p) + vel_err.emult(_params.vel_d) + _thrust_int;

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

void ViconPosControl::automatic_disturb_rej(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reeset_int_z_manual)
{
    const float K1_n = _params.K1_ne;
    const float K2_n = _params.K2_ne;
    const float epsilon1_n = _params.epsilon1_ne;
    const float epsilon2_n = _params.epsilon2_ne;
    const float L1_n = _params.L1_ne;
    const float L2_n = _params.L2_ne;

    const float K1_e = _params.K1_ne;
    const float K2_e = _params.K2_ne;
    const float epsilon1_e = _params.epsilon1_ne;
    const float epsilon2_e = _params.epsilon2_ne;
    const float L1_e = _params.L1_ne;
    const float L2_e = _params.L2_ne;

    /* limit thrust vector and check for saturation */
    bool saturation_x = false;
    bool saturation_y = false;

    float tilt_max = _params.tilt_max_air;

    float hatd_n_dot;
    float z2_n_dot;
    float rhos21_n_dot;
    float rhos22_n_dot;
    float Isd2_n_dot;

    float hatd_e_dot;
    float z2_e_dot;
    float rhos21_e_dot;
    float rhos22_e_dot;
    float Isd2_e_dot;

    if (reset_int_xy) {
        reset_int_xy = false;
        z2_n = 0.0f;
        rhos21_n = 0.0f;
        rhos22_n = 0.0f;
        hatd_n = 0.0f;
        Isd2_n = 0.0f;
        z2_e = 0.0f;
        rhos21_e = 0.0f;
        rhos22_e = 0.0f;
        hatd_e = 0.0f;
        Isd2_e = 0.0f;
    }

    math::Vector<3> force_nav;
    math::Vector<3> force_body;

    // North or X axis
    float e1_n          = _pos(0) - _pos_sp(0);
    float eta2c_n       = _vel_sp(0) - K1_n*e1_n;
    float e2_n          = _vel(0) -  eta2c_n;
    float eta2cdot_n    = _acc_sp(0) - K1_n*(e2_n + eta2c_n - _vel_sp(0));
    force_nav(0)        = (-K2_n*e2_n - hatd_n - e1_n + eta2cdot_n)/(20.0f*_att_sp.thrust);

    // East or Y axis
    float e1_e          = _pos(1) - _pos_sp(1);
    float eta2c_e       = _vel_sp(1) - K1_e*e1_e;
    float e2_e          = _vel(1) - eta2c_e;
    float eta2cdot_e    = _acc_sp(1) - K1_e*(e2_e + eta2c_e - _vel_sp(1));
    force_nav(1)        = (- K2_e*e2_e - hatd_e - e1_e + eta2cdot_e)/(20.0f*_att_sp.thrust);

    /*Avoid wind-up, thus stop integration for the disturbance term*/
    if(force_nav(0)>tilt_max)
    {
        force_nav(0) = tilt_max;
        saturation_x = true;
    }
    else if(force_nav(0)<-tilt_max)
    {
        force_nav(0) = -tilt_max;
        saturation_x = true;
    }

    if(force_nav(1)>tilt_max)
    {
        force_nav(1) = tilt_max;
        saturation_y = true;
    }
    else if(force_nav(1)<-tilt_max)
    {
        force_nav(1) = -tilt_max;
        saturation_y = true;
    }

    //    if(thrust_sp<thr_min)
    //    {
    //        thrust_sp = thr_min;
    //        saturation_z = true;
    //    }
    //    if(thrust_sp>thr_max)
    //    {
    //        thrust_sp = thr_max;
    //        saturation_z =  true;
    //    }

    float roll_sp, pitch_sp;
//    float thrust_sp;
    force_body(0) = cosf(-_yaw)*force_nav(0) - sinf(-_yaw)*force_nav(1);
    force_body(1) = sinf(-_yaw)*force_nav(0) + cosf(-_yaw)*force_nav(1);
    pitch_sp = - force_body(0);
    roll_sp = force_body(1);
//    thrust_sp = - force_body(2);

    math::Vector<3> force_nav_real;


    force_nav_real(0) = -20.0f*_att_sp.thrust*(cosf(_yaw)*_pitch + sinf(_yaw)*_roll);
    force_nav_real(1) = -20.0f*_att_sp.thrust*(sinf(_yaw)*_pitch - cosf(_yaw)*_roll);

    /* update the state */
    // North or X direction
    float s2_n          = z2_n - _vel(0);
    z2_n_dot            = force_nav_real(0)+hatd_n;
    float rhos21_minus_s2_n = rhos21_n - s2_n;
    float sqrt_temp_n     = sqrt(fabs(rhos21_minus_s2_n));
    rhos21_n_dot        = rhos22_n - epsilon1_n*sqrt_temp_n*sign_f(rhos21_minus_s2_n);
    rhos22_n_dot        = -epsilon2_n*sign_f(rhos21_minus_s2_n);
    float sd2_n         = s2_n + rhos21_n_dot;
    sqrt_temp_n         = sqrt(fabs(sd2_n));
    hatd_n_dot          = -rhos21_n_dot - L1_n*sqrt_temp_n*sign_f(sd2_n) - L2_n*Isd2_n;
    Isd2_n_dot          = sign_f(sd2_n);

    z2_n += z2_n_dot*dt;
    rhos21_n += rhos21_n_dot*dt;
    rhos22_n += rhos22_n_dot*dt;

    if(!saturation_x)
    {
        hatd_n += hatd_n_dot*dt;
        Isd2_n += Isd2_n_dot*dt;
    }
    // East or Y axis
    float s2_e          = z2_e - _vel(1);
    z2_e_dot            = force_nav_real(1) + hatd_e;
    float rhos21_minus_s2_e   = rhos21_e - s2_e;
    float sqrt_temp_e         = sqrt(fabs(rhos21_minus_s2_e));
    rhos21_e_dot        = rhos22_e - epsilon1_e*sqrt_temp_e*sign_f(rhos21_minus_s2_e);
    rhos22_e_dot        = -epsilon2_e*sign_f(rhos21_minus_s2_e);
    float sd2_e         = s2_e + rhos21_e_dot;
    sqrt_temp_e         = sqrt(fabs(sd2_e));
    hatd_e_dot          = -rhos21_e_dot - L1_e*sqrt_temp_e*sign_f(sd2_e) - L2_e*Isd2_e;
    Isd2_e_dot          = sign_f(sd2_e);
    z2_e += z2_e_dot*dt;
    rhos21_e += rhos21_e_dot*dt;
    rhos22_e += rhos22_e_dot*dt;

    if(!saturation_y)
    {
        hatd_e += hatd_e_dot*dt;
        Isd2_e += Isd2_e_dot*dt;
    }

    _ibvs_state.hat_e_sh = rhos21_n;
    _ibvs_state.hat_v_sh = rhos21_n_dot;
    _ibvs_state.c_g_hat = hatd_n;

    _ibvs_state.hat_e_sl1 = rhos21_e;
    _ibvs_state.hat_v_sl1 = rhos21_e_dot;
    _ibvs_state.eta_e1_hat = hatd_e;

    _img_att_sp.roll_body = roll_sp;
    _img_att_sp.pitch_body = pitch_sp;
    _img_att_sp.yaw_body = 0.0f;
    _img_att_sp.thrust = 0.0f;
//    _att_sp.yaw_body = _wrap_pi(_att_sp.yaw_body);
//    _att_sp.thrust = thrust_sp;

    if(_att_sp_ibvs_pub_fd != nullptr){
        orb_publish(ORB_ID(vehicle_image_attitude_setpoint),_att_sp_ibvs_pub_fd, &_img_att_sp);
    }
    else{
        orb_advertise(ORB_ID(vehicle_image_attitude_setpoint),&_img_att_sp);
    }

    if(_ibvs_state_pub_fd != nullptr)
    {
        orb_publish(ORB_ID(ibvs_state),_ibvs_state_pub_fd, &_ibvs_state);
    }
    else{
        orb_advertise(ORB_ID(ibvs_state),&_ibvs_state);
    }
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
    const float radius_circle = _params.radius_circle;
    const float omega_circle = _params.omega_circle;


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

                if((_vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)&&
                        (prev_nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER))
                {
                    //TODO change to real vel
                    z2_n = _vel_sp(0);
                    z2_e = _vel_sp(1);
                }

            }
            if (_local_pos.timestamp > 0) {
                _pos_err_d(0) = _pos_x_deriv.update(_pos_sp(0)-_pos(0));
                _pos_err_d(1) = _pos_y_deriv.update(_pos_sp(1)-_pos(1));
                _pos_err_d(2) = _pos_z_deriv.update(_pos_sp(2)-_pos(2));
            }

            automatic_3(dt,reset_int_xy,reset_int_z,reset_int_z_manual);
            if(_vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
                automatic_disturb_rej(dt,reset_int_xy,reset_int_z,reset_int_z_manual);

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
            {
//                warnx("Alt mode");
                float thr_val = throttle_curve(_manual.z, _params.thr_hover);
                _att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

                if (!_vehicle_land_detected.landed) {
                    _att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
                }
            }

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER){
                _att_sp.roll_body = _img_att_sp.roll_body;
                _att_sp.pitch_body = _img_att_sp.pitch_body;
            }

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
