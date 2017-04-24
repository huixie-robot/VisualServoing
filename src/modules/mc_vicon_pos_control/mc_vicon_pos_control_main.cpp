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
extern "C" __EXPORT int mc_vicon_pos_control_main(int argc, char *argv[]);

class ViconControl:public control::SuperBlock
{
public:
    /* Constructor */
    ViconControl();
    /* Destructor */
    virtual ~ViconControl();
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

    bool _reset_pos_sp;

    math::Matrix<3, 3> _R;
    math::Vector<3> _pos;
    math::Vector<3> _pos_sp;
    math::Vector<3> _vel;
    math::Vector<3> _vel_prev;
    math::Vector<3> _vel_sp;
    math::Vector<3> _vel_sp_prev;
    math::Vector<3> _vel_ff;
    math::Vector<3> _vel_err_d;
    math::Vector<3> _pos_err_d;
    math::Vector<3> _thrust_int;


    float _yaw;

    void cal_vel_sp(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
    void automatic(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
    void automatic_2(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);
    void automatic_3(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual);

    static float throttle_curve(float ctl, float ctr);
    void task_main();
    void reset_pos_sp();
    void manual(bool &reset_yaw_sp, float dt);
};

namespace Vicon_Control
{
    ViconControl *g_control;
}

ViconControl::ViconControl():
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

    _reset_pos_sp(true),

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
    _vel_sp_prev.zero();
    _vel_ff.zero();
    _vel_err_d.zero();
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


ViconControl::~ViconControl(){
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

void ViconControl::status(){
   warnx("Refference: Roll:%.5f Pitch:%.5f Yaw:%.5f Thrust:%.5f",
         (double)_att_sp.roll_body, (double)_att_sp.pitch_body,(double)_att_sp.yaw_body,(double)_att_sp.thrust);
}

float ViconControl::throttle_curve(float ctl, float ctr)
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


void ViconControl::reset_pos_sp()
{
    if (_reset_pos_sp) {
        _reset_pos_sp = false;

        // we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
        // continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
        // position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
        _pos_sp(0) = _pos(0);
        _pos_sp(1) = _pos(1);
    }
}

void ViconControl::poll_subscriptions()
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

int ViconControl::parameters_update(bool force)
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

// Calculating the reference velocity based on position setpoint
void ViconControl::cal_vel_sp(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual)
{
    _vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
    _vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
    _vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);

    /* make sure velocity setpoint is saturated in xy*/
    float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
                  _vel_sp(1) * _vel_sp(1));
    if (vel_norm_xy > _params.vel_max(0)) {
        /* note assumes vel_max(0) == vel_max(1) */
        _vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
        _vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
    }

    /* make sure velocity setpoint is saturated in z*/
    if (_vel_sp(2) < -1.0f * _params.vel_max_up){
        _vel_sp(2) = -1.0f * _params.vel_max_up;
    }
    if (_vel_sp(2) >  _params.vel_max_down) {
        _vel_sp(2) = _params.vel_max_down;
    }

    // limit total horizontal acceleration
    math::Vector<2> acc_hor;
    acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
    acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;

    if (acc_hor.length() > _params.acc_hor_max) {
        acc_hor.normalize();
        acc_hor *= _params.acc_hor_max;
        math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
        math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
        _vel_sp(0) = vel_sp_hor(0);
        _vel_sp(1) = vel_sp_hor(1);
    }

    // limit vertical acceleration
    float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;

    if (fabsf(acc_v) > 2 * _params.acc_hor_max) {
        acc_v /= fabsf(acc_v);
        _vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
    }

    _vel_sp_prev = _vel_sp;

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
}

void ViconControl::automatic(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual)
{

    cal_vel_sp(dt,reset_int_xy,reset_int_z,reset_int_z_manual);
    /* velocity error */
    math::Vector<3> vel_err = _vel_sp - _vel;
    math::Vector<3> thrust_sp;
    thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + _thrust_int;

    /* limit thrust vector and check for saturation */
    bool saturation_xy = false;
    bool saturation_z = false;

    float thr_min = _params.thr_min;
    float thrust_abs = thrust_sp.length();
    float tilt_max = _params.tilt_max_air;
    float thr_max = _params.thr_max;

    if (thr_min < 0.0f) {
        /* don't allow downside thrust direction in manual attitude mode */
        thr_min = 0.0f;
    }

    /* limit min lift */
    if (-thrust_sp(2) < thr_min) {
        thrust_sp(2) = -thr_min;
        saturation_z = true;
    }

    const float TILT_COS_MAX = 0.7f;
    float att_comp;

    if (_R(2, 2) > TILT_COS_MAX) {
        att_comp = 1.0f / _R(2, 2);

    } else if (_R(2, 2) > 0.0f) {
        att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
        saturation_z = true;

    } else {
        att_comp = 1.0f;
        saturation_z = true;
    }
    thrust_sp(2) *= att_comp;



    if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
        /* absolute horizontal thrust */
        float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

        if (thrust_sp_xy_len > 0.01f) {
            /* max horizontal thrust for given vertical thrust*/
            float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

            if (thrust_sp_xy_len > thrust_xy_max) {
                float k = thrust_xy_max / thrust_sp_xy_len;
                thrust_sp(0) *= k;
                thrust_sp(1) *= k;
                saturation_xy = true;
            }
        }
    }

    /* limit max thrust */
    thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */

    if (thrust_abs > thr_max) {
        if (thrust_sp(2) < 0.0f) {
            if (-thrust_sp(2) > thr_max) {
                /* thrust Z component is too large, limit it */
                thrust_sp(0) = 0.0f;
                thrust_sp(1) = 0.0f;
                thrust_sp(2) = -thr_max;
                saturation_xy = true;
                saturation_z = true;

            } else {
                /* preserve thrust Z component and lower XY, keeping altitude is more important than position */
                float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
                float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
                float k = thrust_xy_max / thrust_xy_abs;
                thrust_sp(0) *= k;
                thrust_sp(1) *= k;
                saturation_xy = true;
            }

        } else {
            /* Z component is negative, going down, simply limit thrust vector */
            float k = thr_max / thrust_abs;
            thrust_sp *= k;
            saturation_xy = true;
            saturation_z = true;
        }

        thrust_abs = thr_max;
    }

    /* update integrals */
    if (!saturation_xy) {
        _thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
        _thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
    }

    if (!saturation_z) {
        _thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

        /* protection against flipping on ground when landing */
        if (_thrust_int(2) > 0.0f) {
            _thrust_int(2) = 0.0f;
        }
    }


    /* desired body_z axis = -normalize(thrust_vector) */
    math::Matrix<3, 3> R;
    R.identity();
    math::Vector<3> body_x;
    math::Vector<3> body_y;
    math::Vector<3> body_z;

    const float SIGMA = 0.0001f;
    if (thrust_abs > SIGMA) {
        body_z = -thrust_sp / thrust_abs;

    } else {
        /* no thrust, set Z axis to safe value */
        body_z.zero();
        body_z(2) = 1.0f;
    }

    /* vector of desired yaw direction in XY plane, rotated by PI/2 */
    math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

    if (fabsf(body_z(2)) > SIGMA) {
        /* desired body_x axis, orthogonal to body_z */
        body_x = y_C % body_z;

        /* keep nose to front while inverted upside down */
        if (body_z(2) < 0.0f) {
            body_x = -body_x;
        }

        body_x.normalize();

    } else {
        /* desired thrust is in XY plane, set X downside to construct correct matrix,
         * but yaw component will not be used actually */
        body_x.zero();
        body_x(2) = 1.0f;
    }

    /* desired body_y axis */
    body_y = body_z % body_x;

    /* fill rotation matrix */
    for (int i = 0; i < 3; i++) {
        R(i, 0) = body_x(i);
        R(i, 1) = body_y(i);
        R(i, 2) = body_z(i);
    }

    /* copy rotation matrix to attitude setpoint topic */
    memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
    _att_sp.R_valid = true;

    /* copy quaternion setpoint to attitude setpoint topic */
    math::Quaternion q_sp;
    q_sp.from_dcm(R);
    memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));

    /* calculate euler angles, for logging only, must not be used for control */
    math::Vector<3> euler = R.to_euler();
    _att_sp.roll_body = euler(0);
    _att_sp.pitch_body = euler(1);
    /* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
    _att_sp.thrust = thrust_abs;
    _att_sp.timestamp = hrt_absolute_time();
}

void ViconControl::automatic_2(float dt, bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual){

//    reset_int_xy = true;
    cal_vel_sp(dt,reset_int_xy,reset_int_z,reset_int_z_manual);

    /* velocity error */
    math::Vector<3> vel_err = _vel_sp - _vel;
    math::Vector<3> force_nav;
    math::Vector<3> force_body;
    force_nav = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + _thrust_int;

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
    else if(pitch_sp < - tilt_max)
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
        _thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
    }

    /* update integrals */
    if (!saturation_y) {
        _thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
    }

    if (!saturation_z) {
        _thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

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


void ViconControl::automatic_3(float dt,bool &reset_int_xy, bool &reset_int_z, bool &reset_int_z_manual){

    /* Here only reset_int_xy, reset_int_z and reset_int_z manual is used */
    cal_vel_sp(dt,reset_int_xy,reset_int_z,reset_int_z_manual);

    math::Vector<3> pos_err = _pos_sp - _pos;
    math::Vector<3> force_nav;
    math::Vector<3> force_body;


    force_nav = pos_err.emult(_params.vel_p) + _pos_err_d.emult(_params.vel_d)+_thrust_int;
//    force_nav = pos_err.emult(_params.vel_p) - _vel.emult(_params.vel_d)+_thrust_int;

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


void ViconControl::manual(bool &reset_yaw_sp, float dt){
    /* reset yaw setpoint to current position if needed */
    if (reset_yaw_sp) {
        reset_yaw_sp = false;
        _att_sp.yaw_body = _yaw;
    }
    else if (!_vehicle_land_detected.landed &&_manual.z > 0.1f)
    {

//        /* we want to know the real constraint, and global overrides manual */
//        const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
//                       _params.global_yaw_max;
//        const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

//        _att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
//        float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
//        float yaw_offs = _wrap_pi(yaw_target - _yaw);

//        // If the yaw offset became too big for the system to track stop
//        // shifting it

//        // XXX this needs inspection - probably requires a clamp, not an if
//        if (fabsf(yaw_offs) < yaw_offset_max) {
////            _att_sp.yaw_body = yaw_target;
//        }
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

void ViconControl::task_main(){

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
    px4_pollfd_struct_t fds[1];
    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;

    uint8_t prev_nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

    bool ground_set = false;
    math::Vector<3> ground_pos;

//    int count_traj=1;
//    hrt_abstime time = hrt_absolute_time();
//    hrt_abstime print_time = time + 5000000;

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
            _vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
            _vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
            _vel_err_d(2) = _vel_z_deriv.update(-_vel(2));

            _pos_err_d(0) = _pos_x_deriv.update(-_pos(0));
            _pos_err_d(1) = _pos_y_deriv.update(-_pos(1));
            _pos_err_d(2) = _pos_z_deriv.update(-_pos(2));

            if(!ground_set)
            {
                ground_pos(0) = _pos(0);
                ground_pos(1) = _pos(1);
                ground_pos(2) = _pos(2);
                ground_set = true;
            }
        }

        if (_control_mode.flag_armed && !was_armed) {
            reset_int_z = true;
            reset_int_xy = true;
            reset_yaw_sp = true;
            _pos_sp(0) = _pos(0);
            _pos_sp(1) = _pos(1);
            _pos_sp(2) = _pos(2) - 1.0f;
        }
        //Update previous arming state
        was_armed = _control_mode.flag_armed;
        _att_sp.disable_mc_yaw_control = false;

        if((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)||
                (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)||
                (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER))
        {
//           automatic(dt,reset_int_xy,reset_int_z,reset_int_z_manual);
//            automatic_2(dt,reset_int_xy,reset_int_z,reset_int_z_manual);
            automatic_3(dt,reset_int_xy,reset_int_z,reset_int_z_manual);

//            reset_int_z_manual = _control_mode.flag_armed &&
//                    (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
//                     ||_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL);


            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
            {
//                warnx("lateral automatic control");
                if(prev_nav_state!=vehicle_status_s::NAVIGATION_STATE_ALTCTL)
                {
                    _att_sp.yaw_body = _yaw;
                    _pos_sp(0) = _pos(0);
                    _pos_sp(1) = _pos(1);

                }
                float thr_val = throttle_curve(_manual.z, _params.thr_hover);
                _att_sp.thrust = math::min(thr_val, _manual_thr_max.get());

                if (!_vehicle_land_detected.landed) {
                    _att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
                }
            }

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
            {
//                warnx("3D automatic control");
                if(prev_nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
                {
                    _pos_sp(2) = _pos(2);
                    reset_int_z = true;
                    reset_int_xy = true;
                    if(_control_mode.flag_armed)
                        reset_int_z_manual = true;
                }

//                if(prev_nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)
//                {
//                    _pos_sp(2) = ground_pos(2) - 1.5f;
//                    _pos_sp(1) = ground_pos(1) + 0.15f;
//                    _pos_sp(0) = ground_pos(0) - 0.15f;
//                    reset_int_z = true;
//                    reset_int_xy = true;
//                    if(_control_mode.flag_armed)
//                        reset_int_z_manual = true;
//                }


//                if(time> print_time)
//                {
//                    print_time = time + 5000000;
//                    if(count_traj%2==0)
//                    {
//                        _pos_sp(0) += 1.0f;
//                        _pos_sp(1) += 1.0f ;
//                    }
//                    else if(count_traj%2==1)
//                    {
//                        _pos_sp(0) -= 1.0f;
//                        _pos_sp(1) -= 1.0f;
//                    }
//                    count_traj ++;
////                    warnx("thrust_given: %.5f", (double)_att_sp.thrust);
//                }
//                time = hrt_absolute_time();
            }

            if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
            {
//                warnx("ibvs is on");
                uint8_t enable = _params.ibvs_enable;
                bool roll_enabled = (enable & (uint8_t)1);
                bool pitch_enabled = (enable & (uint8_t)2);
                bool thrust_enabled = (enable & (uint8_t)4);
                bool yaw_enabled = (enable & (uint8_t)8);

                bool img_roll_valid  = (_img_att_sp.valid &((uint8_t)1));
                bool img_pitch_valid = (_img_att_sp.valid &((uint8_t)2));
                bool img_thrust_valid = (_img_att_sp.valid &((uint8_t)4));
                bool img_yaw_valid = (_img_att_sp.valid &((uint8_t)8));

                float thr_min = _params.thr_min;
                float tilt_max = _params.tilt_max_air;
                float thr_max = _params.thr_max;


                if(roll_enabled && img_roll_valid)
                {
                    _att_sp.roll_body =_img_att_sp.roll_body;
                    if(_att_sp.roll_body>tilt_max)
                        _att_sp.roll_body = tilt_max;
                    else if(_att_sp.roll_body<-tilt_max)
                        _att_sp.roll_body = -tilt_max;
//                    warnx("roll controlled by image");
                }
                if(pitch_enabled && img_pitch_valid)
                {
                    _att_sp.pitch_body = _img_att_sp.pitch_body;
                    if(_att_sp.pitch_body>tilt_max)
                        _att_sp.pitch_body = tilt_max;
                    else if(_att_sp.pitch_body<-tilt_max)
                        _att_sp.pitch_body = -tilt_max;
//                    warnx("pitch controlled by image");
                }
                if(yaw_enabled && img_yaw_valid)
                {
                    _att_sp.yaw_body = _img_att_sp.yaw_body;
//                    warnx("yaw controlled by image");
                }
                if(thrust_enabled && img_thrust_valid)
                {
                    _att_sp.thrust = _img_att_sp.thrust + _params.thr_hover;
                    if(_att_sp.thrust>thr_max)
                        _att_sp.thrust = thr_max;
                    else if(_att_sp.thrust<thr_min)
                        _att_sp.thrust = thr_min;
//                    warnx("thrust controlled by image");
                }
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

void ViconControl::task_main_trampoline(int argc, char *argv[])
{
    Vicon_Control::g_control->task_main();
}

int ViconControl::start(){
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("mc_vicon_pos_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1900,
                       (px4_main_t)&ViconControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

int mc_vicon_pos_control_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: mc_vicon_pos_control {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (Vicon_Control::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        Vicon_Control::g_control = new ViconControl;

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



