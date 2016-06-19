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

//#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/control_state.h>

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

    /* Subscribe structures */
    struct vehicle_status_s _vehicle_status;
    struct vehicle_local_position_s _local_pos;
    struct actuator_armed_s _arming;
    struct control_state_s _ctrl_state;
    struct manual_control_setpoint_s _manual;
    struct vehicle_control_mode_s _control_mode;

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
    } _params_handles;

    struct{
        float thr_min;
        float thr_max;
        float thr_hover;
        float vel_max_up;
        float vel_max_down;

        math::Vector<3> pos_p;
        math::Vector<3> vel_p;
        math::Vector<3> vel_i;
        math::Vector<3> vel_d;
        math::Vector<3> vel_ff;
        math::Vector<3> vel_max;
    } _params;

    int parameters_update(bool force);

    /* publication topics */
    orb_advert_t _mavlink_log_pub;


    control::BlockDerivative _vel_x_deriv;
    control::BlockDerivative _vel_y_deriv;
    control::BlockDerivative _vel_z_deriv;

    math::Matrix<3, 3> _R;
    math::Vector<3> _pos;
    math::Vector<3> _vel;
    math::Vector<3> _vel_sp;
    math::Vector<3> _vel_ff;
    math::Vector<3> _vel_err_d;

    float _yaw;

    void task_main();
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
    /*subscribed structure*/
    _vehicle_status{},
    _local_pos{},
    _arming{},
    _ctrl_state{},
    _manual{},
    _control_mode{},
    /*publication*/
    _mavlink_log_pub(nullptr),

    _vel_x_deriv(this, "VELD"),
    _vel_y_deriv(this, "VELD"),
    _vel_z_deriv(this, "VELD"),

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
//    _pos_sp.zero();
    _vel.zero();
    _vel_sp.zero();
    _vel_ff.zero();


    _params_handles.thr_min		= param_find("MPC_THR_MIN");
    _params_handles.thr_max		= param_find("MPC_THR_MAX");
    _params_handles.thr_hover	= param_find("MPC_THR_HOVER");

    _params_handles.z_p		= param_find("MPC_Z_P");
    _params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
    _params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
    _params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
    _params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
    _params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");
    _params_handles.z_ff		= param_find("MPC_Z_FF");

    _params_handles.xy_p		= param_find("MPC_XY_P");
    _params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
    _params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
    _params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
    _params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
    _params_handles.xy_ff		= param_find("MPC_XY_FF");

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
    }

    return OK;
}

void ViconControl::task_main(){

    /*Do subscription */
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _arming_sub = orb_subscribe(ORB_ID(actuator_armed));
    _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));


    /* initialize values of critical structs until first regular update */
    _arming.armed = false;
    bool was_armed = false;

    poll_subscriptions();


    hrt_abstime t_prev = 0;

    px4_pollfd_struct_t fds[1];
    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;

    while (!_task_should_exit){
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

        if (_control_mode.flag_armed && !was_armed) {

        }
        //Update previous arming state
        was_armed = _control_mode.flag_armed;


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
        }
        //TODO to check whether this will send log info on QGC
//        mavlink_and_console_log_info(&_mavlink_log_pub, "[vcn] stopped");
    }

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
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}



