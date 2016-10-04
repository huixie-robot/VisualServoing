/****************************************************************************
 *
 *   Copyright (c) 2014 ANCL Development Team. All rights reserved.
 *   Author: @Hui Xie <xie1@ualberta.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_ibvs_main.cpp
 * Multicopter Image Based Visual Servoing controller.
 *
 */
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <fcntl.h>
#include <string.h>
#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>


#include <uORB/uORB.h>
#include <uORB/topics/image_features.h>
#include <uORB/topics/vehicle_image_attitude_setpoint.h>
#include <uORB/topics/ibvs_state.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/perf_counter.h>

#define PI  3.14159f
#define MAX_FEATURES 8


static bool isValid(struct image_features_s *image_features, int n);
//static math::Vector<3> projector(math::Vector<3> input, math::Vector<3> estimate, float bound, float epsilon = 0.000001f);
static math::Vector<2> projector(math::Vector<2> input, math::Vector<2> estimate, float bound, float epsilon = 0.000001f);



class MulticopterOPFIBVS
{
public:
    /**
     * Constructor
     */
    MulticopterOPFIBVS();

    /**
     * Desctructor, also kills task
     */
     ~MulticopterOPFIBVS();
    /**
     * Start task.
     * @return      OK on success
     */
    int start();
	void status();
private:
    bool	_task_should_exit;		/**< if true, task should exit */
    int		_ibvs_task;             /**< task handle for task */

    /* Various subscription topic file descriptor */
    int             _img_feature_sub_fd;
    int             _parameter_update_sub_fd;
//    int             _att_sub_fd;
    int             _vehicle_status_sub_fd;
    int             _ctrl_state_sub_fd;

    /* Various topics subscription variables definition */
    struct image_features_s             _img_feature;
//    struct vehicle_attitude_s           _att;
    struct vehicle_status_s             _vehicle_status;
    struct control_state_s              _ctrl_state;

    /**
     * Check for changes in subscribed topics.
     */
    void		poll_subscriptions();

    struct{
//        Height Gains
        param_t k1_h;
        param_t d1_h;
        param_t k2_h;
        param_t d2_h;
        param_t gamma1_h;
        param_t gamma2_h;
        param_t l1_h;
        param_t l2_h;

//        Roll Gains
        param_t k1_r;
        param_t d1_r;
        param_t k2_r;
        param_t d2_r;
        param_t gamma1_r;
        param_t gamma2_r;
        param_t l1_r;
        param_t l2_r;

//        Pitch Gains
        param_t k1_p;
        param_t d1_p;
        param_t k2_p;
        param_t d2_p;
        param_t gamma1_p;
        param_t gamma2_p;
        param_t l1_p;
        param_t l2_p;

        param_t k_psi;
    }_params_handles;

    struct{
        float k1_h;
        float d1_h;
        float k2_h;
        float d2_h;
        float gamma1_h;
        float gamma2_h;
        float l1_h;
        float l2_h;

        float k1_r;
        float d1_r;
        float k2_r;
        float d2_r;
        float gamma1_r;
        float gamma2_r;
        float l1_r;
        float l2_r;

        float k1_p;
        float d1_p;
        float k2_p;
        float d2_p;
        float gamma1_p;
        float gamma2_p;
        float l1_p;
        float l2_p;

        float k_psi;
    }_params;

    /**
     * Update our local parameter cache.
     */
    int			parameters_update(bool force);

    /* publication topics */
    orb_advert_t    _att_sp_ibvs_pub_fd;
    orb_advert_t    _ibvs_state_pub_fd;
    /*Publish structure*/
    struct vehicle_image_attitude_setpoint_s _att_sp_ibvs;
    struct ibvs_state_s _ibvs_state;

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void		task_main() __attribute__((noreturn));


    // state for height subsystem
    float e_sh;
    math::Vector<2> xi_hy;
    math::Vector<2> xi_hu;
    math::Vector<2> xi_hd;
    math::Vector<2> vartheta_h1;
    math::Vector<2> vartheta_h2;

    float e_sl1;
    float tilde_e_sl1;
    float hat_e_sl1;
    float hat_v_sl1;
    float eta_e1_update;

    float e_sl2;
    float tilde_e_sl2;
    float hat_e_sl2;
    float hat_v_sl2;
    float eta_e2_update;

    math::Vector<3> euler_angles;
    math::Matrix<3, 3> _R;
};

namespace ibvs
{
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;
MulticopterOPFIBVS      *g_control;
}


MulticopterOPFIBVS::MulticopterOPFIBVS():
    _task_should_exit(false),
    _ibvs_task(-1),
    /* subscriptions file descriptor */
    _img_feature_sub_fd(-1),
    _parameter_update_sub_fd(-1),
//    _att_sub_fd(-1),
    _vehicle_status_sub_fd(-1),
    _ctrl_state_sub_fd{-1},
    /* subscribed structures */
    _img_feature{},
//    _att{},
    _vehicle_status{},
    _ctrl_state{},
    // publication
    _att_sp_ibvs_pub_fd(nullptr),
    _ibvs_state_pub_fd{nullptr},
    _att_sp_ibvs{},
    _ibvs_state{},
    // state variables
    e_sh(0.0f),
    e_sl1(0.0f),
    tilde_e_sl1(0.0f),
    hat_e_sl1(0.0f),
    hat_v_sl1(0.0f),
    eta_e1_update(0.0f),
    e_sl2(0.0f),
    tilde_e_sl2(0.0f),
    hat_e_sl2(0.0f),
    hat_v_sl2(0.0f),
    eta_e2_update(0.0f)
{
    // Make the quaternion valid for control state
    _ctrl_state.q[0] = 1.0f;

    xi_hy.zero();
    xi_hu.zero();
    xi_hd.zero();
    vartheta_h1.zero();
    vartheta_h2.zero();

    euler_angles.zero();
    _R.identity();
//    t_prev = 0;

//    /* fetch initial parameter values */
//    parameters_update(true);
////    _loop_perf = perf_alloc(PC_INTERVAL, "mc_ibvs_saturation");
////    _timeout_count = perf_alloc(PC_COUNT, "mc_ibvs_saturation timeout");
//	t=hrt_absolute_time();

    _params_handles.k1_h        = param_find("IBVS_K1_H");
    _params_handles.d1_h        = param_find("IBVS_D1_H");
    _params_handles.k2_h        = param_find("IBVS_K2_H");
    _params_handles.d2_h        = param_find("IBVS_D2_H");
    _params_handles.gamma1_h    = param_find("IBVS_GAMMA1_H");
    _params_handles.gamma2_h    = param_find("IBVS_GAMMA2_H");
    _params_handles.l1_h        = param_find("IBVS_L1_H");
    _params_handles.l2_h         = param_find("IBVS_L2_H");

    _params_handles.k1_r        = param_find("IBVS_K1_R");
    _params_handles.d1_r        = param_find("IBVS_D1_R");
    _params_handles.k2_r        = param_find("IBVS_K2_R");
    _params_handles.d2_r        = param_find("IBVS_D2_R");
    _params_handles.gamma1_r    = param_find("IBVS_GAMMA1_R");
    _params_handles.gamma2_r    = param_find("IBVS_GAMMA2_R");
    _params_handles.l1_r        = param_find("IBVS_L1_R");
    _params_handles.l2_r         = param_find("IBVS_L2_R");

    _params_handles.k1_p        = param_find("IBVS_K1_P");
    _params_handles.d1_p        = param_find("IBVS_D1_P");
    _params_handles.k2_p        = param_find("IBVS_K2_P");
    _params_handles.d2_p        = param_find("IBVS_D2_P");
    _params_handles.gamma1_p    = param_find("IBVS_GAMMA1_P");
    _params_handles.gamma2_p    = param_find("IBVS_GAMMA2_P");
    _params_handles.l1_p        = param_find("IBVS_L1_P");
    _params_handles.l2_p         = param_find("IBVS_L2_P");

    _params_handles.k_psi       = param_find("IBVS_K_PSI1");

    /* fetch initial parameter values */
    parameters_update(true);
}

MulticopterOPFIBVS::~MulticopterOPFIBVS(){

    if(_ibvs_task !=-1){
        _task_should_exit = true;
        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do{
            usleep(20000);
            if(++i > 50){
                task_delete(_ibvs_task);
                break;
            }
        }while(_ibvs_task !=-1);
    }
    ibvs::g_control = nullptr;
}

int
MulticopterOPFIBVS::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(_parameter_update_sub_fd, &updated);

    if(updated)
        orb_copy(ORB_ID(parameter_update), _parameter_update_sub_fd, &param_upd);

    if (updated || force){
        param_get(_params_handles.k1_h, &(_params.k1_h));
        param_get(_params_handles.d1_h, &(_params.d1_h));
        param_get(_params_handles.k2_h, &(_params.k2_h));
        param_get(_params_handles.d2_h, &(_params.d2_h));
        param_get(_params_handles.gamma1_h,&(_params.gamma1_h));
        param_get(_params_handles.gamma2_h,&(_params.gamma2_h));
        param_get(_params_handles.l1_h,&(_params.l1_h));
        param_get(_params_handles.l2_h,&(_params.l2_h));

        param_get(_params_handles.k1_r, &(_params.k1_r));
        param_get(_params_handles.d1_r, &(_params.d1_r));
        param_get(_params_handles.k2_r, &(_params.k2_r));
        param_get(_params_handles.d2_r, &(_params.d2_r));
        param_get(_params_handles.gamma1_r,&(_params.gamma1_r));
        param_get(_params_handles.gamma2_r,&(_params.gamma2_r));
        param_get(_params_handles.l1_r,&(_params.l1_r));
        param_get(_params_handles.l2_r,&(_params.l2_r));

        param_get(_params_handles.k1_p, &(_params.k1_p));
        param_get(_params_handles.d1_p, &(_params.d1_p));
        param_get(_params_handles.k2_p, &(_params.k2_p));
        param_get(_params_handles.d2_p, &(_params.d1_p));
        param_get(_params_handles.gamma1_p,&(_params.gamma1_p));
        param_get(_params_handles.gamma2_p,&(_params.gamma2_p));
        param_get(_params_handles.l1_p,&(_params.l1_p));
        param_get(_params_handles.l2_p,&(_params.l2_p));

        param_get(_params_handles.k_psi,&(_params.k_psi));
    }
    return OK;
}


int
MulticopterOPFIBVS::start()
{
    ASSERT(_ibvs_task == -1);

    //TODO check the priority
    /* start the task */
    _ibvs_task = px4_task_spawn_cmd("mc_ibvs_adap",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_POSITION_CONTROL,
                       1024,
                       (main_t)&MulticopterOPFIBVS::task_main_trampoline,
                       nullptr);

    if (_ibvs_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
MulticopterOPFIBVS::task_main_trampoline(int argc, char *argv[])
{
    ibvs::g_control->task_main();
}

void
MulticopterOPFIBVS::poll_subscriptions()
{
    bool updated;
//  update the attitude
//    orb_check(_att_sub_fd, &updated);
//    if (updated){
//        orb_copy(ORB_ID(vehicle_attitude), _att_sub_fd, &_att);
//    }
//    update the image feature
    orb_check(_img_feature_sub_fd, &updated);
    if (updated)
    {
        orb_copy(ORB_ID(image_features), _img_feature_sub_fd, &_img_feature);
    }
//    update vehicle's status
    orb_check(_vehicle_status_sub_fd, &updated);
    if(updated){
        orb_copy(ORB_ID(vehicle_status),_vehicle_status_sub_fd,&_vehicle_status);
    }

//    update vehicle control state
    orb_check(_ctrl_state_sub_fd,&updated);
    if(updated){
        orb_copy(ORB_ID(control_state),_ctrl_state_sub_fd,&_ctrl_state);
        math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
//        _R = q_att.to_dcm();
        euler_angles = q_att.to_euler();
//        warnx("R P Y: %.4f, %.4f, %.4f", (double)euler_angles(0),(double)euler_angles(1),(double)euler_angles(2));
    }
}

void
MulticopterOPFIBVS::task_main()
{
    warnx("started");

    /* subscription */
    _img_feature_sub_fd         = orb_subscribe(ORB_ID(image_features));
    _parameter_update_sub_fd    = orb_subscribe(ORB_ID(parameter_update));
    _vehicle_status_sub_fd      = orb_subscribe(ORB_ID(vehicle_status));
    _ctrl_state_sub_fd          = orb_subscribe(ORB_ID(control_state));

    parameters_update(true);
    poll_subscriptions();

    struct pollfd fds[1];
    fds[0].fd = _img_feature_sub_fd;
    fds[0].events = POLLIN;

    hrt_abstime t_prev = hrt_absolute_time();
//    hrt_abstime t_print = 0;

    bool ibvs_on_prev = false;
//    bool ibvs_int_reset_roll = true;
//    bool ibvs_int_reset_pitch = true;
    bool ibvs_int_reset_thrust = true;


    uint8_t prev_nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;

    while(!_task_should_exit){
        int poll_ret = poll(fds,(sizeof(fds) / sizeof(fds[0])),500);

        /* this is undesirable but not much we can do */
        if (poll_ret < 0) {
            warn("poll error %d, %d", poll_ret, errno);
            continue;
        }

        if(poll_ret==0)
        {
            warnx("poll no return");
            _att_sp_ibvs.valid = 0;
            if(_att_sp_ibvs_pub_fd != nullptr)
            {
                orb_publish(ORB_ID(vehicle_image_attitude_setpoint),_att_sp_ibvs_pub_fd, &_att_sp_ibvs);
            }
            else{
                orb_advertise(ORB_ID(vehicle_image_attitude_setpoint),&_att_sp_ibvs);
            }
            _ibvs_state.valid = 0;
            if(_ibvs_state_pub_fd != nullptr)
            {
                orb_publish(ORB_ID(ibvs_state),_ibvs_state_pub_fd, &_ibvs_state);
            }
            else{
                orb_advertise(ORB_ID(ibvs_state),&_ibvs_state);
            }
            continue;
        }

        poll_subscriptions();
        parameters_update(false);

        hrt_abstime t = hrt_absolute_time();
        float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
        t_prev = t;

        bool ibvs_on = (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

        if(ibvs_on == true && ibvs_on_prev == false)
        {
//            ibvs_int_reset_roll = true;
//            ibvs_int_reset_pitch = true;
            ibvs_int_reset_thrust = true;
        }

        if(ibvs_on != true)
        {
//            ibvs_int_reset_roll = true;
//            ibvs_int_reset_pitch = true;
            ibvs_int_reset_thrust = true;
        }


        ibvs_on_prev = ibvs_on;

        _att_sp_ibvs.valid = 0;
        _ibvs_state.valid = 0;

//        /* for x direction motion and pitch */
//        if(isValid(&_img_feature,0))
//        {
////            warnx("S1 is valid");
//            e_sl1 = _img_feature.s[0];
//            tilde_e_sl1 = e_sl1 - hat_e_sl1;
//            hat_e_sl1 += (-hat_v_sl1 + _params.l_l1*tilde_e_sl1)*dt;
//            hat_v_sl1 += -_params.l_l1*_params.l_ld1*tilde_e_sl1*dt;

//            float eta_e1_hat;
//            if(ibvs_int_reset_pitch)
//            {
//                eta_e1_hat = 0.0f;
//                eta_e1_update = 0.0f;
//                ibvs_int_reset_pitch = false;
//            }
//            else
//            {
//                eta_e1_update += (e_sl1+tilde_e_sl1)*dt;
//                eta_e1_hat = -_params.gamma_l1*(e_sl1+tilde_e_sl1) -  _params.gamma_l1*_params.l_ld1*eta_e1_update;
//            }

//            _att_sp_ibvs.pitch_body = _params.k_l1*(hat_v_sl1 + (_params.l_ld1-_params.l_l1)*tilde_e_sl1 - _params.l_ld1*e_sl1) + eta_e1_hat;

//            _ibvs_state.hat_e_sl1 = hat_e_sl1;
//            _ibvs_state.hat_v_sl1 = hat_v_sl1;
//            _ibvs_state.eta_e1_hat = eta_e1_hat;

//            _att_sp_ibvs.valid += ((uint8_t)2);
//            _ibvs_state.valid += ((uint8_t)2);
//        }
//        else
//        {
//            eta_e1_update = 0.0f;
//            _att_sp_ibvs.pitch_body = 0.0f;
//        }

//        /* for y direction motion and roll */
//        if(isValid(&_img_feature,1))
//        {
////            warnx("S2 is valid");
//            e_sl2 = _img_feature.s[1];
//            tilde_e_sl2 = e_sl2 - hat_e_sl2;
//            hat_e_sl2 += (-hat_v_sl2 + _params.l_l2*tilde_e_sl2)*dt;
//            hat_v_sl2 += -_params.l_l2*_params.l_ld2*tilde_e_sl2*dt;

//            float eta_e2_hat;
//            if(ibvs_int_reset_roll)
//            {
//                eta_e2_hat = 0.0f;
//                eta_e2_update = 0.0f;
//                ibvs_int_reset_roll = false;
//            }
//            else
//            {
//                eta_e2_update += (e_sl2+tilde_e_sl2)*dt;
//                eta_e2_hat = _params.gamma_l2*(e_sl2+tilde_e_sl2) +  _params.gamma_l2*_params.l_ld2*eta_e2_update;
//            }

//            _att_sp_ibvs.roll_body = -_params.k_l2*(hat_v_sl2 + (_params.l_ld2-_params.l_l2)*tilde_e_sl2 - _params.l_ld2*e_sl2)+ eta_e2_hat;
//            _att_sp_ibvs.valid += ((uint8_t)1);

//            _ibvs_state.hat_e_sl2 = hat_e_sl2;
//            _ibvs_state.hat_v_sl2 = hat_v_sl2;
//            _ibvs_state.eta_e2_hat = eta_e2_hat;
//            _ibvs_state.valid += ((uint8_t)1);
//        }
//        else
//        {
//            eta_e2_update = 0.0f;
//            _att_sp_ibvs.roll_body = 0.0f;
//        }


        if(isValid(&_img_feature,2))
        {
            math::Matrix<2,2> Ao;
            math::Vector<2> Lh(_params.l1_h,-_params.l2_h);
            math::Vector<2> Bo(0.0f,1.0f);

            e_sh = _img_feature.s[2] - 1.0f;
            float tilde_xi_hy1 = xi_hy(0) - e_sh;
            math::Vector<2> varpi_h1((_params.k1_h+_params.d1_h)*e_sh -xi_hy(1),
                                     xi_hd(1));
            math::Vector<2> varpi_h2(vartheta_h1(0)*(_params.k1_h+_params.d1_h)*xi_hu(1)-e_sh,
                                     -vartheta_h1(0)*(_params.k1_h+_params.d1_h)*xi_hd(1));
            float delta_h2 = xi_hu(1) - vartheta_h1*varpi_h1;
            math::Vector<2> ell_h(-(_params.k1_h+_params.d1_h)*xi_hy(1)- _params.l2_h*tilde_xi_hy1,
                                  _params.l2_h*xi_hd(1)+1.0f);

            _att_sp_ibvs.thrust = -_params.l2_h*xi_hu(0) - _params.gamma1_h*e_sh*(varpi_h1*varpi_h1) + vartheta_h1*ell_h
                    -(_params.k2_h+_params.d2_h*(_params.k1_h+_params.d1_h)*(_params.k1_h+_params.d1_h)*vartheta_h1(0)*vartheta_h1(0))*delta_h2
                    -vartheta_h2*varpi_h2;

            // Observer Part
            xi_hy += (Ao*xi_hy - Lh*tilde_xi_hy1)*dt;
            xi_hu += (Ao*xi_hu - Lh*xi_hu(0)+Bo*_att_sp_ibvs.thrust)*dt;
            xi_hd += (Ao*xi_hd - Lh*xi_hd(0)+Bo)*dt;
            // Adaptive Law
            if(ibvs_int_reset_thrust)
            {
                vartheta_h1(0) = 0.0f;
                vartheta_h1(1) = 0.0f;
                vartheta_h2(0) = 0.0f;
                vartheta_h2(1) = 0.0f;
                ibvs_int_reset_thrust = false;
            }
            else
            {
                math::Vector<2> vartheta_h1_update(varpi_h1*(-_params.gamma1_h*e_sh));
                math::Vector<2> vartheta_h2_update(varpi_h2*(_params.gamma2_h*delta_h2));
                //TODO check the bound
                vartheta_h1 = projector(vartheta_h1_update,vartheta_h1,0.1f)*dt;
                vartheta_h2 = projector(vartheta_h2_update,vartheta_h2,0.1f)*dt;
            }

            _att_sp_ibvs.valid += ((uint8_t)4);

            //TODO check IBVS update
//            _ibvs_state.hat_e_sh = hat_e_sh;
//            _ibvs_state.hat_v_sh = hat_v_sh;
//            _ibvs_state.c_g_hat = C_g_hat;
//            _ibvs_state.valid += ((uint8_t)4);
        }
        else
        {
//            C_g_hat_update = 0.0f;
//            _att_sp_ibvs.thrust = 0.0f;
//            _ibvs_state.valid = 0;
        }

        // for yaw motion
        if(isValid(&_img_feature,3))
        {
//            warnx("S4 is valid");
            float alpha = _img_feature.s[3];
            if (alpha>0.4f) alpha  = 0.4f;
            if (alpha<-0.4f) alpha = -0.4f;
            _att_sp_ibvs.yaw_body = euler_angles(2) + _params.k_psi*alpha;
            //TODO how to wrap up yaw motion?
//            _att_sp_ibvs.yaw_body += _params.k_psi*0.2*alpha*dt;
            _att_sp_ibvs.valid += ((uint8_t)8);
//            if(t>t_print)
//            {
//                t_print += 3000000;
//                warnx("alpha: %.4f, yaw_cur:%.4f, k_psi:%.4f, alpha:%.4f, yaw_ref: %.4f"
//                  ,(double)alpha, (double)euler_angles(2),(double)_params.k_psi,(double)(_params.k_psi*alpha),
//                      (double)_att_sp_ibvs.yaw_body);
//            }
        }
        else
        {
            _att_sp_ibvs.yaw_body = 0;
        }





        if (prev_nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
        {
//            warnx("here");
        }

        if(_att_sp_ibvs_pub_fd != nullptr){
            orb_publish(ORB_ID(vehicle_image_attitude_setpoint),_att_sp_ibvs_pub_fd, &_att_sp_ibvs);
        }
        else{
            orb_advertise(ORB_ID(vehicle_image_attitude_setpoint),&_att_sp_ibvs);
        }

        if(_ibvs_state_pub_fd != nullptr)
        {
            orb_publish(ORB_ID(ibvs_state),_ibvs_state_pub_fd, &_ibvs_state);
        }
        else{
            orb_advertise(ORB_ID(ibvs_state),&_ibvs_state);
        }
//        perf_count(_loop_perf);
        prev_nav_state = _vehicle_status.nav_state;
    }


    _ibvs_task = -1;
    _exit(0);
}


void MulticopterOPFIBVS::status()
{
//    warnx("IBVS Gain: IBVS_IZ_P %.4f",(double)_params.ibvs_kz);
    warnx("centeroid features valid : %d", _img_feature.valid);
    warnx("\t s1 s2 s3 s4 s5: %2.3f %2.3f %2.3f %2.3f %2.3f",(double)_img_feature.s[0], (double)_img_feature.s[1],
            (double)_img_feature.s[2], (double)_img_feature.s[3], (double)_img_feature.s[4]);
    warnx("Reference roll pitch yaw thrust: %8.4f %8.4f %8.4f %8.4f",(double)_att_sp_ibvs.roll_body,
          (double)_att_sp_ibvs.pitch_body,(double)_att_sp_ibvs.yaw_body, (double)_att_sp_ibvs.thrust);
//    warnx("e_sh:%.4f, hat_esh:%.4f, tilde_esh:%.4f, hat_vsh:%.4f, C_ghat_update: %.4f"
//          , (double)e_sh, (double)hat_e_sh, (double)tilde_xi_hy1, (double)hat_v_sh, (double)C_g_hat_update);
//    warnx("e_sl1:%.4f, hat_esl1:%.4f, tilde_esl1:%.4f, hat_vsl1:%.4f, eta_1_update: %.4f"
//          , (double)e_sl1, (double)hat_e_sl1, (double)tilde_e_sl1, (double)hat_v_sl1, (double)eta_e1_update);
//    warnx("e_sl2:%.4f, hat_esl2:%.4f, tilde_esl2:%.4f, hat_vsl2:%.4f, eta_2_update: %.4f"
//          , (double)e_sl2, (double)hat_e_sl2, (double)tilde_e_sl2, (double)hat_v_sl2, (double)eta_e2_update);
}

/**
 * Multicopter IBVS control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_ibvs_adap_main(int argc, char *argv[]);


int mc_ibvs_adap_main(int argc, char *argv[])
{
    if (argc < 2)
            errx(1, "missing command usage: mc_ibvs {start|stop|status}");

    if (!strcmp(argv[1], "start")){

        if(ibvs::g_control != nullptr)
            errx(1, "already running");

        ibvs::g_control = new MulticopterOPFIBVS;

        if (ibvs::g_control == nullptr)
            errx(1, "alloc failed");

        if (OK != ibvs::g_control->start()) {
                    delete ibvs::g_control;
                    ibvs::g_control = nullptr;
                    err(1, "start failed");
                }
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (ibvs::g_control == nullptr)
            warnx("not running");

        delete ibvs::g_control;
        ibvs::g_control = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (ibvs::g_control) {
		ibvs::g_control->status();
        }
        else{
            warnx("not running");
        }
	exit(0);
    }

    warnx("unrecognized command");
    return 1;
}


bool isValid(struct image_features_s *image_features, int n)
{
    if (image_features==NULL) return false;
    if (n<0||n>=MAX_FEATURES) return false;
    if (image_features->valid & (1 << n)) return true;
    return false;
}

//TODO improve the function to constant
//math::Vector<3> projector(math::Vector<3> input, math::Vector<3> estimate, float bound, float epsilon){
//    math::Vector<3> update(0.0f,0.0f,0.0f);
//    update = input*bound + estimate*epsilon;
//    return update;
//}

math::Vector<2> projector(math::Vector<2> input, math::Vector<2> estimate, float bound, float epsilon){
    math::Vector<2> update(0.0f,0.0f);
    update = input*bound + estimate*epsilon;
    return update;
}
