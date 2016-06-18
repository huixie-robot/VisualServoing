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
#include <systemlib/scheduling_priorities.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
//#include <errno.h>
//#include <termios.h>
//#include "mc_vicon_pos_control.hpp"


/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_vicon_pos_control_main(int argc, char *argv[]);

class ViconControl
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
//    int vicon_sub;
//    int att_sub;
//    int image_att_sp_sub;
//    int control_mode_sub;
//    int vehicle_status_sub;
//    int param_sub;
//    int rc_channels_sub;
//    int manual_control_sp_sub;
//    int predictor_sub;
//    int R_bias_sub;


    void task_main();
};

namespace Vicon_Control
{
    ViconControl *g_control;
}

ViconControl::ViconControl(){

}

void ViconControl::task_main(){
    /*Do subscription */
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

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

void
ViconControl::task_main_trampoline(int argc, char *argv[])
{
    Vicon_Control::g_control->task_main();
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


