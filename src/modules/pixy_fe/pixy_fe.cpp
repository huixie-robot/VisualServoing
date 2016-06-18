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
 * @file pixy_fe.cpp
 * Feature Extraction from Pixy
 *
 */
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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
#include <systemlib/scheduling_priorities.h>
#include <systemlib/perf_counter.h>


#include <uORB/uORB.h>

//  Topics to subsrcibe
#include <uORB/topics/pixy.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>

// Topics to publish
#include <uORB/topics/image_features.h>


#define PI              3.14159f
#define PI_2            1.570795f
#define EPSILON         0.00005f
#define RAD2DEG         57.295827909f
#define MAX_SIGNATURES  3
#define MAX_POINTS_PER_SIGNATURE 5
#define MAX_FEATURES 8

static void setValid(struct image_features_s *image_features, int n, bool valid);
static bool isValid(struct image_features_s *image_features, int n);

typedef enum {POINTS=0,LINES,CENTEROID, SPHERICAL_POINTS,VIRTUAL_POINTS, VIRTUAL_LINES, OTHER=255} FeatureType;

class PIXY_FEATURE
{
public:
    /**
     * Constructor
     */
    PIXY_FEATURE();

    /**
     * Desctructor, also kills task
     */
     ~PIXY_FEATURE();
    /**
     * Start task.
     * @return      OK on success
     */
    int start();
	void status();
private:
    bool	_task_should_exit;		/**< if true, task should exit */
    int		_pixy_feature_task;             /**< task handle for task */

    /* Various subscription topic file descriptor */
    int     _pixy_sub_fd;
    int     _parameter_update_sub_fd;
    int     _att_sub_fd;
    /* Various handles for topics to be published */
    orb_advert_t    _pixy_image_features_fd;

    /* Various topics subscription variables definition */
    struct pixy_s                   _pixy_features;
    struct vehicle_attitude_s       _att;
    /* publication variables */
    struct image_features_s             _img_feature;


    struct {
        FeatureType feature_to_extract;
        float a_star;
        float roll_trim;
        float pitch_trim;
    }   _params;

    struct onePtIBVS_param_handles {
        param_t feature_to_extract;
        param_t a_star;
        float   roll_trim;
        float   pitch_trim;
    }   _params_handles;

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void		task_main() __attribute__((noreturn));

    /**
     * Check for changes in subscribed topics.
     */
    void		poll_subscriptions();

    /**
     * Update our local parameter cache.
     */
    int			parameters_update(bool force);

    /**
     * Extract the Points Feature.
     */
    void        points_feature_extraction();
    /**
      *
      */

    void        centeroid_feature_extraction();
    /**
     * Extract Spherical Points Feature.
     */
    void        spherical_points_extraction();


    /**
     * Extract the Line Feature.
     */
    void        lines_feature_extraction();

    /**
     * Extract the Virtual Line Feature.
     */
    void        virtual_lines_feature_extraction();


    const int x_centre;
    const int y_centre;
    const float focalLength;
//    const float sensor_size_x; //3.2 mm for pixy
//    const float sensor_size_y; //2.4 mm for pixy
//    const float coeff_for_x_axis;   //Converting Centerized X Coordinates of Pixels To milimeters
//    const float coeff_for_y_axis;    //Converting Centerized Y Coordinates of Pixels To milimeters
    float coeff_for_x_axis;
    float coeff_for_y_axis;

    float alpha[MAX_SIGNATURES];
    float rho[MAX_SIGNATURES];

    uint8_t num_lines;

    float alpha_Virtual[MAX_SIGNATURES];
    float rho_Virtual[MAX_SIGNATURES];

	perf_counter_t	_loop_perf;
	perf_counter_t _timeout_count;
};

namespace pixy_fe
{
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;
PIXY_FEATURE      *pixy_fe_ptr;
}


PIXY_FEATURE::PIXY_FEATURE():
    _task_should_exit(false),
    _pixy_feature_task(-1),
//    subscriptions
    _pixy_sub_fd(-1),
    _parameter_update_sub_fd(-1),
    _att_sub_fd(-1),
//    publish
//    _pixy_image_features_fd(-1),
//    intrinsic parameters
    x_centre(160),
    y_centre(100),
    focalLength(2.8),//in milimeters
    num_lines(0)
//    sensor_size_x(3.2),
//    sensor_size_y(2.4)
//    coeff_for_x_axis(0.5*3.2/((float)x_centre)),
//    coeff_for_y_axis(0.5*2.4/((float)y_centre))
{
    /* subscription variables initialization */
    memset(&_att, 0, sizeof(_att));
    memset(&_pixy_features,0,sizeof(_pixy_features));

    /* publication variables initialization */
    memset(&_img_feature,0,sizeof(_img_feature));

    coeff_for_x_axis = 0.5f*3.2f/((float)x_centre);
    coeff_for_y_axis = 0.5f*2.4f/((float)y_centre);

    /* Feature Selection Parameters */
    _params_handles.feature_to_extract 	=	param_find("FEATURE_TYPE");
    _params_handles.a_star              =   param_find("A_STAR");
    _params_handles.roll_trim           =   param_find("TRIM_ROLL");
    _params_handles.pitch_trim          =   param_find("TRIM_PITCH");
    /* fetch initial parameter values */
    parameters_update(true);
    _pixy_image_features_fd = orb_advertise(ORB_ID(image_features),&_img_feature);

    _loop_perf = perf_alloc(PC_INTERVAL, "pixy_fe");
    _timeout_count = perf_alloc(PC_COUNT, "pixy_fe timeout");
}

PIXY_FEATURE::~PIXY_FEATURE(){

    if(_pixy_feature_task !=-1){
        _task_should_exit = true;
        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do{
            usleep(20000);
            if(++i > 50){
                task_delete(_pixy_feature_task);
                break;
            }
        }while(_pixy_feature_task !=-1);
    }
    pixy_fe::pixy_fe_ptr = nullptr;
}

int
PIXY_FEATURE::parameters_update(bool force)
{
    bool updated = false;
    struct parameter_update_s param_upd;
    orb_check(_parameter_update_sub_fd, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(parameter_update), _parameter_update_sub_fd, &param_upd);
    }
    if (updated || force){
        param_get(_params_handles.feature_to_extract, &(_params.feature_to_extract));
        param_get(_params_handles.a_star, &(_params.a_star));
        param_get(_params_handles.roll_trim,&(_params.roll_trim));
        param_get(_params_handles.pitch_trim,&(_params.pitch_trim));
    }
    return OK;
}


int
PIXY_FEATURE::start()
{
    ASSERT(_pixy_feature_task == -1);

    //TODO check the priority
    /* start the task */
    _pixy_feature_task = px4_task_spawn_cmd("pixy_fe",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_SLOW_DRIVER-1,
                       1024,
                       (main_t)&PIXY_FEATURE::task_main_trampoline,
                       nullptr);

    if (_pixy_feature_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
PIXY_FEATURE::task_main_trampoline(int argc, char *argv[])
{
    pixy_fe::pixy_fe_ptr->task_main();
}

void
PIXY_FEATURE::poll_subscriptions()
{
    bool updated;

    orb_check(_att_sub_fd, &updated);
    if (updated)
        orb_copy(ORB_ID(vehicle_attitude), _att_sub_fd, &_att);

    orb_check(_pixy_sub_fd, &updated);
    if(updated)
        orb_copy(ORB_ID(pixy),_pixy_sub_fd,&_pixy_features);
}

void
PIXY_FEATURE::task_main()
{
    warnx("started");
    /* subscription */
    _pixy_sub_fd             = orb_subscribe(ORB_ID(pixy));
    _att_sub_fd              = orb_subscribe(ORB_ID(vehicle_attitude));
    _parameter_update_sub_fd = orb_subscribe(ORB_ID(parameter_update));

    parameters_update(true);

    /* get an initial update for all sensor and status data */
    poll_subscriptions();

    struct pollfd fds[1];
    fds[0].fd = _pixy_sub_fd;
    fds[0].events = POLLIN;

//    hrt_abstime t_prev = 0;

    while(!_task_should_exit){
        int poll_ret = poll(fds,(sizeof(fds) / sizeof(fds[0])),500);

        /* this is undesirable but not much we can do */
//        if (poll_ret < 0) {
//            warn("poll error %d, %d", poll_ret, errno);
//            continue;
//        }

        if(poll_ret==0 || poll_ret<0)
        {
            _img_feature.valid = 0;
            orb_publish(ORB_ID(image_features),_pixy_image_features_fd, &_img_feature);
//            if(_pixy_image_features_fd>0){
//                    orb_publish(ORB_ID(image_features),_pixy_image_features_fd, &_img_feature);
//                }
//                else{
//                    _pixy_image_features_fd = orb_advertise(ORB_ID(image_features),&_img_feature);
//                }
            continue;
        }


        poll_subscriptions();
        parameters_update(false);

        switch(_params.feature_to_extract)
        {
        case POINTS:
            points_feature_extraction();
            break;
        case LINES:
            lines_feature_extraction();
            break;
        case CENTEROID:
            centeroid_feature_extraction();
            break;
        case SPHERICAL_POINTS:
            spherical_points_extraction();
            break;
        case VIRTUAL_POINTS:
            // TODO write the function for VIRTUAL_POINTS
            break;
        case 5:
            // TODO write the function for VIRTUAL_LINES
            lines_feature_extraction();
            virtual_lines_feature_extraction();
            break;
        case OTHER:
            _img_feature.valid = 0;
            break;
        default:
            _img_feature.valid = 0;
            warnx("Unknown Features");
        }


        orb_publish(ORB_ID(image_features),_pixy_image_features_fd, &_img_feature);
//        if(_pixy_image_features_fd>0){
//            orb_publish(ORB_ID(image_features),_pixy_image_features_fd, &_img_feature);
//        }
//        else{
//            _pixy_image_features_fd = orb_advertise(ORB_ID(image_features),&_img_feature);
//        }
        perf_count(_loop_perf);
        //usleep(200000);//50Hz
    }


    _pixy_feature_task = -1;
    _exit(0);

}


void PIXY_FEATURE::points_feature_extraction(){
    int x_coord, y_coord;
//    float coeff_for_x_axis  = 0.5*sensor_size_y/((float)y_centre);
//    float coeff_for_y_axis  = 0.5*sensor_size_x/((float)x_centre);
    for(int i=0;i<MAX_SIGNATURES;i++)
    {
        x_coord = 0;
        y_coord = 0;
        if(_pixy_features.num_of_sig_blks[i]>0)
        {
            for(int j=0;j<_pixy_features.num_of_sig_blks[i];j++)
            {
                x_coord += _pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE + j];
                y_coord += _pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE + j];
            }
//            _img_feature.s[2*i]     = ((((float)x_coord/(float)_pixy_features.num_of_sig_blks[i]) - (float)x_centre))/(float)x_centre;
//            _img_feature.s[2*i+1]   = ((((float)y_coord/(float)_pixy_features.num_of_sig_blks[i]) - (float)y_centre))/(float)y_centre;
            _img_feature.s[2*i]     = ((((float)x_coord/(float)_pixy_features.num_of_sig_blks[i]) - (float)x_centre))/(float)x_centre;
            _img_feature.s[2*i+1]   = ((((float)y_coord/(float)_pixy_features.num_of_sig_blks[i]) - (float)y_centre))/(float)y_centre;
            setValid(&_img_feature,2*i,true);
            setValid(&_img_feature,2*i + 1,true);
        }
        else
        {
            setValid(&_img_feature,2*i,false);
            setValid(&_img_feature,2*i + 1,false);
        }
    }
}

void PIXY_FEATURE::centeroid_feature_extraction(){
    float beta;
    float x_real, y_real;
    float x_virtual[10], y_virtual[10]; //TODO temporarily set at 10 pts

    float x_g = 0;
    float y_g = 0 ;
    int num_pts = 0;

    for(int i=0;i<MAX_SIGNATURES;i++)
    {
        if(_pixy_features.num_of_sig_blks[i]>0)
        {
            for(int j=0;j<_pixy_features.num_of_sig_blks[i];j++)
            {
                float roll  = _att.roll - _params.roll_trim;
                float pitch = _att.pitch- _params.pitch_trim;

                x_real = ((float)(_pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE + j] - x_centre))*((float)coeff_for_x_axis);
                y_real = ((float)(_pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE + j] - y_centre))*((float)coeff_for_y_axis);
                beta = focalLength/(-x_real*sinf(roll) + y_real*sinf(roll)*cosf(pitch) + focalLength*cosf(roll)*cosf(pitch));
                x_virtual[j+num_pts] = beta*(cosf(pitch)*x_real + sinf(roll)*sinf(pitch)*y_real + focalLength*cosf(roll)*sinf(pitch));
                y_virtual[j+num_pts] = beta*(cosf(roll)*y_real - sinf(roll)*focalLength);
                x_g += x_virtual[j+num_pts];
                y_g += y_virtual[j+num_pts];
            }
            num_pts += _pixy_features.num_of_sig_blks[i];
        }
    }

    float miu02 = 0;
    float miu20 = 0;
    float miu11 = 0;

//    if(num_pts<0||num_pts==0)
//    {
//        _img_feature.valid = 0;
//    }
//    else if(num_pts<2)
//    {
//        x_g = x_g/((float)(num_pts));
//        y_g = y_g/((float)(num_pts));
//        _img_feature.s[0] = x_g;
//        _img_feature.s[1] = y_g;
//        setValid(&_img_feature,0,true);
//        setValid(&_img_feature,1,true);
//        setValid(&_img_feature,2,false);
//        setValid(&_img_feature,3,false);
//        setValid(&_img_feature,4,false);
//    }
    if(num_pts == 2)
    {
        x_g = x_g/((float)(num_pts));
        y_g = y_g/((float)(num_pts));

        float a;
        for(int i=0;i<num_pts;i++){
            miu20 += (x_virtual[i] - x_g)*(x_virtual[i] - x_g);
            miu02 += (y_virtual[i] - y_g)*(y_virtual[i] - y_g);
            miu11 += (x_virtual[i] - x_g)*(y_virtual[i] - y_g);
        }
        a = miu02 + miu20;
//        _params.a_star;
//        _img_feature.s[2] = sqrt(1.0/(miu02+miu20));
        _img_feature.s[2] = sqrt(_params.a_star/a);
        _img_feature.s[0] = x_g * _img_feature.s[2];
        _img_feature.s[1] = y_g * _img_feature.s[2];
        _img_feature.s[3] = 0.5f*atan2f(2*miu11,miu20-miu02);
//        _img_feature.s[4] = a;

    _img_feature.s[4] = (float)_pixy_features.x_coord[0*MAX_POINTS_PER_SIGNATURE + 0];
    _img_feature.s[5] = (float)_pixy_features.y_coord[0*MAX_POINTS_PER_SIGNATURE + 0];
    _img_feature.s[6] = (float)_pixy_features.x_coord[0*MAX_POINTS_PER_SIGNATURE + 1];
    _img_feature.s[7] = (float)_pixy_features.y_coord[0*MAX_POINTS_PER_SIGNATURE + 1];

	for (int i=0;i<4;i++)
	        setValid(&_img_feature,i,true);
    }
    else
    {
        _img_feature.valid = 0;
    }
}
void PIXY_FEATURE::spherical_points_extraction(){
    float xs_coord = 0, ys_coord = 0, zs_coord = 0;

    for(int i=0;i<MAX_SIGNATURES;i++)
    {
        xs_coord = 0;
        ys_coord = 0;
        zs_coord = 0;
        if(_pixy_features.num_of_sig_blks[i]>0)
        {
            for(int j=0;j<_pixy_features.num_of_sig_blks[i];j++)
            {
                float x,y,radius;
                x = ((float)(_pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE + j] - x_centre))*coeff_for_x_axis;
                y = ((float)(_pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE + j] - y_centre))*coeff_for_y_axis;
                radius = sqrt(x*x+y*y+focalLength*focalLength);
                xs_coord += x/radius;
                ys_coord += y/radius;
                zs_coord += focalLength/radius;
                _img_feature.s[3] = radius;
            }

            _img_feature.s[3*i]     = xs_coord/(float)_pixy_features.num_of_sig_blks[i];
            _img_feature.s[3*i+1]   = ys_coord/(float)_pixy_features.num_of_sig_blks[i];
            _img_feature.s[3*i+2]   = zs_coord/(float)_pixy_features.num_of_sig_blks[i];
		

            setValid(&_img_feature,3*i,true);
            setValid(&_img_feature,3*i + 1,true);
            setValid(&_img_feature,3*i + 2,true);
        }
        else
        {
            setValid(&_img_feature,3*i,false);
            setValid(&_img_feature,3*i + 1,false);
            setValid(&_img_feature,3*i + 2,false);
        }
    }
}

void PIXY_FEATURE::lines_feature_extraction(){

    float alpha_tmp;
    float sumX, sumY, sumXsquare, sumYsquare, sumXY, x_avg, y_avg;
    float slopeX_num, slopeY_num, slope_den;
    float slopeX, slopeY, intersetX, intersetY;

    num_lines = 0;
    for(int i=0;i<MAX_SIGNATURES;i++)
    {
        if(_pixy_features.num_of_sig_blks[i]>1)
        {
            sumX = 0;
            sumY = 0;
            sumXsquare = 0;
            sumYsquare = 0;
            sumXY = 0;
            float x_tmp, y_tmp;
            for(int j=0;j<_pixy_features.num_of_sig_blks[i];j++)
            {
                x_tmp        = ((float)(_pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE + j] - x_centre))*coeff_for_x_axis;
                y_tmp        = ((float)(_pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE + j] - y_centre))*coeff_for_y_axis;
                sumX        += x_tmp;
                sumY        += y_tmp;
                sumXsquare  += x_tmp*x_tmp;
                sumYsquare  += y_tmp*y_tmp;
                sumXY       += x_tmp*y_tmp;

                //HACK TO SEE EXTRA DATA - NEEDS TO BE REMOVED - TODO
                _img_feature.s[j*2]=x_tmp;
                _img_feature.s[j*2+1]=y_tmp;
            }

            slopeX_num  = (float)(sumXsquare) - (float)(sumX*sumX)/((float)(_pixy_features.num_of_sig_blks[i]));
            slopeY_num  = (float)(sumYsquare) - (float)(sumY*sumY)/((float)(_pixy_features.num_of_sig_blks[i]));
            slope_den   = (float)(sumXY) - ((float)(sumX*sumY))/((float)(_pixy_features.num_of_sig_blks[i]));
            x_avg       = (float)sumX/((float)_pixy_features.num_of_sig_blks[i]);
            y_avg       = (float)sumY/((float)_pixy_features.num_of_sig_blks[i]);

            if(fabsf(slopeX_num)>fabsf(slopeY_num))
            {
                slopeX      = slope_den/slopeX_num;

                intersetY   = y_avg - slopeX*x_avg;
//                alpha_tmp   = atan(-coeff_for_alpha*slopeX);
                alpha_tmp   = atan(-slopeX);
                if(fabsf(fabsf(alpha_tmp) - PI_2) < EPSILON)
                    alpha_tmp = -PI_2;
                rho[i]      = intersetY*cosf(alpha_tmp);
            }
            else
            {
                slopeY      = slope_den/slopeY_num;
                intersetX  = x_avg - slopeY*y_avg;
//                alpha_tmp    = atan(-coeff_for_alpha/slopeY);
                alpha_tmp    = atanf(-1.0f/slopeY);
                if(fabsf(fabsf(alpha_tmp) - PI_2) < EPSILON)
                    alpha_tmp = -PI_2;
                rho[i]      = intersetX*sinf(alpha_tmp);
            }

            alpha[i]    = alpha_tmp;
            _img_feature.s[2*i]     = alpha[i];
            _img_feature.s[2*i+1]   = rho[i];
            setValid(&_img_feature,2*i,true);
            setValid(&_img_feature,2*i + 1,true);
            num_lines++;
        }
        else
        {
            setValid(&_img_feature,2*i,false);
            setValid(&_img_feature,2*i + 1,false);
        }
    }
}


void PIXY_FEATURE::virtual_lines_feature_extraction()
{

    math::Matrix<3, 3> B;
    math::Matrix<3, 3> Binv;
    math::Matrix<3, 3> Rthephi;
    math::Vector<3> real_line;
    math::Vector<3> virtual_line;
    B.identity();
    Binv.identity();
    B.data[2][2] = focalLength;
    Binv.data[2][2] = 1/focalLength;
//    alpha_Virtual;
    float roll  = _att.roll - _params.roll_trim;
    float pitch = _att.pitch- _params.pitch_trim;
//    Rthephi.from_euler(_att.roll,_att.pitch,0.0);
    Rthephi.from_euler(roll,pitch,0.0);

    float rho_g = 0.0f;
    float alpha_g = 0.0f;
    float miu20 = 0.0f;

    if(_img_feature.valid>0)
    {
        for(int i=0;i<MAX_SIGNATURES;i++)
        {
            if(isValid(& _img_feature,2*i))
            {
                real_line.data[0] = sin(alpha[i]);
                real_line.data[1] = cos(alpha[i]);
                real_line.data[2] = -rho[i];
                virtual_line = ((B*Rthephi)*Binv)*real_line;
                //TODO confirm the difference between atan2f atan2;
                alpha_Virtual[i] = atan2f(virtual_line.data[0],virtual_line.data[1]);
                rho_Virtual[i] = -virtual_line.data[2]/sqrtf(virtual_line.data[0]*virtual_line.data[0]+ virtual_line.data[1]*virtual_line.data[1]);
                rho_g   += rho_Virtual[i];
                alpha_g += alpha_Virtual[i];
            }
        }

        rho_g   = rho_g/((float)num_lines);
        alpha_g = alpha_g/((float)num_lines);

        _img_feature.valid = 0; //reset the valid here
        if(num_lines ==0)
        {
            _img_feature.s[0]     = 0.0f;
            _img_feature.s[1]     = 0.0f;
        }
        if(num_lines<2)
        {
            _img_feature.s[0]     = alpha_g;
            _img_feature.s[1]     = rho_g;
            setValid(&_img_feature,0,true);
            setValid(&_img_feature,1,true);
        }
        else
        {
            _img_feature.s[0]   = alpha_g;  //angle
            setValid(&_img_feature,0,true);
            for(int i=0;i<MAX_SIGNATURES;i++)
            {
                if(isValid(& _img_feature,2*i))
                {
                    miu20 += (rho_Virtual[i] - rho_g)*(rho_Virtual[i] - rho_g);
                }
            }
            _img_feature.s[2] = sqrt(_params.a_star/miu20); // height
            setValid(&_img_feature,2,true);
            _img_feature.s[1] = rho_g*_img_feature.s[2];    //lateral
            setValid(&_img_feature,1,true);
            _img_feature.s[3] = miu20;                      //for detect desired height
            setValid(&_img_feature,3,true);
        }
    }
}

void PIXY_FEATURE::status()
{
    warnx("Received Data From Pixy");
    warnx("No of Sig1:%d, Sig2:%d, Sig3: %d",_pixy_features.num_of_sig_blks[0],_pixy_features.num_of_sig_blks[1],_pixy_features.num_of_sig_blks[2]);
    for(int i=0;i<MAX_SIGNATURES;i++)
    {
        if(_pixy_features.num_of_sig_blks[i]>0)
        {
            warnx("\t%d sig[%d] blocks",_pixy_features.num_of_sig_blks[i],i);
            for(int j=0;j<_pixy_features.num_of_sig_blks[i];j++)
                warnx("\t\tx:%d y:%d width:%d height:%d", _pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE + j], _pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE+j], _pixy_features.width[i*MAX_POINTS_PER_SIGNATURE + j], _pixy_features.height[i*MAX_POINTS_PER_SIGNATURE + j]);
        }
    }
    switch(_params.feature_to_extract)
    {
    case POINTS:
        warnx("The Feature Type is Points");
        break;
    case LINES:
        warnx("The Feature Type is Lines");
        break;
    case CENTEROID:
        warnx("The Feature Type is Centeroid");
        break;
    case SPHERICAL_POINTS:
        warnx("The Feature Type is spherical points");
        break;
    case VIRTUAL_LINES:
        warnx("The Feature Type is Virtual Lines");
        break;
    case OTHER:
        warnx("The Feature Type is Other");
        break;
    default:
        warnx("Unknown Features");
    }

    warnx("Feature Valid %d",_img_feature.valid);
	for (int j=0;j<MAX_FEATURES;j++)
            warnx("s%d:%.4f",j+1,(double)_img_feature.s[j]);
   warnx("Attitude (%.3f,%.3f,%.3f)",(double)_att.roll,(double)_att.pitch,(double)_att.yaw);

}

/**
 * Pixy Image Feature Extraction app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int pixy_fe_main(int argc, char *argv[]);


int pixy_fe_main(int argc, char *argv[])
{
    if (argc < 2)
            errx(1, "missing command usage: pixy_fe {start|stop|status}");

    if (!strcmp(argv[1], "start")){

        if(pixy_fe::pixy_fe_ptr!= nullptr)
            errx(1, "already running");

        pixy_fe::pixy_fe_ptr = new PIXY_FEATURE;

        if (pixy_fe::pixy_fe_ptr == nullptr)
            errx(1, "alloc failed");

        if (OK != pixy_fe::pixy_fe_ptr->start()) {
                    delete pixy_fe::pixy_fe_ptr;
                    pixy_fe::pixy_fe_ptr = nullptr;
                    err(1, "start failed");
                }
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (pixy_fe::pixy_fe_ptr == nullptr)
            warnx("not running");

        delete pixy_fe::pixy_fe_ptr;
        pixy_fe::pixy_fe_ptr = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (pixy_fe::pixy_fe_ptr) {
        pixy_fe::pixy_fe_ptr->status();
        }
        else{
            warnx("not running");
        }
        exit(0);
    }

    warnx("unrecognized command");
    return 1;
}


static void setValid(struct image_features_s *image_features, int n, bool valid)
{
    if (image_features!=NULL) {
        if (n>=0&&n<MAX_FEATURES) {
            if (valid) image_features->valid |= 1 << n;
            else image_features->valid &= ~(1 << n);
        }
    }
}

static bool isValid(struct image_features_s *image_features, int n)
{
    if (image_features==NULL) return false;
    if (n<0||n>=MAX_FEATURES) return false;
    if (image_features->valid & (1 << n)) return true;
    return false;
}
