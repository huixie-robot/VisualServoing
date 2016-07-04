/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <mathlib/mathlib.h>

#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/home_position.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int px4_daemon_tpl_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon_tpl {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int px4_daemon_tpl_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon_tpl",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 1204,
						 px4_daemon_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int px4_daemon_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	thread_running = true;
	
	orb_advert_t mocap_pub;
	struct att_pos_mocap_s _mocap;
	int _home_pos_sub = -1;
	struct home_position_s hom_pos;
	bool updated;
	
	
	mocap_pub = orb_advertise(ORB_ID(att_pos_mocap), &_mocap);
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	
	memset(&_mocap, 0, sizeof(_mocap));
	_mocap.q[0] = 1.0f;
	_mocap.x = 1.0f;
	_mocap.y = 1.0f;
	_mocap.z = -2.0f;
	
	
	
	while (!thread_should_exit) {
		float tmp;
		tmp = sinf(_mocap.timestamp_boot/1000000.0f);
		_mocap.timestamp_boot = hrt_absolute_time();
		_mocap.x = 1.0f ;
		_mocap.y = 1.0f + tmp*0.0f;
		_mocap.z = -1.5f;
		
		orb_check(_home_pos_sub, &updated);
		if(updated){
			orb_copy(ORB_ID(home_position),_home_pos_sub,&hom_pos);
			warnx("hom_pos: z: %.6f", (double) hom_pos.alt);
		}
		
		orb_publish(ORB_ID(att_pos_mocap),mocap_pub,&_mocap);
/*		warnx("Publish mocap data");*/
		usleep(100000);
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
