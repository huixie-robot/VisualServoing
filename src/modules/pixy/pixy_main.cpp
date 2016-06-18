/**
 * @file pixy_main.cpp
 *
 * @author Hui Xie and Geoff Fink
 */

#include <fcntl.h>
#include <nuttx/config.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#include <stdlib.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <float.h>
#include <errno.h>
#include <termios.h>
#include "pixy.hpp"

static bool thread_should_exit = false;     /**< Deamon exit flag */
static int  daemon_task;             /**< Handle of deamon task / thread */
static Pixy *pixy = NULL;

int mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original);
/**
 * Deamon management function.
 */
extern "C" __EXPORT int pixy_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int pixy_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

    warnx("usage: pixy {start|stop|status} [-d <devicename>] [-b <baud rate>] [-p <print received data>]");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int pixy_main(int argc, char *argv[])
{

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (pixy!=NULL) {
			warnx("already running");
			/* this is not an error */
            exit(0);
		}

		thread_should_exit = false;
        //TODO choose appropriate priority from : "./src/modules/systemlib/scheduling_priorities.h"
//        SCHED_PRIORITY_WATCHDOG
//        SCHED_PRIORITY_ACTUATOR_OUTPUTS
//        SCHED_PRIORITY_POSITION_CONTROL
//        SCHED_PRIORITY_SLOW_DRIVER
        daemon_task = px4_task_spawn_cmd("pixy",
					 SCHED_DEFAULT,
                    // SCHED_PRIORITY_SLOW_DRIVER,
		SCHED_PRIORITY_POSITION_CONTROL-1,
                     2048,
					 pixy_thread_main,
                     (argv) ? (char **)&argv[2] : (char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		while (pixy!=NULL) {
			warnx(".");
			usleep(1000000);
		}
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (pixy!=NULL) {
			pixy->print();
			exit(0);

		} else {
			warnx("not started");
			exit(1);
		}
	}

	usage("unrecognized command");
	exit(1);
}

int mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original)
{
	/* process baud rate */
	int speed;

	switch (baud) {

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
        warnx("ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n\n", baud);
		return -EINVAL;
	}

	/* open uart */
	warnx("UART is %s, baudrate is %d\n", uart_name, baud);
	int uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
    if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERROR get termios config %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
    uart_config.c_cflag |= CS8;         //8 data bit
    uart_config.c_cflag &= ~PARENB;     //no parity
    uart_config.c_cflag &= ~CSTOPB;     //1 stop bit

    /* Set baud rate */
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        warnx("ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
        close(uart);
        return -1;
    }


    if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		warnx("ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}




int pixy_thread_main(int argc, char *argv[])
{

	warnx("starting");

    int baud = 57600;
    char const *uart_name = "/dev/ttyS6";
	struct termios uart_config_original;
    bool print_data_flag = false;

	int ch;
    while ((ch = getopt(argc, argv, "b:d:p")) != EOF) {
		switch (ch) {
		case 'b':
			baud = strtoul(optarg, NULL, 10);
			if (baud == 0)
				errx(1, "invalid baud rate '%s'", optarg);
			break;

		case 'd':
			uart_name = optarg;
			break;
        case 'p':   //  only print all received data
            print_data_flag = true;
            break;
		default:
			usage("unrecognized command");
		}
	}

    int uart_fd = mavlink_open_uart(baud,uart_name,&uart_config_original);
	if (uart_fd < 0) err(1, "could not open %s", uart_name);
	
    pixy = new Pixy(uart_fd, print_data_flag);

	while (!thread_should_exit) {
        pixy->update();
//        usleep(1000);
	}

	delete(pixy);
	pixy=NULL;
	warnx("terminated.");
	return 0;
}

