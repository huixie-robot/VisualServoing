/**
 * @file rpi.hpp
 *
 * vicon
 */

#pragma once

#define MAX_SIGNATURES 3


#include <poll.h>
//#include <uORB/topics/image_features.h>
#include <uORB/topics/pixy.h>

#include <systemlib/err.h>
#include <systemlib/scheduling_priorities.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

/**
 * Raspberry Pi class
 */
class Pixy
{
public:
	/**
	 * Constructor
	 */
    Pixy(int uart_fd, bool print_data_flag);

	/**
	 * Deconstuctor
	 */

	virtual ~Pixy();

	/**
	 * The main callback function for the class
	 */
	void update();
	void print();

protected:

    uint16_t print_count;
    void print_serial_code(uint8_t data);       //Print all data received
    bool print_received_data_flag;

    perf_counter_t count_empty;
    perf_counter_t count_CS_error;
    perf_counter_t count_success;
    perf_counter_t count_extra;

//	orb_advert_t image_features_pub;
    orb_advert_t pixy_pub;
    struct pixy_s pixy_features;
    bool new_correct_frame_flag;
    static const int timeout = 1000;
    uint8_t _buf[40];
	struct pollfd _fds[1];

    struct Block
    {
      uint16_t checksum;
      uint16_t signature;
      uint16_t x;
      uint16_t y;
      uint16_t width;
      uint16_t height;
    };

    struct Pixy_parse_status
    {
        uint8_t block_status;
        uint8_t num_of_blks;        // number of blocks for one frame;
        uint8_t error_blocks;
        uint8_t num_of_sig_blks[MAX_SIGNATURES];
    };

    int pixy_parse_char(uint8_t data);

    struct Block blk;
    struct Pixy_parse_status parse_status;

    uint16_t word_got; //only use in pixy_parse_get_word(uint8_t data, uint16_t word);
    uint8_t word_parse_status;
    uint8_t pixy_parse_get_word(uint8_t data);

};
