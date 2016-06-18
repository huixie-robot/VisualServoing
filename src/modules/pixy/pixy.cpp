/**
 * @file rpi.cpp
 *
 * Vicon
 */



#include "pixy.hpp"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>

//#define PI          3.14159f
//#define PI_2        1.570795f
//#define EPSILON     0.00005f
//#define RAD2DEG     57.295827909f


#define PIXY_BLOCK_HEADER_FIRST_HALF    0x55
#define PIXY_BLOCK_HEADER_LAST_HALF     0xaa
#define PIXY_BLOCK_HEADER               0xaa55

#define PIXY_PARSE_STATUS_HEAD          0x00
#define PIXY_PARSE_STATUS_CHECKSUM      0x01
#define PIXY_PARSE_STATUS_SIGNATURE     0x02
#define PIXY_PARSE_STATUS_X             0x03
#define PIXY_PARSE_STATUS_Y             0x04
#define PIXY_PARSE_STATUS_WIDTH         0x05
#define PIXY_PARSE_STATUS_HEIGHT        0x06

#define PIXY_PARSE_WORD_COMPLETE    0x00
#define PIXY_PARSE_WORD_HALF        0x01


#define MAX_SIGNATURES 3
#define MAX_POINTS_PER_SIGNATURE 5

Pixy::Pixy(int uart_fd, bool print_data_flag):
    print_count(0),
    print_received_data_flag(false),
    new_correct_frame_flag(false)
{
    count_empty     = perf_alloc(PC_INTERVAL, "pixy_im_empty");
    count_CS_error  = perf_alloc(PC_INTERVAL, "pixy_im_csErr");
    count_success   = perf_alloc(PC_INTERVAL, "pixy_im_succ");
    count_extra     = perf_alloc(PC_INTERVAL, "pixy_im_extra");

    print_received_data_flag = print_data_flag;
    memset(&pixy_features, 0, sizeof(pixy_features));
    memset(&blk,0,sizeof(blk));
    memset(&parse_status,0,sizeof(parse_status));
	_fds[0].fd = uart_fd;
	_fds[0].events = POLLIN;
    memset(&parse_status, 0, sizeof(parse_status));
    word_parse_status = 0;
    pixy_pub = orb_advertise(ORB_ID(pixy), &pixy_features);
}

Pixy::~Pixy()
{
//	close(image_features_pub);
//    close(pixy_pub);
}

uint8_t Pixy::pixy_parse_get_word(uint8_t data)
{
    if(word_parse_status == PIXY_PARSE_WORD_COMPLETE)
    {
        word_got = data;
        word_parse_status = PIXY_PARSE_WORD_HALF;
        return 0;
    }
    else if(word_parse_status == PIXY_PARSE_WORD_HALF)
    {
        uint16_t tmp;
        tmp = (uint16_t)data;
        word_got += tmp<<8;
        word_parse_status = PIXY_PARSE_WORD_COMPLETE;
        return 1;
    }
    else
        word_parse_status = PIXY_PARSE_WORD_COMPLETE;
    return 0;
}

int Pixy::pixy_parse_char(uint8_t data)
{
    switch(parse_status.block_status)
    {
    case PIXY_PARSE_STATUS_HEAD:            // Look for the header of a blob
	if(data==0) break;
        if(word_parse_status == PIXY_PARSE_WORD_COMPLETE)
        {
            if(data == PIXY_BLOCK_HEADER_FIRST_HALF)
            {
                word_parse_status = PIXY_PARSE_WORD_HALF;
            }
            else if (data==0) perf_count(count_empty);
            else perf_count(count_extra);
        }
        else if(word_parse_status == PIXY_PARSE_WORD_HALF)
        {
            if(data == PIXY_BLOCK_HEADER_LAST_HALF) // Header is found
            {
                parse_status.block_status = PIXY_PARSE_STATUS_CHECKSUM;
                word_parse_status = PIXY_PARSE_WORD_COMPLETE;
            }
            else if(data == PIXY_BLOCK_HEADER_FIRST_HALF)   // Start of a Header
                word_parse_status = PIXY_PARSE_WORD_HALF;
            else
                word_parse_status = PIXY_PARSE_WORD_COMPLETE;
            if (data==0) perf_count(count_empty);
        }
        break;

    case PIXY_PARSE_STATUS_CHECKSUM:        // Read the checksum
        if(pixy_parse_get_word(data))
        {
            if(word_got == PIXY_BLOCK_HEADER)   // new frame is coming
            {
                parse_status.block_status = PIXY_PARSE_STATUS_CHECKSUM;
                if((parse_status.num_of_blks>0) && (parse_status.error_blocks==0))
                {
                    print_count++;

                    for(int i=0;i<MAX_SIGNATURES;i++)
                    {
                        pixy_features.num_of_sig_blks[i] = parse_status.num_of_sig_blks[i];

                    }
                    new_correct_frame_flag = true;
                }
                parse_status.num_of_blks        = 0;
                parse_status.error_blocks       = 0;
                for(int i=0;i<MAX_SIGNATURES;i++)
                    parse_status.num_of_sig_blks[i]   = 0;

                if(new_correct_frame_flag)
                {
                    new_correct_frame_flag = false;
                    return 1;
                }
            }
            else
            {
                blk.checksum = word_got;
                parse_status.block_status = PIXY_PARSE_STATUS_SIGNATURE;
            }
        }
        break;

    case PIXY_PARSE_STATUS_SIGNATURE:           //Read the signature
        if(pixy_parse_get_word(data))
        {
            blk.signature = word_got;
            parse_status.block_status = PIXY_PARSE_STATUS_X;
        }
        break;

    case PIXY_PARSE_STATUS_X:                   // Read the x coordinate of the blobs
        if(pixy_parse_get_word(data))
        {
            blk.x = word_got;
            parse_status.block_status = PIXY_PARSE_STATUS_Y;
        }
        break;

    case PIXY_PARSE_STATUS_Y:                   // Read the y coordinate of the blobs
        if(pixy_parse_get_word(data))
        {
            blk.y = word_got;
            parse_status.block_status = PIXY_PARSE_STATUS_WIDTH;
        }
        break;

    case PIXY_PARSE_STATUS_WIDTH:               // Read the width of the blobs
        if(pixy_parse_get_word(data))
        {
            blk.width = word_got;
            parse_status.block_status = PIXY_PARSE_STATUS_HEIGHT;
        }
        break;

    case PIXY_PARSE_STATUS_HEIGHT:              // Read the heigth of the blobs
        if(pixy_parse_get_word(data))
        {
            blk.height = word_got;
            parse_status.block_status = PIXY_PARSE_STATUS_HEAD;
            uint16_t cs;
            cs = blk.height + blk.width + blk.y + blk.x + blk.signature;
            if(cs!=blk.checksum)         // check sum error
            {
                   perf_count(count_CS_error);
                   parse_status.error_blocks ++;
            }
            else
            {
                perf_count(count_success);
                parse_status.num_of_blks ++;

                int index = blk.signature - 1;

                pixy_features.x_coord[index*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[index]] = blk.x;
                pixy_features.y_coord[index*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[index]] = blk.y;
                pixy_features.width[index*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[index]] = blk.width;
                pixy_features.height[index*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[index]] = blk.height;

                parse_status.num_of_sig_blks[index]++;
//                if(blk.signature == 1)
//                {
//                    pixy_features.x_coord[0*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[0]] = blk.x;
//                    pixy_features.y_coord[0*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[0]] = blk.y;
//                    pixy_features.width[0*MAX_POINTS_PER_SIGNATURE + parse_status.num_of_sig_blks[0]]    = blk.width;
//                    pixy_features.height[0 + parse_status.num_of_sig_blks[0]]  = blk.height;
//                    //TODO avoid the dimension exceed the maximum capacity of the array;
//                    parse_status.num_of_sig_blks[0]++;
//                }
//                else if(blk.signature == 2)
//                {
//                   pixy_features.x_coord[5+ parse_status.num_of_sig_blks[1]] = blk.x;
//                   pixy_features.y_coord[5+ parse_status.num_of_sig_blks[1]] = blk.y;
//                   pixy_features.width[5 + parse_status.num_of_sig_blks[1]]    = blk.width;
//                   pixy_features.height[5 + parse_status.num_of_sig_blks[1]]  = blk.height;
//                   parse_status.num_of_sig_blks[1]++;
//                }
//                else if(blk.signature == 3)
//                {
//                    pixy_features.x_coord[10 + parse_status.num_of_sig_blks[2]] = blk.x;
//                    pixy_features.y_coord[10 + parse_status.num_of_sig_blks[2]] = blk.y;
//                    pixy_features.width[10 + parse_status.num_of_sig_blks[2]]    = blk.width;
//                    pixy_features.height[10 + parse_status.num_of_sig_blks[2]]  = blk.height;
//                    parse_status.num_of_sig_blks[2]++;
//                }
            }
        }
        break;
    default:
        parse_status.block_status = PIXY_PARSE_STATUS_HEAD;
        break;
    }
    return 0;
}

void Pixy::update()
{
    if (poll(_fds, POLLIN, timeout) > 0) {
        // non-blocking read. read may return negative values
        ssize_t nread = read(_fds[0].fd, _buf, sizeof(_buf));
        // if read failed, this loop won't execute
        for (ssize_t i = 0; i < nread; i++) {
            if(print_received_data_flag)
                print_serial_code(_buf[i]); // only need to print data for debug
            else
            {
                if (pixy_parse_char(_buf[i])){
                    orb_publish(ORB_ID(pixy), pixy_pub, &pixy_features);
                }
            }
        }
    }
    usleep(0);
}

void Pixy::print()
{
        warnx("%d Sig1, %d Sig2, %d Sig3",pixy_features.num_of_sig_blks[0],pixy_features.num_of_sig_blks[1],pixy_features.num_of_sig_blks[2]);
        for(int i=0;i<MAX_SIGNATURES;i++)
        {
            if(pixy_features.num_of_sig_blks[i]>0)
            {
                warnx("\t%d sig[%d] blocks",pixy_features.num_of_sig_blks[i],i);
                for(int j=0;j<pixy_features.num_of_sig_blks[i];j++)
                    warnx("\t\tx:%d y:%d width:%d height:%d", pixy_features.x_coord[i*MAX_POINTS_PER_SIGNATURE+j], pixy_features.y_coord[i*MAX_POINTS_PER_SIGNATURE+j], pixy_features.width[i*MAX_POINTS_PER_SIGNATURE+j], pixy_features.height[MAX_POINTS_PER_SIGNATURE*i+j]);
            }
        }
}

void Pixy::print_serial_code(uint8_t data)
{
    if((data==0x55)||(print_count>200))
    {
        printf("\n%x ", data);
        print_count = 0;
    }
    else
        printf("%x ", data);
    print_count++;
}
