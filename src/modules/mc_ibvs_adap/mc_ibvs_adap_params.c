/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 */

#include <systemlib/param/param.h>

/*ROLL-> Y motion -> S2 Feature*/
PARAM_DEFINE_FLOAT(IBVS_K1_R, 0.6f);
PARAM_DEFINE_FLOAT(IBVS_D1_R, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_K2_R, 2.0f);
PARAM_DEFINE_FLOAT(IBVS_D2_R, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA1_R, 0.2f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA2_R, 0.2f);
PARAM_DEFINE_FLOAT(IBVS_L1_R, 6.0f);
PARAM_DEFINE_FLOAT(IBVS_L2_R, 9.0f);

/*PITCH -> X motion -> S1 Feature*/
PARAM_DEFINE_FLOAT(IBVS_K1_P, 0.6f);
PARAM_DEFINE_FLOAT(IBVS_D1_P, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_K2_P, 2.0f);
PARAM_DEFINE_FLOAT(IBVS_D2_P, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA1_P, 0.2f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA2_P, 0.2f);
PARAM_DEFINE_FLOAT(IBVS_L1_P, 6.0f);
PARAM_DEFINE_FLOAT(IBVS_L2_P, 9.0f);

/*ALTITUE -> Z motion -> S3 Feature*/
PARAM_DEFINE_FLOAT(IBVS_K1_H, 3.0f);
PARAM_DEFINE_FLOAT(IBVS_D1_H, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_K2_H, 10.0f);
PARAM_DEFINE_FLOAT(IBVS_D2_H, 0.001f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA1_H, 1.0f);
PARAM_DEFINE_FLOAT(IBVS_GAMMA2_H, 1.0f);
PARAM_DEFINE_FLOAT(IBVS_L1_H, 12.0f);
PARAM_DEFINE_FLOAT(IBVS_L2_H, 36.0f);

/*YAW -> Yaw motion -> S4 Feature*/
PARAM_DEFINE_FLOAT(IBVS_K_PSI1, 0.5f);




