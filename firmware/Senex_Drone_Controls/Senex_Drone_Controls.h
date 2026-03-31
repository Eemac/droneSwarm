/* SENEX VR CONFIDENTIAL
* 
*  [2021] Senex VR, LLC 
*  All Rights Reserved.
* 
* NOTICE:  All information contained herein is, and remains
* the property of Senex VR, LLC and its suppliers,
* if any. The intellectual and technical concepts contained
* herein are proprietary to Senex VR, LLC and its suppliers and may 
* be covered by U.S. and Foreign Patents, patents in process, and 
* are protected by trade secret or copyright law. Dissemination of 
* this information or reproduction of this material is strictly 
* forbidden unless prior written permission is obtained
* from Senex VR, LLC.
*/

/* Comment Syntax:
* -------------------------------------------------------------------------------------------
* |   Title   |   Meaning																	|
* -------------------------------------------------------------------------------------------
* |   @name   |   The name of the function being defined.									|
* |   @brief  |   A quick definition of what the function does.								|
* |   @param  |   A parameter in the function, and a simple description of what it is.		|
* |   @type   |   Whether the function is used in the CONTROLLER, the CORE, or BOTH.		|
* |   @note   |   An additional piece of information, usually when the function was tested.	|
* |   @return |   Possible values the fucntion returns, if any.								|
* -------------------------------------------------------------------------------------------
*/

#ifndef _SENEX_DRONE_CONTROLS_H
#define _SENEX_DRONE_CONTROLS_H

	#include "Arduino.h"

	#define MIXER_THRUST_COEF				1.0
	#define MIXER_ROLL_COEF 				1.0
	#define MIXER_PITCH_COEF 				1.0
	#define MIXER_YAW_COEF 					1.0

	struct DroneController {

	    // =======================
	    // PID GAINS
	    // =======================

	    // Position PIDs (outermost)
	    double kp_pos[3] = {1.0f, 1.0f, 1.5f};
	    double ki_pos[3] = {0.0f, 0.0f, 0.1f};
	    double kd_pos[3] = {0.0f, 0.0f, 0.0f};

	    // Angle-loop PIDs (middle)
	    double kp_ang[3]  = {4.0f, 4.0f, 3.0f};
	    double ki_ang[3]  = {0.0f, 0.0f, 0.0f};
	    double kd_ang[3]  = {0.1f, 0.1f, 0.05f};

	    // Rate-loop PIDs (inner)
	    double kp_rate[3] = {0.15f, 0.15f, 0.10f};
	    double ki_rate[3] = {0.02f, 0.02f, 0.01f};
	    double kd_rate[3] = {0.003f,0.003f,0.002f};

	    // =======================
	    // PID STATE
	    // =======================

	    // Position loop integrators + derivative filters
	    double i_pos[3] = {0};
	    double last_err_pos[3] = {0};
	    double deriv_pos[3] = {0};
	    double alpha_pos = 0.3f;   // LPF constant
	    double ff_pos[3] = {0,0,0};

	    // Angle loop state
	    double i_ang[3]   = {0};
	    double last_err_ang[3] = {0};
	    double deriv_ang[3] = {0};
	    double alpha_ang = 0.4f;
	    double ff_ang[3] = {0,0,0};

	    // Rate loop state
	    double i_rate[3]  = {0};
	    double last_err_rate[3] = {0};
	    double deriv_rate[3] = {0};
	    double alpha_rate = 0.4f;
	    double ff_rate[3] = {0,0,0};

	    // =======================
	    // OUTPUT LIMITS
	    // =======================
	    double out_min = -1.0f;
	    double out_max =  1.0f;

	    double pos_out_min = -5.0f;
	    double pos_out_max =  5.0f;

	    double ang_out_min = -10.0f;
	    double ang_out_max =  10.0f;

	    // Anti-windup gain
	    double Kt = 1.5f;

	    // =======================
	    // UTILS
	    // =======================
	    static double wrap_pi(double x) {
	        while (x >  3.14159f) x -= 2*3.14159f;
	        while (x < -3.14159f) x += 2*3.14159f;
	        return x;
	    }

	    // ======================================================================
	    // UPDATED PID: derivative filtering + feed-forward + clamp + backcalc
	    // ======================================================================
	    double pid_full(double err, double target, double dt, double &i_term, double &last_err, double &dfilt, double alpha, double kp, double ki, double kd, double ff_gain, double out_min, double out_max);

	    // ======================================================================
	    // MAIN UPDATE: FULL 3-LOOP (position → angle → rate)
	    // ======================================================================
	    									  //yaw, pitch, roll   //// p,q,r
	    void flightModel(double sensed_pos[3], double sensed_ang[3], double sensed_rate[3], double target_pos[3], double target_ang_in[3], double thrust_in, double out_cmd[4], double dt);
	};

	void flightModel(double sensed_pos[3], double sensed_ang[3], double sensed_rate[3], double target_pos[3], double target_ang_in[3], double thrust_in, double out_cmd[4], double dt);

	/*
	* 	Given the following motor mixer matrix:
	*	|-  -|      |-          -|   |-      -|
	*	| fr |      | 1 -1  1  1 |   | thrust |
	*	| bl |  =   | 1  1 -1  1 | * | roll   |
	*	| fl |      | 1  1  1 -1 |   | pitch  |
	*	| br |      | 1 -1 -1 -1 |   | yaw    |
	*	|-  -|      |-          -|   |-      -|

	*   it is possible to calculate motor power from thrust, 
	*/
	void flightHWUpdateMixer(double * controlInputs, uint8_t * controlOutputs);

#endif