/* SENEX VR CONFIDENTIAL
* 
*  [2021-2025] Senex VR, LLC 
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

#ifndef _SENEX_DRONE_CONTROLS_CPP
#define _SENEX_DRONE_CONTROLS_CPP

#include "Senex_Drone_Controls.h"
#include "Arduino.h"
	double DroneController::pid_full(double err, double target, double dt, double &i_term, double &last_err, double &dfilt, double alpha, double kp, double ki, double kd, double ff_gain, double out_min, double out_max) {
	    // P term
	    double p = kp * err;

	    // D term with LPF (IIR, alpha is mix%, 100% is all old data)
	    double raw_d = (err - last_err) / dt;
	    dfilt = alpha * dfilt + (1 - alpha) * raw_d;
	    double d = kd * dfilt;

	    last_err = err;

	    // Feed-forward
	    double ff = ff_gain * target;

	    // Combine (pre-sat)
	    double u_raw = p + d + ff + i_term;

	    // Clamp
	    double u_sat = u_raw;
	    if (u_sat > out_max) u_sat = out_max;
	    if (u_sat < out_min) u_sat = out_min;

	    // Anti-windup (backcalc)
	    double aw = Kt * (u_sat - u_raw);

	    // Integrator update
	    i_term += (ki * err + aw) * dt;

	    return u_sat;
	}

	void DroneController::flightModel(double sensed_pos[3], double sensed_ang[3], double sensed_rate[3], double target_pos[3], double target_ang_in[3], double thrust_in, double out_cmd[4], double dt) {
	    // ==============================================================
	    // 1) POSITION LOOP → generates target angles + thrust
	    // ==============================================================
	    double pos_err[3] = {
	        target_pos[0] - sensed_pos[0],
	        target_pos[1] - sensed_pos[1],
	        target_pos[2] - sensed_pos[2]
	    };

	    double pos_out[3];
	    for (int i = 0; i < 3; i++) {
	        pos_out[i] = pid_full(
	            pos_err[i], target_pos[i], dt,
	            i_pos[i], last_err_pos[i], deriv_pos[i],
	            alpha_pos,
	            kp_pos[i], ki_pos[i], kd_pos[i],
	            ff_pos[i],
	            pos_out_min, pos_out_max
	        );
	    }

	    // outer-loop → angle targets
	    double target_ang[3];
	    target_ang[0] =  pos_out[1];       // roll  ← y-position
	    target_ang[1] =  pos_out[0];       // pitch ← x-position
	    target_ang[2] =  target_ang_in[2]; // yaw unchanged

	    double thrust_cmd = pos_out[2];

	    // ==============================================================
	    // 2) ANGLE LOOP → generates rate setpoints
	    // ==============================================================
	    double ang_err[3] = {
	        target_ang[0] - sensed_ang[0],
	        target_ang[1] - sensed_ang[1],
	        wrap_pi(target_ang[2] - sensed_ang[2])
	    };

	    double rate_sp[3];
	    for (int i = 0; i < 3; i++) {
	        rate_sp[i] = pid_full(
	            ang_err[i], target_ang[i], dt,
	            i_ang[i], last_err_ang[i], deriv_ang[i],
	            alpha_ang,
	            kp_ang[i], ki_ang[i], kd_ang[i],
	            ff_ang[i],
	            ang_out_min, ang_out_max
	        );
	    }

	    // ==============================================================
	    // 3) RATE LOOP (inner loop)
	    // ==============================================================
	    double rate_err[3] = {
	        rate_sp[0] - sensed_rate[0],
	        rate_sp[1] - sensed_rate[1],
	        rate_sp[2] - sensed_rate[2]
	    };

	    double roll_cmd = pid_full(
	        rate_err[0], rate_sp[0], dt,
	        i_rate[0], last_err_rate[0], deriv_rate[0],
	        alpha_rate,
	        kp_rate[0], ki_rate[0], kd_rate[0],
	        ff_rate[0],
	        out_min, out_max
	    );

	    double pitch_cmd = pid_full(
	        rate_err[1], rate_sp[1], dt,
	        i_rate[1], last_err_rate[1], deriv_rate[1],
	        alpha_rate,
	        kp_rate[1], ki_rate[1], kd_rate[1],
	        ff_rate[1],
	        out_min, out_max
	    );

	    double yaw_cmd = pid_full(
	        rate_err[2], rate_sp[2], dt,
	        i_rate[2], last_err_rate[2], deriv_rate[2],
	        alpha_rate,
	        kp_rate[2], ki_rate[2], kd_rate[2],
	        ff_rate[2],
	        out_min, out_max
	    );

	    // ==============================================================
	    // OUTPUT TO MIXER
	    // ==============================================================
	    out_cmd[0] = thrust_in + thrust_cmd;  // sum pilot + position ctrl
	    out_cmd[1] = roll_cmd;
	    out_cmd[2] = pitch_cmd;
	    out_cmd[3] = yaw_cmd;
	}
	

	void flightHWUpdateMixer(double * controlInputs, uint8_t * controlOutputs) {

		//controlInputs: thrust, roll, pitch, yaw
		double fr = 0.0 - controlInputs[1] * MIXER_ROLL_COEF + controlInputs[2] * MIXER_PITCH_COEF + controlInputs[3] * MIXER_YAW_COEF;
		double bl = 0.0 + controlInputs[1] * MIXER_ROLL_COEF - controlInputs[2] * MIXER_PITCH_COEF + controlInputs[3] * MIXER_YAW_COEF;
		double fl = 0.0 + controlInputs[1] * MIXER_ROLL_COEF + controlInputs[2] * MIXER_PITCH_COEF - controlInputs[3] * MIXER_YAW_COEF;
		double br = 0.0 - controlInputs[1] * MIXER_ROLL_COEF - controlInputs[2] * MIXER_PITCH_COEF - controlInputs[3] * MIXER_YAW_COEF;

		double eulerBounds[2] = {0.0};
		double scaleReduction = 1.0;

		//Adjust rotations to preserve thrust spec (minimum case)
		eulerBounds[0] = min(fr, min(bl, min(fl, br)));
		if(controlInputs[0]  + eulerBounds[0] < 0) {
			scaleReduction = controlInputs[0] / eulerBounds[0];
		}

		//Adjust rotations to preserve thrust spec (maximum case)
		eulerBounds[1] = max(fr, max(bl, max(fl, br)));
		if(controlInputs[0] + eulerBounds[1] > 255.0) {
			scaleReduction = min(scaleReduction, (255.0 - controlInputs[0]) / eulerBounds[0]);
		}

		//Bounding and convert to PWM uint8_t
		controlOutputs[0] = (uint8_t)max(0.0, min(255.0, (controlInputs[0] * MIXER_THRUST_COEF + fr * scaleReduction)));
		controlOutputs[1] = (uint8_t)max(0.0, min(255.0, (controlInputs[0] * MIXER_THRUST_COEF + fl * scaleReduction)));
		controlOutputs[2] = (uint8_t)max(0.0, min(255.0, (controlInputs[0] * MIXER_THRUST_COEF + br * scaleReduction)));
		controlOutputs[3] = (uint8_t)max(0.0, min(255.0, (controlInputs[0] * MIXER_THRUST_COEF + bl * scaleReduction)));
	}


#endif