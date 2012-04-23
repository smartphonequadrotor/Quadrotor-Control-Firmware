/*****************************************************************************
Copyright (c) 2011 Abhin Chhabra, Jordan Woehr, Lisa Graham, Ryan Bray

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

#include "qcb.h"
#if defined AHRS_KIN
#include "pid/kinematics_AHRS.h"

#include "pid/globalDefined.h"
#include "pid/math.h"


//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files


//---------------------------------------------------------------------------------------------------
// Definitions


volatile float SamplePeriod;       		// sample period in seconds
#define betaDef		0.15f		// 2 * proportional gain


//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q1 = -0.776976407f, q2 = 0.00115542114f, q3 = 0.000856686675f, q4 = -0.629528165f;	// quaternion of sensor frame relative to auxiliary frame
volatile float kinematicsAngle[3] = {0.0,0.0,0.0};
//====================================================================================================
// Functions

float get_kinematics_angle(uint8_t axis){
	return kinematicsAngle[axis];
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	   // short name local variable for readability
	            float norm;
	            float hx, hy, _2bx, _2bz;
	            float s1, s2, s3, s4;
	            float qDot1, qDot2, qDot3, qDot4;

	            // Auxiliary variables to avoid repeated arithmetic
	            float _2q1mx;
	            float _2q1my;
	            float _2q1mz;
	            float _2q2mx;
	            float _4bx;
	            float _4bz;
	            float _2q1 = 2.0f * q1;
	            float _2q2 = 2.0f * q2;
	            float _2q3 = 2.0f * q3;
	            float _2q4 = 2.0f * q4;
	            float _2q1q3 = 2.0f * q1 * q3;
	            float _2q3q4 = 2.0f * q3 * q4;
	            float q1q1 = q1 * q1;
	            float q1q2 = q1 * q2;
	            float q1q3 = q1 * q3;
	            float q1q4 = q1 * q4;
	            float q2q2 = q2 * q2;
	            float q2q3 = q2 * q3;
	            float q2q4 = q2 * q4;
	            float q3q3 = q3 * q3;
	            float q3q4 = q3 * q4;
	            float q4q4 = q4 * q4;

	            // Normalise accelerometer measurement
	            norm = (float)sqrt(ax * ax + ay * ay + az * az);
	            if (norm == .0f) return; // handle NaN
	            norm = 1 / norm;        // use reciprocal for division
	            ax *= norm;
	            ay *= norm;
	            az *= norm;

	            // Normalise magnetometer measurement
	            norm = (float)sqrt(mx * mx + my * my + mz * mz);
	            if (norm == .0f) return; // handle NaN
	            norm = 1 / norm;        // use reciprocal for division
	            mx *= norm;
	            my *= norm;
	            mz *= norm;

	            // Reference direction of Earth's magnetic field
	            _2q1mx = 2.0f * q1 * mx;
	            _2q1my = 2.0f * q1 * my;
	            _2q1mz = 2.0f * q1 * mz;
	            _2q2mx = 2.0f * q2 * mx;
	            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	            _2bx = (float)sqrt(hx * hx + hy * hy);
	            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	            _4bx = 2.0f * _2bx;
	            _4bz = 2.0f * _2bz;

	            // Gradient decent algorithm corrective step
	            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	            norm = 1.0f / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	            s1 *= norm;
	            s2 *= norm;
	            s3 *= norm;
	            s4 *= norm;

	            // Compute rate of change of quaternion
	            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	            // Integrate to yield quaternion
	            q1 += qDot1 * SamplePeriod;
	            q2 += qDot2 * SamplePeriod;
	            q3 += qDot3 * SamplePeriod;
	            q4 += qDot4 * SamplePeriod;
	            norm = 1.0f / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	            q1 = q1 * norm;
	            q2 = q2 * norm;
	            q3 = q3 * norm;
	            q4 = q4 * norm;
}



////////////////////////////////////////////////////////////////////////////////
// Calculate AHRS
////////////////////////////////////////////////////////////////////////////////

void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,
                 float longitudinalAccel, float lateralAccel, float verticalAccel,
                 float measuredMagX,      float measuredMagY, float measuredMagZ,
				 float G_Dt) {

	SamplePeriod = G_Dt;
	MadgwickAHRSupdate(rollRate,          pitchRate,    yawRate,
             longitudinalAccel, lateralAccel, verticalAccel,
             measuredMagX,      measuredMagY, measuredMagZ);
  eulerAngles();
}

void eulerAngles(void)
{

	//we swap X and Y to get desired orientation...
  kinematicsAngle[XAXIS]  = asin(2 * (q1*q3 - q2*q4));
  kinematicsAngle[YAXIS] = atan2(2 * (q1*q2 + q3*q4), 1 - 2 *(q2*q2 + q3*q3));
  kinematicsAngle[ZAXIS]   = -atan2(2 * (q1*q4 + q2*q3), 1 - 2 *(q3*q3 + q4*q4));


  	for (uint8_t axis = kin_fourth_x; axis <= kin_fourth_z; axis++) {
  		kinematicsAngle[axis] = computeFourthOrder(kinematicsAngle[axis], axis);
  	}

  //Note that the previous configuration did NOT negate the z-axis!
	/*
float heading, bank, attitude;

		float test = q2*q3 + q4*q1;
		if (test > 0.4999f) { // singularity at north pole
			heading = 2 * atan2(q2,q1);
			attitude = PI/2;
			bank = 0;
			return;
		}
		if (test < -0.4999f) { // singularity at south pole
			heading = -2 * atan2(q2,q1);
			attitude = - PI/2;
			bank = 0;
			return;
		}
	    double sqx = q2*q2;
	    double sqy = q3*q3;
	    double sqz = q4*q4;
	    heading = atan2(2*q2*q1-2*q2*q4 , 1 - 2*sqy - 2*sqz);
		attitude = asin(2*test);
		bank = atan2(2*q2*q1-2*q3*q4 , 1 - 2*sqx - 2*sqz);

		kinematicsAngle[ZAXIS] = attitude;
		kinematicsAngle[ZAXIS] = bank;
		kinematicsAngle[ZAXIS] = -heading;
*/
}

void initializeKinematics(){
	q1 = -0.776976407f; q2 = 0.00115542114f; q3 = 0.000856686675f; q4 = -0.629528165f;	// quaternion of sensor frame relative to auxiliary frame
	kinematicsAngle[0] = 0.0;
	kinematicsAngle[1] = 0.0;
	kinematicsAngle[2] = 0.0;
}

#endif
