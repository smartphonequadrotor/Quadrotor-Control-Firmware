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

#if defined ARG_KIN
#include "pid/kinematics_ARG.h"

/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "pid/globalDefined.h"
#include "qcb.h"
#include "pid/math.h"

// This class is responsible for calculating vehicle attitude
uint8_t kinematicsType = 0;
float kinematicsAngle[3] = {0.0,0.0,0.0};
float gyroAngle[2] = {0.0,0.0};
float correctedRateVector[3] = {0.0,0.0,0.0};
float earthAccel[3] = {0.0,0.0,0.0};

float accelCutoff = 0.0;

float Kp = 0.0;                   					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.0;                   					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error

float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;

float get_kinematics_angle(uint8_t axis){
	return kinematicsAngle[axis];
}

void initializeBaseKinematicsParam(float hdgX, float hdgY) {
  for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++)
    kinematicsAngle[axis] = 0.0;
  gyroAngle[XAXIS] = 0;
  gyroAngle[YAXIS] = 0;
}

  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
const float kinematicsGetDegreesHeading(uint8_t axis) {
  float tDegrees;

  tDegrees = degrees(kinematicsAngle[axis]);
  if (tDegrees < 0.0)
    return (tDegrees + 360.0);
  else
    return (tDegrees);
}


////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt) {

  float norm;
  float vx, vy, vz;
  float q0i, q1i, q2i, q3i;
  float ex, ey, ez;

  halfT = G_Dt/2;

  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (vy*az - vz*ay);
  ey = (vz*ax - vx*az);
  ez = (vx*ay - vy*ax);

  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;

  eyInt = eyInt + ey*Ki;
  if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;

  ezInt = ezInt + ez*Ki;
  if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;

  // adjusted gyroscope measurements
  correctedRateVector[XAXIS] = gx = gx + Kp*ex + exInt;
  correctedRateVector[YAXIS] = gy = gy + Kp*ey + eyInt;
  correctedRateVector[ZAXIS] = gz = gz + Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  // save the adjusted gyroscope measurements
  correctedRateVector[XAXIS] = gx;
  correctedRateVector[YAXIS] = gy;
  correctedRateVector[ZAXIS] = gz;
}

void eulerAngles()
{
  kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ARG
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics(float hdgX, float hdgY)
{
  initializeBaseKinematicsParam(hdgX,hdgY);
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;

  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  Kp = 0.2; // 2.0;
  Ki = 0.0005; //0.005;
}

////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,
                         float longitudinalAccel, float lateralAccel, float verticalAccel,
                         float measuredMagX,      float measuredMagY, float measuredMagZ,
                         float G_Dt) {

  argUpdate(rollRate,          pitchRate,    yawRate,
            longitudinalAccel, lateralAccel, verticalAccel,
            measuredMagX,      measuredMagY, measuredMagZ,
		    G_Dt);
  eulerAngles();
}

float getGyroUnbias(uint8_t axis) {
  return correctedRateVector[axis];
}

#endif





