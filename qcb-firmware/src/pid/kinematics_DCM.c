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

#if defined DCM_KIN

#include "pid/kinematics_DCM.h"

#include "pid/globalDefined.h"
#include "qcb.h"
#include "pid/math.h"

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


// Written by William Premerlani
// Modified by Jose Julio for multicopters
// http://diydrones.com/profiles/blogs/dcm-imu-theory-first-draft
// Optimizations done by Jihlein
// http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=12286&viewfull=1#post12286
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

float dcmMatrix[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float omegaP[3] = {0.0,0.0,0.0};
float omegaI[3] = {0.0,0.0,0.0};
float omega[3] = {0.0,0.0,0.0};
float errorCourse = 0.0;
float kpRollPitch = 0.0;
float kiRollPitch = 0.0;
float kpYaw = 0.0;
float kiYaw = 0.0;


void initializeBaseKinematicsParam(float hdgX, float hdgY) {
  for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++)
    kinematicsAngle[axis] = 0.0;
  gyroAngle[XAXIS] = 0;
  gyroAngle[YAXIS] = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Matrix Update
////////////////////////////////////////////////////////////////////////////////

void matrixUpdate(float p, float q, float r, float G_Dt)
{
  float rateGyroVector[3];
  float temporaryMatrix[9];
  float updateMatrix[9];

  rateGyroVector[XAXIS]  = p;
  rateGyroVector[YAXIS] = q;
  rateGyroVector[ZAXIS]   = r;

  vectorSubtract(3, &omega[XAXIS], &rateGyroVector[XAXIS], &omegaI[XAXIS]);
  vectorSubtract(3, &correctedRateVector[XAXIS], &omega[XAXIS], &omegaP[XAXIS]);

  //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter

  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt * correctedRateVector[ZAXIS];    // -r
  updateMatrix[2] =  G_Dt * correctedRateVector[YAXIS];  //  q
  updateMatrix[3] =  G_Dt * correctedRateVector[ZAXIS];    //  r
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt * correctedRateVector[XAXIS];   // -p
  updateMatrix[6] = -G_Dt * correctedRateVector[YAXIS];  // -q
  updateMatrix[7] =  G_Dt * correctedRateVector[XAXIS];   //  p
  updateMatrix[8] =  0;

  matrixMultiply(3, 3, 3, temporaryMatrix, dcmMatrix, updateMatrix);
  matrixAdd(3, 3, dcmMatrix, dcmMatrix, temporaryMatrix);
}

////////////////////////////////////////////////////////////////////////////////
// Normalize
////////////////////////////////////////////////////////////////////////////////
void normalize()
{
  float error=0;
  float temporary[9];
  float renorm=0;

  error= -vectorDotProduct(3, &dcmMatrix[0], &dcmMatrix[3]) * 0.5;         // eq.18

  vectorScale(3, &temporary[0], &dcmMatrix[3], error);                     // eq.19
  vectorScale(3, &temporary[3], &dcmMatrix[0], error);                     // eq.19

  vectorAdd(6, &temporary[0], &temporary[0], &dcmMatrix[0]);               // eq.19

  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);           // eq.20

  for(uint8_t v=0; v<9; v+=3) {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));  // eq.21
    vectorScale(3, &dcmMatrix[v], &temporary[v], renorm);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Drift Correction
////////////////////////////////////////////////////////////////////////////////

void driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY)
{
  //  Compensation of the Roll, Pitch and Yaw drift.
  float accelMagnitude;
  float accelVector[3];
  float accelWeight;
  float errorRollPitch[3];
  #ifdef HeadingMagHold
    float errorCourse;
    float errorYaw[3];
    float scaledOmegaP[3];
  #endif
  float scaledOmegaI[3];

  //  Roll and Pitch Compensation
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  // Calculate the magnitude of the accelerometer vector
  accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] +
                         accelVector[YAXIS] * accelVector[YAXIS] +
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;

  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // accelWeight = constrain(1 - 4*abs(1 - accelMagnitude),0,1);

  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);

  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch * accelWeight);

  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);

  //  Yaw Compensation
  #ifdef HeadingMagHold
    errorCourse = (dcmMatrix[0] * magY) - (dcmMatrix[3] * magX);
    vectorScale(3, errorYaw, &dcmMatrix[6], errorCourse);

    vectorScale(3, &scaledOmegaP[0], &errorYaw[0], kpYaw);
    vectorAdd(3, omegaP, omegaP, scaledOmegaP);

    vectorScale(3, &scaledOmegaI[0] ,&errorYaw[0], kiYaw);
    vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  #else
    omegaP[ZAXIS] = 0.0;
    omegaI[ZAXIS] = 0.0;
  #endif
}

////////////////////////////////////////////////////////////////////////////////
// Accel Adjust
////////////////////////////////////////////////////////////////////////////////

/*void Accel_adjust(void) {
  // ADC : Voltage reference 3.0V / 10bits(1024 steps) => 2.93mV/ADC step
  // ADXL335 Sensitivity(from datasheet) => 330mV/g, 2.93mV/ADC step => 330/0.8 = 102
  #define GRAVITY 102 //this equivalent to 1G in the raw data coming from the accelerometer
  #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

  accelVector[1] += Accel_Scale(speed_3d*omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
  accelVector[2] -= Accel_Scale(speed_3d*omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}*/

////////////////////////////////////////////////////////////////////////////////
// Euler Angles
////////////////////////////////////////////////////////////////////////////////

void eulerAngles(void)
{
  kinematicsAngle[XAXIS]  =  atan2(dcmMatrix[7], dcmMatrix[8]);
  kinematicsAngle[YAXIS] =  -asin(dcmMatrix[6]);
  kinematicsAngle[ZAXIS]   =  atan2(dcmMatrix[3], dcmMatrix[0]);
}

////////////////////////////////////////////////////////////////////////////////
// Earth Axis Accels
////////////////////////////////////////////////////////////////////////////////

void earthAxisAccels(float ax, float ay, float az, float oneG)
{
  float accelVector[3];

  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  earthAccel[XAXIS] = vectorDotProduct(3, &dcmMatrix[0], &accelVector[0]);
  earthAccel[YAXIS] = vectorDotProduct(3, &dcmMatrix[3], &accelVector[0]);
  earthAccel[ZAXIS] = vectorDotProduct(3, &dcmMatrix[6], &accelVector[0]) + oneG;
}


////////////////////////////////////////////////////////////////////////////////
// Initialize DCM
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics(float hdgX, float hdgY)
{
  initializeBaseKinematicsParam(hdgX,hdgY);
  for (uint8_t i=0; i<3; i++) {
    omegaP[i] = 0;
    omegaI[i] = 0;
  }
  dcmMatrix[0] =  hdgX;
  dcmMatrix[1] = -hdgY;
  dcmMatrix[2] =  0;
  dcmMatrix[3] =  hdgY;
  dcmMatrix[4] =  hdgX;
  dcmMatrix[5] =  0;
  dcmMatrix[6] =  0;
  dcmMatrix[7] =  0;
  dcmMatrix[8] =  1;

  kpRollPitch = 0.1;        // alternate 0.05;
  kiRollPitch = 0.0002;     // alternate 0.0001;

  kpYaw = -0.1;             // alternate -0.05;
  kiYaw = -0.0002;          // alternate -0.0001;

}

////////////////////////////////////////////////////////////////////////////////
// Calculate DCM
////////////////////////////////////////////////////////////////////////////////

void calculateKinematics(float rollRate,            float pitchRate,      float yawRate,
                         float longitudinalAccel,   float lateralAccel,   float verticalAccel,
                         float oneG,                float magX,           float magY,
				         float G_Dt) {

  matrixUpdate(rollRate, pitchRate, yawRate, G_Dt);
  normalize();
  driftCorrection(longitudinalAccel, lateralAccel, verticalAccel, oneG, magX, magY);
  eulerAngles();
  earthAxisAccels(longitudinalAccel, lateralAccel, verticalAccel, oneG);
}

#endif
