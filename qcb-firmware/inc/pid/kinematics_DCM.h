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

#if !(defined KINEMATICS_DCM_H_) && defined DCM_KIN
#define KINEMATICS_DCM_H_

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

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

void initializeBaseKinematicsParam(float hdgX, float hdgY);
void initializeKinematics(float hdgX, float hdgY);
void matrixUpdate(float p, float q, float r, float G_Dt);
void normalize(void);
void driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY);
void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel,
                         float oneG,               float magX,          float magY,
                         float G_Dt);
float getGyroUnbias(uint8_t axis);
void calibrateKinematics(void);
const float kinematicsGetDegreesHeading(uint8_t axis);

void earthAxisAccels(float ax, float ay, float az, float oneG);
void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt);
void eulerAngles(void);

#endif /* KINEMATICS_DCM_H_ */
