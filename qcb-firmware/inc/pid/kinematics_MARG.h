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

#if !(defined KINEMATICS_MARG_H_) && defined MARG_KIN
#define KINEMATICS_MARG_H_

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

void initializeBaseKinematicsParam(float hdgX, float hdgY);
void initializeKinematics(float hdgX, float hdgY);
void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel,
                         float oneG,               float magX,          float magY,
                         float G_Dt);
float getGyroUnbias(uint8_t axis);
void calibrateKinematics(void);
const float kinematicsGetDegreesHeading(uint8_t axis);

void margUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt);
void eulerAngles(void);

#endif /* KINEMATICS_ARG_H_ */