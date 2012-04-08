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

#ifndef _PID_KINEMATICS_H_
#define _PID_KINEMATICS_H_

#include "qcb.h"
#include "pid/globalDefined.h"

#define KINEMATICS_UPDATE_PERIOD 20*SYSTEM_1_MS

void kinematics_init(void);
void kinematics_stop(void);

#if defined ARG_KIN
#include "pid/kinematics_ARG.h"
#elif defined MARG_KIN
#include "pid/kinematics_MARG.h"
#elif defined DCM_KIN
#include "pid/kinematics_DCM.h"
#elif defined AHRS_KIN
#include "pid/kinematics_AHRS.h"
#endif

#endif // _PID_KINEMATICS_H_
