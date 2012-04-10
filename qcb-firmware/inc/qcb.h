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

#ifndef _QCB_H_
#define _QCB_H_

/*
 * File for QCB specific defines, variables, functions that don't belong
 * anywhere else, and commonly used headers.
 */

/* Commonly used headers */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* microcontroller header */
#include "AT91SAM7S161.h"

#define QCB_MCK 48054857

// One of the following should be defined:
// 1) ARG_KIN if not using magnetometer
// 2) MARG_KIN if using magnetometer
// 3) DCM_KIN if using plain DCM filter using
//		computed heading values rather than integrating
//		raw values into kinematics.
// 4) AHRS_KIN
#define DCM_KIN

/*
 * TODO:
 * Implement workaround for errata  detailed in section 40.22.2.1
 */

#endif // _QCB_H_
