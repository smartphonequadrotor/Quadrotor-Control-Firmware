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

#include "pid/kinematics.h"
#include "pid/math.h"
#include "pid/compass.h"
#include "pid/tasks.h"
#include "system.h"
#include "eq.h"

// Must not be called until after a calibration has been performed
void kinematics_init(void)
{
	#if defined ARG_KIN || defined DCM_KIN
	initializeKinematics(1.0, 0.0);  // with no compass (magnetometer), DCM matrix initalizes to a heading of 0 degrees
	#elif defined MARG_KIN
	initializeKinematics(getHdgXY(XAXIS), getHdgXY(YAXIS));
	#endif

	setupFourthOrder(); //initializes the fourth order filter stuff...

	eq_post_timer(pid_100Hz_task, 10*SYSTEM_1_MS, eq_timer_periodic);
}

void kinematics_stop(void)
{
	eq_remove_timer(pid_100Hz_task);
}

#if defined ARG_KIN
#include "kinematics_ARG.c"
#elif defined MARG_KIN
#include "kinematics_MARG.c"
#elif defined DCM_KIN
#include "kinematics_DCM.c"
#elif defined AHRS_KIN
#include "kinematics_AHRS.c"
#endif
