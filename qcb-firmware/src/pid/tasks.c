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

#include "qcb.h"
#include "qcfp.h"
#include "pid/tasks.h"
#include "system.h"
#include "pid/accel.h"
#include "pid/gyro.h"
#include "pid/compass.h"
#include "pid/math.h"
#include "pid/kinematics_ARG.h"
#include "pid/kinematics_MARG.h"
#include "pid/kinematics_DCM.h"
#include "pid/globalDefined.h"
#include "pid/flight_controller.h"

static float G_Dt = .02;
static uint32_t pid_100HZ_previousTime = 0;


void pid_100Hz_task(){

	//update times...
	uint32_t current_time = system_uptime();
	G_Dt = ((float)(current_time - pid_100HZ_previousTime))/1000.0;
	pid_100HZ_previousTime = current_time;

	//call to get accel information.
	evaluateMetersPerSec();
	evaluateGyroRate();

	//fourth order filter...
	float filtered_accel[3] = {0.0,0.0,0.0};
	for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
		filtered_accel[axis] = computeFourthOrder(get_axis_mps(axis), axis);
	}

	//kinematics calculation to determine orientation
	#if defined MARG_KIN
	calculateKinematics(get_axis_gr(XAXIS),
						get_axis_gr(YAXIS),
						get_axis_gr(ZAXIS),
						filtered_accel[XAXIS],
						filtered_accel[YAXIS],
						filtered_accel[ZAXIS],
						read_compass_raw(XAXIS),
						read_compass_raw(YAXIS),
						read_compass_raw(ZAXIS),
						G_Dt);
	#elif defined ARG_KIN
	calculateKinematics(get_axis_gr(XAXIS),
						get_axis_gr(YAXIS),
						get_axis_gr(ZAXIS),
						filtered_accel[XAXIS],
						filtered_accel[YAXIS],
						filtered_accel[ZAXIS],
						0.0,
						0.0,
						0.0,
						G_Dt);
	#elif defined DCM_KIN
	calculateKinematics(get_axis_gr(XAXIS),
						get_axis_gr(YAXIS),
						get_axis_gr(ZAXIS),
						filtered_accel[XAXIS],
						filtered_accel[YAXIS],
						filtered_accel[ZAXIS],
						get_accel_one_G(),
						getHdgXY(XAXIS),
						getHdgXY(YAXIS),
						G_Dt);
	#else
		#error "Must define at least one of ARG_KIN, MARG_KIN, or DCM_KIN"
	#endif

	qcfp_send_kinematics_angles();

	if(qcfp_pid_enabled())
	{
		//update flight parameters using kinematics.
		process_flight_control();
	}
}
