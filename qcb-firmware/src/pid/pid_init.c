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

#include "pid/pid_init.h"
#include "pid/kinematics_ARG.h"
#include "pid/kinematics_MARG.h"
#include "pid/kinematics_DCM.h"
#include "pid/pid.h"
#include "pid/math.h"
#include "pid/tasks.h"
#include "pid/compass.h"
#include "pid/flight_controller.h"
#include "eq.h"

void pid_init(){

	zeroIntegralError(); //zero the integral error to init
	//TODO this needs to be called whenever pid takes over.
	reset_heading_values();//reset all heading info
	windupGuard_init(); //init x&y windupGuards.

	//TODO
	//We should consider adding the "computeAccelBias" function after calibration.
	//TODO: We need to get exactly SAMPLECOUNT (400) accel samples to compute bias.

	//(Should) also run zeroIntegralError after all calibrations as well.

	#if defined ARG_KIN || defined DCM_KIN
	initializeKinematics(1.0, 0.0);  // with no compass (magnetometer), DCM matrix initalizes to a heading of 0 degrees
	#elif defined MARG_KIN
	initializeKinematics(getHdgXY(XAXIS), getHdgXY(YAXIS));
	#endif

	setupFourthOrder(); //initializes the fourth order filter stuff...

	eq_post_timer(pid_100Hz_task, PID_100Hz, eq_timer_periodic);


}
