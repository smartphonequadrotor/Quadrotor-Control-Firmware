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

#include "pid/pid_init.h"
#include "pid/kinematics_ARG.h"
#include "pid/pid.h"
#include "pid/math.h"
#include "pid/tasks.h"
#include "eq.h"


void pid_init(){

	zeroIntegralError(); //zero the integral error to init
	windupGuard_init(); //init x&y windupGuards.

	//TODO
	//We should consider adding the "computeAccelBias" function after calibration.
	//TODO: We need to get exactly SAMPLECOUNT (400) accel samples to compute bias.

	//(Should) also run zeroIntegralError after all calibrations as well.

	initializeKinematics(1.0, 0.0);  // with no compass (magnetometer), DCM matrix initalizes to a heading of 0 degrees
	setupFourthOrder(); //initializes the fourth order filter stuff...

	//TODO
	//When measuring data, we need to pop it into something like this:
	// meterPerSecSec[YAXIS] = readReverseShortI2C() * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
	//meterPerSecSec[XAXIS] = readReverseShortI2C() * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
	//meterPerSecSec[ZAXIS] = readReverseShortI2C() * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];

	eq_post_timer(pid_100Hz_task, PID_100Hz, eq_timer_periodic);


}
