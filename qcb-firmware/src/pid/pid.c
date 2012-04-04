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
#include "pid/pid.h"
#include "pid/math.h"
#include "pid/flight_controller.h"
#include "system.h"

// PID Variables
static PIDdata PID[10];

//static float windupGuard;

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)

void pid_init(void)
{
	zeroIntegralError(); // zero the integral error to init
	reset_heading_values();// reset all heading info

	PID[RATE_XAXIS_PID_IDX].P = 100.0;
	PID[RATE_XAXIS_PID_IDX].I = 0.0;
	PID[RATE_XAXIS_PID_IDX].D = -300.0;
	PID[RATE_YAXIS_PID_IDX].P = 100.0;
	PID[RATE_YAXIS_PID_IDX].I = 0.0;
	PID[RATE_YAXIS_PID_IDX].D = -300.0;
	PID[ZAXIS_PID_IDX].P = 200.0;
	PID[ZAXIS_PID_IDX].I = 5.0;
	PID[ZAXIS_PID_IDX].D = 0.0;
	PID[ATTITUDE_XAXIS_PID_IDX].P = 10.0;
	PID[ATTITUDE_XAXIS_PID_IDX].I = 4.0;
	PID[ATTITUDE_XAXIS_PID_IDX].D = -20.0;
	PID[ATTITUDE_YAXIS_PID_IDX].P = 10.0;
	PID[ATTITUDE_YAXIS_PID_IDX].I = 4.0;
	PID[ATTITUDE_YAXIS_PID_IDX].D = -20.0;
	PID[HEADING_HOLD_PID_IDX].P = 3.0;
	PID[HEADING_HOLD_PID_IDX].I = 0.1;
	PID[HEADING_HOLD_PID_IDX].D = 0.0;
	// AKA PID experiements
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = 100.0;
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = -300.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = 100.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = -300.0;

	PID[ALTITUDE_HOLD_PID_IDX].P = 25.0;
	PID[ALTITUDE_HOLD_PID_IDX].I = 0.6;
	PID[ALTITUDE_HOLD_PID_IDX].D = 0.0;
	PID[ALTITUDE_HOLD_PID_IDX].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
	PID[ZDAMPENING_PID_IDX].P = 0.0;
	PID[ZDAMPENING_PID_IDX].I = 0.0;
	PID[ZDAMPENING_PID_IDX].D = 0.0;

	float windupGuard = 1000.0;

	// AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
	for (uint8_t i = XAXIS; i <= ZDAMPENING_PID_IDX; i++ ) {
		if (i != ALTITUDE_HOLD_PID_IDX) {
			PID[i].windupGuard = windupGuard;
		}
	}

	windupGuard_init();
}

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, uint8_t index) {

	PIDdata *PIDparameters = &PID[index];
  // AKA PID experiments
	uint32_t currentTime = system_uptime();
  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  float error = targetPosition - currentPosition;

  PIDparameters->integratedError += error * deltaPIDTime;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  float dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / (deltaPIDTime * 100); // dT fix from Honk
  PIDparameters->lastPosition = currentPosition;
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() {
  for (uint8_t axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = system_uptime();
  }
}


void windupGuard_init(){
	PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = WINDUP_DEFAULT;
	PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = WINDUP_DEFAULT;
}

void reset_heading_error(){
	PID[HEADING_HOLD_PID_IDX].integratedError = 0;
}
