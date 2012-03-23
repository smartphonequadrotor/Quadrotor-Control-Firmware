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

#include "pid/pid.h"
#include "pid/math.h"
#include "system.h"


// PID Variables
PIDdata PID[10];

float windupGuard;

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)


// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, PIDdata *PIDparameters) {

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
