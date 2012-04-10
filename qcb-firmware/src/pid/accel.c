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
#include "pid/accel.h"
#include "pid/globalDefined.h"
#include "pid/math.h"


// 4mg per LSB so accel in m/s/s is the value read *0.004*g
float accelScaleFactor[3] = {0.004*9.8065, 0.004*9.8065, 0.004*9.8065};
float runTimeAccelBias[3] = {0, 0, 0};
float accelOneG = 0.0;
float meterPerSecSec[3] = {0.0, 0.0, 0.0};
long accelSample[3] = {0, 0, 0};
uint8_t accelSampleCount = 0;

void record_accel_sample(int16_t x, int16_t y, int16_t z ){
	  accelSample[XAXIS] += x;
	  accelSample[YAXIS] += y;
	  accelSample[ZAXIS] += z;
	  accelSample[ZAXIS] += z;
	  accelSampleCount++;
}

void evaluateMetersPerSec() {

  for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (((float)accelSample[axis]) / (float)accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }

  accelSampleCount = 0;
}

void reset_accel_samples(){
	accelSample[XAXIS] = 0;
	accelSample[YAXIS] = 0;
	accelSample[ZAXIS] = 0;
	accelSample[ZAXIS] = 0;
	accelSampleCount = 0;
}

void computeAccelBias() {
  for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = ((float)(accelSample[axis])/((float)accelSampleCount)) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = 0;//-(9.8065 - meterPerSecSec[ZAXIS]);
  accelOneG = 9.8065; //abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

float get_axis_mps(uint8_t axis){
	return meterPerSecSec[axis];
}

float get_accel_one_G(){
	return accelOneG;
}
