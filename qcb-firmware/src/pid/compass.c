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

#include "pid/compass.h"
#include "qcb.h"
#include "pid/math.h"

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

float hdgX = 0.0;
float hdgY = 0.0;

float measuredMagX = 0.0;
float measuredMagY = 0.0;
float measuredMagZ = 0.0;

float rawMag[3] = {0.0,0.0,0.0};
float magBias[3] = {0.0,0.0,0.0};

void record_compass_sample(int16_t x, int16_t y, int16_t z ){
	rawMag[XAXIS] = x;
	rawMag[YAXIS] = y;
	rawMag[ZAXIS] = z;
}

void read_compass(float roll, float pitch) {

  measuredMagX = rawMag[XAXIS] + magBias[XAXIS];
  measuredMagY = rawMag[YAXIS] + magBias[YAXIS];
  measuredMagZ = rawMag[ZAXIS] + magBias[ZAXIS];

  const float cosRoll =  cos(roll);
  const float sinRoll =  sin(roll);
  const float cosPitch = cos(pitch);
  const float sinPitch = sin(pitch);

  float magX = (float)measuredMagX * cosPitch +
                     (float)measuredMagY * sinRoll * sinPitch +
                     (float)measuredMagZ * cosRoll * sinPitch;

  float magY = (float)measuredMagY * cosRoll -
                     (float)measuredMagZ * sinRoll;

  float tmp  = sqrt(magX * magX + magY * magY);

  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

float getHdgXY(uint8_t axis) {
  if (axis == XAXIS) {
    return hdgX;
  } else {
    return hdgY;
  }
}

int read_compass_raw(uint8_t axis) {
  return rawMag[axis];
}
