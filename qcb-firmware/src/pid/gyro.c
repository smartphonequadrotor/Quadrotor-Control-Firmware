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

#include "pid/gyro.h"
#include "qcb.h"
#include "system.h"
#include "pid/globalDefined.h"
#include "pid/math.h"

float gyroRate[3] = {0.0,0.0,0.0};
int   gyroZero[3] = {0,0,0};
long  gyroSample[3] = {0,0,0};
float gyroSmoothFactor = 1.0;
float gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
float gyroHeading = 0.0;
unsigned long gyroLastMesuredTime = 0;
uint8_t gyroSampleCount = 0;

void record_gyro_sample(int16_t x, int16_t y, int16_t z ){
	gyroSample[XAXIS] += x;
	gyroSample[YAXIS] += y;
	gyroSample[ZAXIS] += z;
	gyroSampleCount++;
}

void evaluateGyroRate() {
	int gyroADC[3];
	gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
	gyroADC[YAXIS] = (gyroSample[YAXIS] / gyroSampleCount) - gyroZero[YAXIS];
	gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);
	//reset sampling
	gyroSample[XAXIS] = 0;
    gyroSample[YAXIS] = 0;
    gyroSample[ZAXIS] = 0;
    gyroSampleCount = 0;

    for (uint8_t axis = 0; axis <= ZAXIS; axis++) {
        gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
      }

    // Measure gyro heading
    long int currentTime = system_uptime();
    if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    	  gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000.0);
    }
  gyroLastMesuredTime = currentTime;
}

void reset_gyro_samples(){

	gyroSample[XAXIS] = 0;
	gyroSample[YAXIS] = 0;
	gyroSample[ZAXIS] = 0;
	gyroSampleCount = 0;
}

void computeGyroBias() {

  //TODO: We need to get exactly SAMPLECOUNT_G (400) gyro samples to compute bias.
  int gyroADC[3];
  for (uint8_t axis = 0; axis < 3; axis++) {
	  gyroADC[axis] = ((float)(gyroSample[axis])/SAMPLECOUNT_G);
    gyroSample[axis] = 0;
  }
  gyroSampleCount = 0;

  gyroZero[XAXIS] = gyroADC[XAXIS];
  gyroZero[YAXIS] = gyroADC[YAXIS];
  gyroZero[ZAXIS] = gyroADC[ZAXIS];
}
