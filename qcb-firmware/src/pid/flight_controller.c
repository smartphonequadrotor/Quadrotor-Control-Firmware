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

#include "pid/globalDefined.h"
#include "pid/flight_controller.h"
#include "pid/pid.h"
#include "pid/kinematics_ARG.h"
#include "pid/kinematics_MARG.h"
#include "pid/kinematics_DCM.h"
#include "pid/gyro.h"
#include "system.h"
#include "qcfp.h"
#include "pwm.h"



//values between 1000 to 2000
int receiverCommand[4] = {1500, 1500, 1500, 1000};
int receiverZero[4] = {1500, 1500, 1500, 1000};
int throttle = 1000;


uint8_t maxLimit = OFF;
uint8_t minLimit = OFF;

uint8_t YAW_DIRECTION = 1;
int motorAxisCommandRoll = 0;
int motorAxisCommandPitch = 0;
int motorAxisCommandYaw = 0;

//heading values...
uint32_t headingTime = 0;
float headingHold         = 0; // calculated adjustment for quad to go to heading (PID output)
float heading             = 0; // measured heading from yaw gyro (process variable)
float relativeHeading     = 0; // current heading the quad is set to (set point)
float setHeading          = 0;
uint8_t headingHoldState  = OFF;

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};
int motorCommand[4] = {0,0,0,0};


//heading reset...
void reset_heading_values(){
	headingHold         = 0;
	heading             = 0;
	relativeHeading     = 0;
	setHeading          = 0;
}

void calculateFlightError()
{

    float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS]*ATTITUDE_SCALING), get_kinematics_angle(XAXIS), ATTITUDE_XAXIS_PID_IDX);
    float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS]*ATTITUDE_SCALING), -get_kinematics_angle(YAXIS), ATTITUDE_YAXIS_PID_IDX);
    motorAxisCommandRoll   = updatePID(rollAttitudeCmd, get_axis_gr(XAXIS)*1.2, ATTITUDE_GYRO_XAXIS_PID_IDX);
    motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -get_axis_gr(YAXIS)*1.2, ATTITUDE_GYRO_YAXIS_PID_IDX);

}

/**
 * processHeading
 *
 * This function will calculate the craft heading correction depending
 * of the users command. Heading correction is process with the gyro
 * or a magnetometer
 */
void processHeading()
{
	#if defined MARG_KIN || defined DCM_KIN
      heading = degrees(get_kinematics_angle(ZAXIS));
    #else
      heading = degrees(gyroHeading);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }

    // Apply heading hold only when throttle high enough to start flight
    uint32_t currentTime = system_uptime();
      if ((receiverCommand[ZAXIS] > (MIDCOMMAND + 25)) || (receiverCommand[ZAXIS] < (MIDCOMMAND - 25))) {
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        reset_heading_error();
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < 0.25 && relativeHeading > -0.25) {
          headingHold = 0;
          reset_heading_error();
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
          if ((currentTime - headingTime) > 500000) {
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, HEADING_HOLD_PID_IDX);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }

  // NEW SI Version
  const float commandedYaw = constrain(getReceiverSIData(ZAXIS) + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, get_axis_gr(ZAXIS), ZAXIS_PID_IDX);
}


//check motors... upon inspection, unsure if needed.
void processMinMaxCommand()
{

  // Force motors to be equally distant from throttle value for balanced motor output during hard yaw
  uint8_t motorMaxCheck = OFF;
  uint8_t motorMinCheck = OFF;

  // Check if everything within motor limits
  for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
    motorMaxCheck = motorMaxCheck | (motorCommand[motor] >= MAXCOMMAND);
    motorMinCheck = motorMinCheck | (motorCommand[motor] <= minArmedThrottle);
  }

  // If everything within limits, turn flags off and reset max/mins to default
  if (!motorMaxCheck) {
    if (maxLimit) { // only reset if flag was on
      for (uint8_t motor = 0; motor < LASTMOTOR; motor++)
        motorMinCommand[motor] = minArmedThrottle;
      maxLimit = OFF;
    }
  }
  if (!motorMinCheck) {
    if (minLimit) { // only reset if flag was on
      for (uint8_t motor = 0; motor < LASTMOTOR; motor++)
        motorMaxCommand[motor] = MAXCOMMAND;
      minLimit = OFF;
    }
  }

  // If any limits reached, freeze current min/max values and turn limit flag on
  // In future iterations, if limit still exceeded again, use only first frozen values
  for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
    if ((motorCommand[motor] >= MAXCOMMAND) && maxLimit == OFF) {
      for (uint8_t motorLimit = 0; motorLimit < LASTMOTOR; motorLimit++)
        motorMinCommand[motorLimit] = motorCommand[motorLimit];
      maxLimit = ON;
    }
    if ((motorCommand[motor] <= minArmedThrottle) && minLimit == OFF) {
      for (uint8_t motorLimit = 0; motorLimit < LASTMOTOR; motorLimit++)
        motorMaxCommand[motorLimit] = motorCommand[motorLimit];
      minLimit = ON;
    }
  }
}

/**
 * processFlightControl
 *
 * Main flight control processos function
 */
void process_flight_control() {

  // ********************** Calculate Flight Error ***************************
  calculateFlightError();

  // ********************** Update Yaw ***************************************
  processHeading();

/*
  // ********************** Process Altitude hold **************************
    processAltitudeHold();
    // ********************** Process throttle correction ********************
    processThrottleCorrection();
*/

  //MANUALLY setting throttle instead of handling in processAltitudeHold.
  throttle = receiverCommand[THROTTLE];

  // ********************** Calculate Motor Commands *************************
  if (qcfp_flight_enabled()) {
    applyMotorCommand();
  }
  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  //processHardManuevers();

  // If throttle in minimum position, don't apply yaw
  if (receiverCommand[THROTTLE] < MINCHECK) {
    for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
      motorMaxCommand[motor] = minArmedThrottle;
    }
  }

  // Apply limits to motor commands
  for (uint8_t motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // *********************** Command Motors **********************
  if (qcfp_flight_enabled()) {
    writeMotors();
  }
}

// return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverSIData(uint8_t channel) {
  return ((receiverCommand[channel] - receiverZero[channel]) * SI_SCALING);  // +/- 2.5RPS 50% of full rate
}

// return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverAData(uint8_t channel) {
  return ((receiverCommand[channel] - receiverZero[channel]) * ATTITUDE_SCALING);  // +/- 2.5RPS 50% of full rate
}

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear
  const int correctedThrottle = throttle - abs(motorAxisCommandYaw*2/4);
  motorCommand[FRONT_LEFT] = correctedThrottle - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = correctedThrottle - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT] = correctedThrottle + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT] = correctedThrottle + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
}

void writeMotors() {
	pwm_set(pwm_motor1, (motorCommand[MOTOR1]-MIN_PWM_COMMAND)/COMMAND_PWM_RATIO);
	pwm_set(pwm_motor2, (motorCommand[MOTOR2]-MIN_PWM_COMMAND)/COMMAND_PWM_RATIO);
	pwm_set(pwm_motor3, (motorCommand[MOTOR3]-MIN_PWM_COMMAND)/COMMAND_PWM_RATIO);
	pwm_set(pwm_motor4, (motorCommand[MOTOR4]-MIN_PWM_COMMAND)/(COMMAND_PWM_RATIO*2));
}

void write_raw_pid_command(uint8_t axis, int value){
	receiverCommand[axis] = value;
}

