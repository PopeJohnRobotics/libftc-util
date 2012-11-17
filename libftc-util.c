/*
Copyright (C) 2011 by Pope John XXIII Robotics Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#pragma systemFile

#include "drivers/common.h"
#include "drivers/HTAC-driver.h"

#define PAUSE wait10Msec(17)

///////////////////////////////////////////////////////////
// CONFIG
///////////////////////////////////////////////////////////
int gyroOff = 600;// Usually correct.

int libftc_initGyro(){
  int error = 0;
  if (HAS_GYRO){
    gyroOff = 0;
    int min = 9001,max = 0;
    PlayTone(7000,10);
    for (int i = 0; i < 10; ++i){
      int sens_Val;
      gyroOff += (sens_Val = SensorRaw[gyroPort]);
      sens_Val < min ? min = sens_Val : (sens_Val > max ? max = sens_Val : 1);// Set the max and min values.
    }
    PlayTone(2000,10);
    gyroOff /= 10;
    if (gyroOff < 400){ error++; writeDebugStreamLine("gyroOff too low.");    }
    if (gyroOff > 800){ error++; writeDebugStreamLine("gyroOff too high.");   }
    if (max - min > 5){ error++; writeDebugStreamLine("Max-min: %d",max-min); }
    nxtDisplayCenteredTextLine(1,"%d",gyroOff);
  }
  return error;
}

int accelOff = 0;
int libftc_initAccel(){
  int error = 0;
  if (HAS_ACCEL){
    int min = 0,max = 0;
    PlayTone(7000,10);
    for (int i = 0; i < 10; ++i){
      int sens_Val;
      HTACreadX(accelPort, sens_Val);
      accelOff += sens_Val;
      sens_Val < min ? min = sens_Val : (sens_Val > max ? max = sens_Val : 1);// Set the max and min values.
      wait10Msec(10);
    }
    PlayTone(2000,10);
    accelOff /= 10;
    if (accelOff > 30) error++;
    if (max-min > 5) error++;
  }
  return error;
}

// Call this before the round starts.
int libftc_init(){
  int error = 0;
  if (HAS_GYRO) error += libftc_initGyro();
  if (HAS_ACCEL) error += libftc_initAccel();
  if (error > 0) PlayImmediateTone(500,1000);
  return error;
}

#include "ConnectedJoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

#define joy1Btn(btn) ((joystick.joy1_Buttons & (1 << (btn - 1))) != 0)
#define joy2Btn(btn) ((joystick.joy2_Buttons & (1 << (btn - 1))) != 0)

// Goes forward at given power for given time in milliseconds.
void libftc_forwardForTimeAndStop(int power,int time)
{
  rightSide(power);
  leftSide(power);
  wait1Msec(time);
  rightSide(0);
  leftSide(0);
}

void libftc_turnRightForTimeAndStop(int power,int time)
{
  rightSide(power);
  leftSide(-power);
  wait1Msec(time);
  rightSide(0);
  leftSide(0);
}

void turnRightForGyroAndStop(int power,int gyroDist)
{
  rightSide(power);
  leftSide(-power);
  float gyroVal = 0;
  bool running = true;
  bool returning = false;
  while (running) {
    float deltaGyro = (float)(abs(SensorValue[gyroPort]-gyroOff) < 2 ? 0 : (SensorValue[gyroPort]-gyroOff))/200.0;
    gyroVal += deltaGyro;
    if (abs(gyroVal) > abs(gyroDist)){
      returning = true;
    }
    if (returning){
      rightSide(-power/4);
      leftSide(power/4);
      if (abs(gyroVal) < abs(gyroDist)){
        running = false;
        writeDebugStreamLine("Final GyroVal: %f",gyroVal);
      }
    }

    wait1Msec(5);
  }
  rightSide(0);
  leftSide(0);
}

#define WAITING_TO_START 0
#define MOVING 1
#define WAITING_TO_STRAIGHTEN 4
#define STRAIGHTENING_DECIDE 2
#define STRAIGHTENING_EXEC 3

///////////////////////////////////////////////////////////
// EXPLANATION OF STATE MACHINE
///////////////////////////////////////////////////////////
// The state machine has 4 states. It starts in WAITING_TO_START. This gives it a slight delay, until it goes to MOVING.
// Once in MOVING, the robot moves at full speed, and goes straight using the gyro.
// The robot leaves MOVING when the time is up, and goes to STRAIGHTENING_DECIDE.
// STRAIGHTENING_DECIDE picks whether to move the robot to the right or the left, based on its wrongness in position.
// STRAIGHTENING_EXEC spins the robot the appropriate direction to center the robot on its desired heading.
// Once STRAIGHTENING_EXEC decides its done, the robot stops moving and the function exits.
///////////////////////////////////////////////////////////
float libftc_gyroForwardAndStop(int power,int time)
{
  int movement_state = WAITING_TO_START;
  ClearTimer(T4);
  float gyroVal = 0;
  bool shouldStart = false;
  bool mainLoop = true;
  bool straighten_going_right = false;
  while (mainLoop) {
    // Integrate gyro readings
    float deltaGyro = (float)(SensorValue[gyroPort]-gyroOff)/50.0;
    gyroVal += deltaGyro;
    //State machine.
    switch (movement_state){
      case WAITING_TO_START:
        if (shouldStart == false){
          shouldStart = true;
        }else {
          movement_state = MOVING;
        }
        break;
      case MOVING:
        int rightPower = power + (int)gyroVal*8;
        int leftPower = power - (int)gyroVal*8;
        if (max(rightPower,leftPower) > 100){
          rightPower -= max(rightPower,leftPower)-100;
          leftPower  -= max(rightPower,leftPower)-100;
        }
        if (min(rightPower,leftPower) < 100){
          rightPower -= min(rightPower,leftPower)+100;
          leftPower  -= min(rightPower,leftPower)+100;
        }
        rightSide(rightPower);
        leftSide(leftPower);
        if (time1[T4] > time){
          movement_state = WAITING_TO_STRAIGHTEN;
          ClearTimer(T3);
        }
        break;
      case WAITING_TO_STRAIGHTEN:
        if (time1[T3] > 150){
          movement_state = STRAIGHTENING_DECIDE;
        }
        rightSide(0);
        leftSide(0);
        break;
      case STRAIGHTENING_DECIDE:
        if (gyroVal > 0){
          straighten_going_right = true;
          PlayImmediateTone(300,10);
        }else if (gyroVal < 0){
          straighten_going_right = false;
          PlayImmediateTone(2000,10);
        }
        //if (power < 0) straighten_going_right = !straighten_going_right;
        movement_state = STRAIGHTENING_EXEC;
        break;
      case STRAIGHTENING_EXEC:
        if (straighten_going_right){
          rightSide(-power/2);
          leftSide(power/2);
          if (gyroVal > 0){
            mainLoop = false;
          }
        }else{
          rightSide(power/2);
          leftSide(-power/2);
          if (gyroVal < 0){
            mainLoop = false;
          }
        }
        break;
    }// End of state machine
    writeDebugStreamLine("[fwd] st: %d",movement_state);
    writeDebugStreamLine("[fwd] gyroVal: %f",gyroVal);
    wait1Msec(20);
  }
  rightSide(0);
  leftSide(0);
  writeDebugStreamLine("_________________");
  return gyroVal;
}
/* Doesn't work
void libftc_gyroForwardAndStopSmooth(int power,int time)
{
  libftc_gyroForwardAndStop(power/4,time/10);
  libftc_gyroForwardAndStop(power, 8*time/10);
  libftc_gyroForwardAndStop(power/4,time/10);
}
*/
void libftc_checkBattery(){
  PlayTone(15000 - externalBattery, 200);
  while(externalBattery < 13000){
    PlayTone(3000, 40);
    PlayTone(8000, 40);
  }
  ClearSounds();
}

task libftc_killSwitch()
{
  nSchedulePriority = 10;
  int i = 1;
  while(true) {
    if ((joy1Btn(1) && joy1Btn(2) && joy1Btn(3) && joy1Btn(4))||(joy2Btn(1) && joy2Btn(2) && joy2Btn(3) && joy2Btn(4))) {
      leftSide(0);
      rightSide(0);
      StopAllTasks();
    }
    wait10Msec(100);
    ++i;
  }
}
