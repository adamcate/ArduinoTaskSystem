/*
EXAMPLE 1: Simple driving
*/
#include <Arduino.h>

#include "types.h"                  // OppyJr specific libraries
#include "queue\queue.hpp"
#include "driving.hpp"
#include "mathLib\angles.hpp"
#include "pins.h"



Queue queue{};  // queue is defined as empty

Task _empty_tasks[MAX_TASKS] = {EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY()};
Task _init_tasks[MAX_TASKS] = {DRIVE_T(0,255,0,255,2000),DRIVE(255,255,1000),HALT(1000),EMPTY(),EMPTY()}; // initial set of tasks to be pushed to the queue. will repeat

// Ramp up to full over 2 seconds (2000ms), drive for 1 second (1000ms), then halt in 1 second



void clearQueue(Queue& queue){  // remove all elements from the queue, and reset their interrupt flags
  for(int i = 0; i < MAX_TASKS; ++i) queue.setAbsolute(_empty_tasks[i],i);
}



void setup(){
  Serial.begin(9600); // initialize the serial monitor to print the wheel speeds

  for(int task = 0; task < MAX_TASKS; ++task) queue.setAbsolute(_init_tasks[task],task);  // push all the initial tasks to the queue


  INIT_MOTOR_PINS();
}

void loop(){
  queue.startFrame(); // all code lies in between queue.startFrame() and queue.endFrame()
  

  queue.executeAction();

  Serial.print("Left: ");
  Serial.print(queue.wheelSpeed[LEFT]);
  Serial.print(" Right: ");
  Serial.println(queue.wheelSpeed[RIGHT]);


  queue.endFrame();
}