/*
Written by Adam Cate - last revision 4/13/2022
OppyJr : main.cpp
What is a main? A miserable little pile of secrets. But enough talk!
*/

#include <Wire.h>                   // External libraries
#include <Arduino.h>
#include <Arduino_NineAxesMotion.h>
#include <NewPing.h>

#include "types.h"                  // OppyJr specific libraries
#include "queue\queue.hpp"
#include "vec.h"
#include "bitmatrix.hpp"
#include "Raycast.hpp"
#include "driving.hpp"
#include "mathLib\angles.hpp"

#include "pins.h"


#define MAX_DISTANCE 50  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SONAR_NUM 3

const u8 trigger_pins[SONAR_NUM]{};
const u8 echo_pins[SONAR_NUM]{};

#define GRID_WIDTH   10   // Width and height used for obstacle_map. Setting values too high causes excessive memory allocation, increase at your own risk
#define GRID_HEIGHT  10

#define BACKUP_DIST  1.71f// How far the robot will attempt to back up after detecting an obstacle (meters)

u8 IMUmode;

#define SET_9DOF_MODE(sensor,mode){\
  sensor.setOperationMode(mode);   \
  IMUmode = mode;                  \
}

NineAxesMotion IMU;
u32 IMUlastStreamTime = 0;
const u16 IMUstreamPeriod = 50;

const s16 s_detect_threshold = 40;
bool inThreshold(s16 level){return (level != 0 && level < s_detect_threshold);}
NewPing sonars[SONAR_NUM]{
  NewPing(12, 13, MAX_DISTANCE), // RIGHT
  NewPing(33, 35, MAX_DISTANCE), // MID
  NewPing(38, 40, MAX_DISTANCE)  // LEFT
};

s16 sonicValues[SONAR_NUM]{};

u32 pingTimer = 0;
const u16 pingSpeed = 50;

u16 speedFactor = 0.01f;
f32 heading_angle = 0.f;
f32 calib_angle = 0.f;

f32 pitch{};
Vec2 dir(1.f,0.f);
Vec2 vel{};
Vec2 pos{};
f32 acc{};
Vec3 prevAcc{};

const s16 numCasts = 15;
Vec3 raycastOutputs[numCasts]{};
Vec2 detect_point{};
Vec2 dest_point(5.5f, 5.5f);

bitMatrix obstacleMap{};


Queue queue{};
Queue avoidance{};
Task _empty_tasks[MAX_TASKS] = {EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY()};
Task _init_tasks[MAX_TASKS] = {EMPTY(), DRIVE_T(0,150,0,150,1000),DRIVE_STABLE(150,12000),HALT(1000),EMPTY(),EMPTY(),EMPTY(),EMPTY(),EMPTY()};
// 2 2 1 6 0 0 0 0 0

Task _steering_tasks[8]{HALT(1000),DRIVE_T(0,-150,0,-150,1000),DRIVE_STABLE(-150,1000),HALT(1000),ROTATE(30), DRIVE_STABLE(200,1500),HALT(1000),ROTATE(-30)};

#define ID_AVOID_CMPL     0x1
#define ID_PATHFIND_CMPL  0x2
void clearQueue(Queue& queue){
  for(int i = 0; i < MAX_TASKS; ++i) queue.setAbsolute(_empty_tasks[i],i);
}

int queueIndexLast = 0;
int queueIndexCurr = 0;

bool updateSensors = true;
bool updateHeading = true;
bool calculatingPath = false;

int timer = 0;
int rotateStallTimer = 0;

float pitch_angle = 0.f;
const int stallTimerThreshold = 3000;
const int rStallTimerThreshold = 5000;

void calculatePath();
bool detectStall();
void integrateVelocity();
void integratePosition();
bool isDriving();

void setup(){
  Serial.begin(9600);
  Wire.begin();

  IMU.initSensor();
  SET_9DOF_MODE(IMU,OPERATION_MODE_NDOF);
  IMU.setUpdateMode(MANUAL);


  for(int task = 0; task < MAX_TASKS; ++task) queue.setAbsolute(_init_tasks[task],task);

  initializeBitMatrix(obstacleMap,10,10);

  _steering_tasks[7].interruptID = ID_AVOID_CMPL;

  INIT_MOTOR_PINS();
}

void loop(){
  queue.startFrame();
  queueIndexLast = queue.iter;
  Serial.print(queue.wheelSpeed[0]);
  Serial.print(" ");
  Serial.println(queue.wheelSpeed[1]);

  if(updateHeading && ((millis() - IMUlastStreamTime) >= IMUstreamPeriod)){
    IMUlastStreamTime = millis();
    IMU.updateGyro();
    IMU.updateEuler();
    IMU.updateAccel();
    IMU.updateLinearAccel();
    IMU.updateGravAccel();

    acc = IMU.readLinearAccelX();
    if(fabsf(acc) < 0.1f) acc = 0.0f;
    sonicValues[0] = sonars[0].ping_median(2) / US_ROUNDTRIP_CM;
    sonicValues[1] = sonars[1].ping_median(2) / US_ROUNDTRIP_CM;
    sonicValues[2] = sonars[2].ping_median(2) / US_ROUNDTRIP_CM;

    pitch_angle = IMU.readEulerPitch();

    queue.sentinel[9] = pitch_angle;

    queue.sentinel[5] = pos.i;
    queue.sentinel[6] = pos.j;
    heading_angle = IMU.readEulerHeading();
    dir = Vec2(cos(heading_angle * DEG_TO_RAD),sin(heading_angle*DEG_TO_RAD));
    if(updateSensors){
      if(inThreshold(sonicValues[0])){
        clearQueue(queue);
        _steering_tasks[4].params[0] = -30;
        _steering_tasks[7].params[0] = 30;
        for(int i = 0; i < 8; ++i) queue.setRelative(_steering_tasks[i],i+1);
        queue.beginNext();
        queue.deltaT = 0;

        detect_point = pos + dir * ((f32)s_detect_threshold / 100.f);
        setBitMatrixElement(obstacleMap, detect_point.i, detect_point.j, true);
        updateSensors = false;
      }else if(inThreshold(sonicValues[2])){
        clearQueue(queue);

        _steering_tasks[4].params[0] = 30;
        _steering_tasks[7].params[0] = -30;
        for(int i = 0; i < 8; ++i) queue.setRelative(_steering_tasks[i],i+1);
        queue.beginNext();
        queue.deltaT = 0;

        detect_point = pos + dir * ((f32)s_detect_threshold / 100.f);
        setBitMatrixElement(obstacleMap, detect_point.i, detect_point.j, true);
        updateSensors = false;
      }
      else if(inThreshold(sonicValues[1])){
        if(sonicValues[0] > 0 && sonicValues[1] > 0){
            if(sonicValues[0] < sonicValues[1]){
              clearQueue(queue);
              _steering_tasks[4].params[0] = -30;
              _steering_tasks[7].params[0] = 30;
              for(int i = 0; i < 8; ++i) queue.setRelative(_steering_tasks[i],i+1);
              queue.beginNext();
              queue.deltaT = 0;

              detect_point = pos + dir * ((f32)s_detect_threshold / 100.f);
              setBitMatrixElement(obstacleMap, detect_point.i, detect_point.j, true);
              updateSensors = false;
            }
            else{
              clearQueue(queue);

              _steering_tasks[4].params[0] = 30;
              _steering_tasks[7].params[0] = -30;
              for(int i = 0; i < 8; ++i) queue.setRelative(_steering_tasks[i],i+1);
              queue.beginNext();
              queue.deltaT = 0;

              detect_point = pos + dir * ((f32)s_detect_threshold / 100.f);
              setBitMatrixElement(obstacleMap, detect_point.i, detect_point.j, true);
              updateSensors = false;
            }
        }
      }
      // 4 and 7
    }
  }


  queue.sentinel[2] = heading_angle;

  queue.executeAction();
  queueIndexCurr = queue.iter;

  if(queueIndexCurr != queueIndexLast)
  {
    queue.sentinel[0] = heading_angle;
    queue.sentinel[7] = pos.i;
    queue.sentinel[8] = pos.j;
  }

  if(!detectStall() && isDriving()){
    integratePosition();
  }

  if(isDriving() && (queue.curr->ID == T_DRIVE || queue.curr->ID == T_DRIVE_H || queue.curr->ID == T_DRIVE_T || queue.curr->ID == T_REVERSE_T || queue.curr->ID == T_DRIVE_D)){
    integrateVelocity();
  }


  switch(queue.currFlag){
    case ID_AVOID_CMPL:
      clearQueue(queue);
      for(int i = 0; i < MAX_TASKS; ++i) queue.setRelative(_init_tasks[i],i);
      updateSensors = true;
      break;
    default:
      break;
  }

  queue.endFrame();
}

bool detectStall(){
  if((queue.curr->ID == T_DRIVE || queue.curr->ID == T_DRIVE_H || queue.curr->ID == T_DRIVE_T) && magnitude(vel) <= 0.1f && abs(queue.wheelSpeed[0]) > 20 && abs(queue.wheelSpeed[1]) > 20){
    timer += queue.deltaT / 1000;
  }else{
    timer = 0;
  }

  if(timer >= stallTimerThreshold) return true;

  return false;
}

bool detectRotationStall(){
  if(queue.curr->ID == T_ROTATE) rotateStallTimer += queue.deltaT / 1000;
  else rotateStallTimer = 0;

  if(rotateStallTimer >= rStallTimerThreshold) return true;

  return false;
}

void integrateVelocity(){
  vel = vel + dir * acc * (((int)(queue.deltaT/1000))/1000.f);
  if(magnitude(vel) < 0.01f) vel = Vec2(0.f,0.f);
  if(abs(queue.wheelSpeed[0]) <= 20 && abs(queue.wheelSpeed[1]) <= 20) vel = Vec2(0.f,0.f);
}

void integratePosition(){
  pos = pos + vel * (((int)(queue.deltaT/1000))/1000.f);
}

bool isDriving(){
  return (abs(queue.wheelSpeed[0]) <= 20 && abs(queue.wheelSpeed[1]) <= 20);
}