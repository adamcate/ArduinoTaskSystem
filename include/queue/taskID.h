#ifndef TASK_ID_H
#define TASK_ID_H

#include "queue/queue.hpp"

// ID Flags:
#define T_EMPTY     0x0
#define T_DRIVE     0x1
#define T_DRIVE_T   0x2
#define T_REVERSE_T 0x3
#define T_WAIT_T    0x4
#define T_ROTATE    0x5
#define T_HALT      0x6
#define T_DRIVE_H   0x8

// Task constructor macros:
#define EMPTY()                                     Task(0,0,0,0,0,0,T_EMPTY)
#define DRIVE(left, right, time)                    Task(left,right,0,0,0,time,T_DRIVE)
#define DRIVE_T(left1, left2, right1, right2, time) Task(left1,left2,right1,right2,0,time,T_DRIVE_T)
#define REVERSE_T(left,right,time)                  Task(left,right,0,0,0,time,T_REVERSE_T)
#define HALT(time)                                  Task(0,0,0,0,0,time,T_HALT)
#define ROTATE(angle)                               Task(angle,0,0,0,0,0,T_ROTATE)
#define WAIT_T(time)                                Task(0,0,0,0,0,time,T_WAIT_T)
#define DRIVE_STABLE(speed,time)                    Task(speed,0,0,0,0,time,T_DRIVE_H)

// Completion interrupt flags:
#define INTR_EMPTY          0x0
#define INTR_COMPLETE       0x1
#define INTR_SEG_COMPLETE   0x2

#endif