#ifndef QUEUE_H
#define QUEUE_H

#include "queue/taskID.h"
#include "types.h"

#define LEFT        0
#define RIGHT       1

#define MAX_PARAMS  5
#define MAX_TASKS   15


// Type to hold Task information, uses ID for identification
struct Task{
    s8 ID = T_EMPTY;
    s16 params[MAX_PARAMS]{};

    bool completionInterrupt = false;
    s8 interruptID = INTR_COMPLETE;

    Task();
    Task(s16 args[], s16 numArgs,s8 ID);
    Task(s16 a,s16 b,s16 c,s16 d,s16 e, s16 f, s8 ID); 
    Task(const Task& task);
    Task& operator=(const Task& second);
};

// Type to hold list of task, execute them & read frame time info
struct Queue{
    Task tasks[MAX_TASKS]{};

    Task * curr = &tasks[0];
    Task * next = &tasks[1];

    u8 iter = 0;
    s8 currFlag = INTR_EMPTY;
    s8 currFlagLeading = INTR_EMPTY;

    s16 wheelSpeed[2]{};

    u64 startTime = 0ull;
    u32 deltaT = 1ul;
    s32 timeAccumulator = 0l;

    f32 sentinel[10]{};
    bool sentinelSet = false;
    s16 sNumbers = 0;

    Queue();
    Queue(Task init[], s16 sz);
    ~Queue();

    void setRelative(Task task, s16 offset);
    void setAbsolute(Task task, u16 pos);
    bool pushToNextEmpty(Task task);

    void startFrame();
    void executeAction();
    void endFrame();

    void beginNext();
};

#endif