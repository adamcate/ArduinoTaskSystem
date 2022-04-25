#include "queue/queue.hpp"
#include "queue/driving.hpp"
#include "mathLib/angles.hpp"
//#include <Arduino.h>


Task::Task(){}

Task::Task(s16 args[], s16 numArgs,s8 ID){
    if (numArgs > MAX_PARAMS) return;
    for(s16 i = 0; i < numArgs; ++i)
        params[i] = args[i];
    this->ID = ID;
}
Task::Task(s16 a,s16 b,s16 c,s16 d,s16 e, s16 f, s8 ID){
    params[0] = a;
	params[1] = b;
	params[2] = c;
	params[3] = d;
	params[4] = e;
    params[5] = f;
    this->ID = ID;
}

Task::Task(const Task& task){
    ID = task.ID;
    for(s16 i = 0; i < MAX_PARAMS; ++i)
        params[i] = task.params[i];
    completionInterrupt = task.completionInterrupt;
    interruptID = task.interruptID;
}
Task& Task::operator=(const Task& second){
    for(int i =0; i < MAX_PARAMS; ++i) params[i] = second.params[i];
    ID = second.ID;
    interruptID = second.interruptID;
    return *this;
}
// Queue constructors

Queue::Queue(){
    curr = &tasks[0];
    next = &tasks[1];
}

Queue::Queue(Task init[], s16 sz){
	for(s16 i = 0; i < sz; ++i)
		setAbsolute(init[i],i);
}

Queue::~Queue(){ // take care of dangling pointers
	curr = nullptr;
	next = nullptr;
}
// add specified task to next empty index, return true if succeeded,
// return false if no empty slot was found
bool Queue::pushToNextEmpty(Task task){
	for(s16 i = 0; i < MAX_TASKS; ++i){
		if(tasks[(iter+i)%MAX_TASKS].ID == T_EMPTY){
			iter = 0;
			tasks[(iter+i)%MAX_TASKS] = task;
			return true;
		}
	}
	return false;
}
// set the task at an index relative to the current action:
// wraps around when exceeding size
void Queue::setRelative(Task task, s16 offset){
	tasks[(iter+offset)%MAX_TASKS] = task;
}
// set task at absolute index, unless out of bounds
void Queue::setAbsolute(Task task, u16 pos){
	tasks[pos] = task;
}
// step forward in Queue and reset timers
void Queue::beginNext(){
	currFlag = curr->interruptID;
    currFlagLeading = next->interruptID;

	curr = &tasks[(iter+1)% MAX_TASKS];
	next = &tasks[(iter+2)% MAX_TASKS];

	timeAccumulator = 0;

    ++iter;
	iter = iter % MAX_TASKS;

    sentinel[3] = wheelSpeed[LEFT];
    sentinel[4] = wheelSpeed[RIGHT];
}
// get the start time & reset flag to -1
void Queue::startFrame(){
	startTime = micros();
	currFlag = -1;
    currFlagLeading = -1;
}
// calculate frame time by time difference from startTime
void Queue::endFrame(){
	deltaT = micros() - startTime + 1ul;
}

void Queue::executeAction(){	// really gross long switch statement

	timeAccumulator += deltaT / 1000;


    f32 frac = static_cast<f32>(timeAccumulator) / curr->params[5];

    f32 difference = 0.f;

    switch(curr->ID){
        case T_EMPTY:
            drive(sentinel[3],sentinel[4]);
            beginNext();
            return;
        case T_DRIVE:
            wheelSpeed[LEFT] = curr->params[0];
		    wheelSpeed[RIGHT] = curr->params[1];
            drive(wheelSpeed[LEFT],wheelSpeed[RIGHT]);
            break;
        case T_DRIVE_T:
            wheelSpeed[LEFT] = lerpf(curr->params[0],curr->params[1],frac);
		    wheelSpeed[RIGHT] = lerpf(curr->params[2],curr->params[3],frac);
            drive(wheelSpeed[LEFT],wheelSpeed[RIGHT]);
            break;
        case T_REVERSE_T:
		    wheelSpeed[LEFT] = lerpf(curr->params[0],-1.f * curr->params[0],frac);
		    wheelSpeed[RIGHT] = lerpf(curr->params[1],-1.f * curr->params[1],frac);
            drive(wheelSpeed[LEFT],wheelSpeed[RIGHT]);
            break;
        case T_HALT:
            wheelSpeed[LEFT] = lerpf(sentinel[3],0.f,frac);
		    wheelSpeed[RIGHT] = lerpf(sentinel[4],0.f,frac);
            drive(wheelSpeed[LEFT],wheelSpeed[RIGHT]);
            break;
        case T_ROTATE:
            // TODO add watcher & ROTATE code
            // also ensure that the code is getting the correct angular displacement direction
            difference = angleDiff(sentinel[2],MOD_U(sentinel[0]+curr->params[0],360.f));
            //Serial.print(sentinel[0]);
            if(fabsf(difference) >= 10.f)
            {
                wheelSpeed[0] = (difference) * 2;
                wheelSpeed[1] = -2 * difference;
                /*if(abs(wheelSpeed[0]) > 200) wheelSpeed[0] = (wheelSpeed[0] >= 0 ? 1 : -1) * 140;
                if(abs(wheelSpeed[1]) > 200) wheelSpeed[1] = (wheelSpeed[1] >= 0 ? 1 : -1) * 140;*/


                if(abs(wheelSpeed[0]) < 255) wheelSpeed[0] = (wheelSpeed[0] >= 0 ? 1 : -1) * 255;
                if(abs(wheelSpeed[1]) < 255) wheelSpeed[1] = (wheelSpeed[1] >= 0 ? 1 : -1) * 255;

                drive(wheelSpeed[0],wheelSpeed[1]);
                //else if (difference < 0.f) drive(curr->params[0] - 10 * difference, curr->params[0]);
                return;
            }else{
                wheelSpeed[0] = 0;
                wheelSpeed[1] = 0;
            }
            beginNext();
            return;
        case T_DRIVE_H:
            difference = angleDiff(sentinel[2],sentinel[0]);

            if(fabsf(difference) >= 10.f)
            {
                
                if(difference > 0.f)
                {
                    wheelSpeed[0] = (float)curr->params[0];
                    wheelSpeed[1] = (float)curr->params[0] - 2.f * difference;

                    drive(wheelSpeed[0],wheelSpeed[1]);
                }
                else{
                    wheelSpeed[0] = (float)curr->params[0] - 2.f * difference;
                    wheelSpeed[1] = (float)curr->params[0];

                    drive(wheelSpeed[0],wheelSpeed[1]);
                }
                //else if (difference < 0.f) drive(curr->params[0] - 10 * difference, curr->params[0]);
            }
            else{
                drive(curr->params[0],curr->params[0]);
                wheelSpeed[0] = curr->params[0];
                wheelSpeed[1] = curr->params[0];
            }
            break;
        case T_DRIVE_D:
            difference = angleDiff(sentinel[2],sentinel[0]);
            
            if(fabsf(difference) >= 10.f)
            {
                
                if(difference > 0.f)
                {
                    wheelSpeed[0] = (float)curr->params[0];
                    wheelSpeed[1] = (float)curr->params[0] - difference;

                    drive(wheelSpeed[0],wheelSpeed[1]);
                }
                else{
                    wheelSpeed[0] = (float)curr->params[0] + difference;
                    wheelSpeed[1] = (float)curr->params[0];

                    drive(wheelSpeed[0],wheelSpeed[1]);
                }
                //else if (difference < 0.f) drive(curr->params[0] - 10 * difference, curr->params[0]);
            }
            else{
                drive(curr->params[0],curr->params[0]);
                wheelSpeed[0] = curr->params[0];
                wheelSpeed[1] = curr->params[0];
            }
            f32 disp = magnitude(Vec2(sentinel[5],sentinel[6])-Vec2(sentinel[7],sentinel[8]));
            /*Serial.print("p: ");
                Serial.println(disp);*/
            if(disp >= (f32)curr->params[1] / 100.f){
                
                beginNext();
                return;
            }
            break;
        default:
            break;
    }
    if(timeAccumulator >= curr->params[5] && curr->ID != T_DRIVE_D){
        beginNext();
        return;
    }
}