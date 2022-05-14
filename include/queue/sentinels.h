#ifndef SENTINELS_H

#define SENTINELS_H


#define SET_START_ANGLE(angle, queue){\
    queue.sentinel[0] = angle;                 \
}

#define SET_CURR_ANGLE(angle, queue){\
    queue.sentinel[2] = angle;                \
}


#endif