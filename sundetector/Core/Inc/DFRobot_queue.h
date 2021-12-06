#ifndef DFROBOT_QUEUE_H_
#define DFROBOT_QUEUE_H_
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
struct sQueueData{
    struct sQueueData *next;
    uint8_t handle;
    uint8_t len;
    uint8_t data[];
};


extern void cuappEnqueue(uint8_t *pbuf, uint16_t len, uint16_t conn_id);
extern struct sQueueData *cuappDequeue( void );
extern struct sQueueData *getQueueTail(void);
extern struct sQueueData *getQueueHead(void);
#endif
