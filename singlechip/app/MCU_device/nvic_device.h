#ifndef NVIC_DEVICE_H
#define NVIC_DEVICE_H
#include "tim.h"

typedef void (*nvic_callback)(void);

void start_interrupt(void);
void nvic_register_callback(nvic_callback fun);

#endif

