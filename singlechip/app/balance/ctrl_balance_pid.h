#ifndef balance_pid_h
#define balance_pid_h

#include "stdio.h"
#include<stdlib.h>
#include<math.h>

float pid_vertical(float target, float feedback, float gyro);
float pid_speed(float target, float feedback);

#ifdef DEBUG_PID
void set_PID(int _KP, int _KD, int _S_KP, int _S_KI);
#endif

#endif
