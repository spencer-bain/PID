#ifndef PID_H
#define PID_H

#include<stdio.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>

struct Pid{
	uint16_t kd;
	uint16_t kp;
	uint16_t ki;

	int16_t set_point;
	int16_t error;
	int16_t measurement;
	int16_t last_error;
	int16_t integral;
	uint16_t time_step;//this should be in clock cylces
};

void set_gains(struct Pid *pid, uint16_t kd, uint16_t kp, uint16_t ki);
void set_setpoint(struct Pid *pid, int16_t set_point);
void set_time_step(struct Pid *pid, uint16_t clock_cycles);
int16_t update_control_signal(struct Pid *pid, int16_t measurmeant);
#endif
