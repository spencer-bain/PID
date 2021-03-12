//Written by Spencer Bain
#include"pid.h"

void set_gains(struct Pid *pid, uint16_t kd, uint16_t kp, uint16_t ki){
	pid->kd = kd;
	pid->kp = kp;
	pid->ki = ki;
}

void set_setpoint(struct Pid *pid, int16_t set_point){
	pid->set_point = set_point;
}

void set_time_step(struct Pid *pid, uint16_t clock_cycles){
	pid->time_step = clock_cycles;
}

int16_t update_control_signal(struct Pid *pid, int16_t measuremeant){
	int16_t control_signal = 0;

	int16_t derivative = (pid->last_error - pid->error) / pid->time_step;
	pid->error = pid->set_point - measuremeant;	
	pid->integral += (pid->last_error - pid->error) * pid->time_step;

	control_signal += derivative * pid->kd;
	control_signal += pid->error * pid->kp;
	control_signal += pid->integral * pid->ki;

	pid->last_error = pid->error;
	
	return control_signal;
}
