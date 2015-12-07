#ifndef _MOTOR_H
#define _MOTOR_H
#include "canfestival.h"

UNS8 motor_get_state(UNS16 state);
char* motor_get_state_title(UNS8 key);
char* motor_get_error_title(UNS16 data);
int switch_motor_on(char* slave_id);
int switch_motor_off(char* slave_id);

int motor_change_profile(char* slave_id, int Vmode);
int motor_target(UNS16 state);
int motor_start(char* slave_id,int start); // SON -> OP
int motor_forward(char* slave_id,int polar ); // Polarity

int motor_check_velocity(INTEGER32 acc);
int motor_check_acceleration(INTEGER32 acc);
int motor_check_torque(INTEGER16 torque);
int motor_check_torque_slope(INTEGER32 torqueSlope);

int motor_set_torque_speed(UNS8 dat);
UNS8 motor_get_torque_speed();

#endif
