/****
 This file contains logic for the PIDS (rotation and straight)
****/

#ifndef _PID_C
#define _PID_C

#define FAST_MOTOR_SPEED 40
#define SLOW_MOTOR_SPEED 10

#include "roboAI.h"

double turn_on_spot_PID(double angle_error);
int turn_to_target(double current_heading_x, double current_heading_y, double target_heading_x, double target_heading_y, int is_precise);

double bootleg_turn_on_spot_PID(double angle_err){
  int angle_bound ;
  int  motorR_speed, motorL_speed;
  if(angle_err > 0.3 ){ 
    // Args:
//   angle_error : if negative turn right motor more  crossie(heading_dir, target_dir) <0
//                if positive turn left motor more crossie(heading_dir, target_dir) > 0
//                     this is because y axis is flipped
    motorR_speed =  (angle_err < 0)? 10:  -10;
    motorL_speed =  (angle_err < 0)? -10:  10;
  }else{ // PID slows bot if close enough and turns off other pid
    motorR_speed =  (angle_err < 0)? 5:  -5;
    motorL_speed =  (angle_err < 0)? -5:  5;
  }

  printf("motorL_speed %d motorL_speed %d \n",motorL_speed,motorR_speed );
  BT_motor_port_start(RIGHT_MOTOR, motorR_speed);
  BT_motor_port_start(LEFT_MOTOR, motorL_speed);

  return 1; 
}

double turn_on_spot_PID(double angle_error)
{
  // TODO: use pid_context_struct

  static double old_error = 0;
  static double i_error = 0;

  double p_error = angle_error;
  i_error += p_error;
  double d_error = angle_error - old_error;

  old_error = p_error;

  double k_p = 1, k_i = 0, k_d = 0, k_s = 0.15;
  double k_sum_pid = k_p + k_d + k_i;
  double normalized_k_p = k_p / k_sum_pid, normalized_k_i = k_i / k_sum_pid, normalized_k_d = k_d / k_sum_pid;
  double output = k_s * (normalized_k_p * p_error + normalized_k_i * i_error + normalized_k_d * d_error);
  if (fabs(output) > 1)
  {
    return output / fabs(output);
  }
  else
  {
    return output;
  }
}

/**
 * Returns angle error between the two vectors put in
 */
int turn_to_target(double current_heading_x, double current_heading_y, double target_heading_x, double target_heading_y, int is_precise)
{
  // printf("current_heading_x: %f  current_heading_y: %f\n", current_heading_x, current_heading_y);

  double current_heading_standard_angle = get_standard_angle_for_vector(current_heading_x, current_heading_y);
  double target_heading_standard_angle = get_standard_angle_for_vector(target_heading_x, target_heading_y);
  // printf("current_heading_standard_angle: %f  target_heading_standard_angle: %f\n", current_heading_standard_angle, target_heading_standard_angle);

  double angle_error = boundAngle180To180(target_heading_standard_angle - current_heading_standard_angle);
  printf("angle_error: %f\n", angle_error);
  double pid_out = turn_on_spot_PID(angle_error);
  double MAX_SPEED = 100;
  double MIN_SPEED = 10;

  //int motor_out = bootleg_turn_on_spot_PID(angle_error);
  int motor_out = round(pid_out * MAX_SPEED);
  // printf("motor_out: %d\n", motor_out);

  // // TODO : remove below if no work
  if (motor_out != 0 && abs(motor_out) < MIN_SPEED){ //sets min power the motors should get
    motor_out = MIN_SPEED * (motor_out/abs(motor_out));
  }

  //fflush(stdout);

  if (fabs(angle_error) < (is_precise ? 0.05: 0.1))
  { //event = facing ball
    printf("condition met\n");
    BT_motor_port_stop(LEFT_MOTOR, 1);
    BT_motor_port_stop(RIGHT_MOTOR, 1);
    return 1;
  }
  else
  {
    BT_motor_port_start(LEFT_MOTOR, -1 * motor_out);
    BT_motor_port_start(RIGHT_MOTOR, motor_out);
    return 0;
  }
}

#endif