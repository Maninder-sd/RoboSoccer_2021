
#include <math.h>
#include <stdio.h>

static double det(double a_x, double a_y, double b_x, double b_y) {
  return a_x * b_y - a_y * b_x;
}

static double crossie_sign_2(double vx, double vy, double ux, double uy)
{
 // Returns the sign of the Z component of the cross product of 
 //   vectors [vx vy 0] and [ux uy 0]
 // MIND THE ORDER! v rotating onto u.     
 // i   j   k 
 // vx  vy  0
 // ux  uy  0
    
 if ((vx*uy)-(ux*vy)<0) return -1;
 else return 1;
}

static double dottie_vector(double v[2], double u[2])
{
 // Returns the dot product of the two vectors 
 return (v[0]*u[0])+(v[1]*u[1]);
}


void finding_errors( double ball_pos[2], double bot_pos[2], double target_pos[2] , double * distance_err, double* lateral_err){

    *distance_err = sqrt(  (ball_pos[0] - bot_pos[0])*(ball_pos[0] - bot_pos[0]) +   (ball_pos[1] - bot_pos[1])*(ball_pos[1] - bot_pos[1]) );
    
    double ball_to_goal[2];
    ball_to_goal[0] =  target_pos[0] -ball_pos[0];
    ball_to_goal[1] =  target_pos[1] -ball_pos[1];
    
    double bot_to_ball[2];
    bot_to_ball[0] =  ball_pos[0] -bot_pos[0];
    bot_to_ball[1] =  ball_pos[1] -bot_pos[1];
    
    double projected_dist= dottie_vector(ball_to_goal, bot_to_ball) / sqrt(dottie_vector(ball_to_goal, ball_to_goal));

    *lateral_err = sqrt((*distance_err)*(*distance_err) - projected_dist*projected_dist)
      * crossie_sign_2(ball_to_goal[0], ball_to_goal[1], bot_to_ball[0], bot_to_ball[1]);

}

#define INTEGRATION_DEPTH 10

double align_straight_to_target_PID(double lateral_err) {

  static double old_error = 0;
  static double i_errors[INTEGRATION_DEPTH] ;
  static int i_index; //index to change 

  double i_error=0;


  double p_error = lateral_err;
  double d_error = (lateral_err - old_error)/ 0.083;
  old_error = lateral_err;

  i_errors[i_index] = lateral_err;
  i_index =(i_index+1) % INTEGRATION_DEPTH;
  for(int i=0; i<INTEGRATION_DEPTH; i++){
    i_error+=i_errors[i];
  }


  double k_p = 1, k_i = 0, k_d = 0, k_s = 0.015;
  double k_sum_pid = k_p + k_d + k_i;
  double normalized_k_p = k_p / k_sum_pid, normalized_k_i = k_i / k_sum_pid, normalized_k_d = k_d / k_sum_pid; 
  double output = k_s * (normalized_k_p * p_error + normalized_k_i * i_error + normalized_k_d * d_error);
  if (fabs(output) > 1) {
    return output / fabs(output);
  } else {
    return output;
  }
  
}


int  simple_straight_to_target_PID(double distance_err, double angle_error) {
  /*
    Index rules:
    0 - distance_err
    1 - bot_heading_to_target_angle

    -20 deg< angle_error < 20 deg otherwise angle too steep
    20 deg = 0.35 rad

    return 1 if running
    return 0 if done
    return -1 if angle too steep 
  */

  double angle_bound=0.35;

  static double old_error[2] = {0,0};
  static double i_errors[INTEGRATION_DEPTH][2] ;
  static int i_index; //index to change 

  double P_error[2] = {distance_err, angle_error};

  // We dont use the ID in PID -----------
  double D_error[2] = {(P_error[0] - old_error[0])/ 0.1, (P_error[1] - old_error[1])/ 0.1 };
  old_error[0] = distance_err;
  old_error[1] = angle_error;
  double I_error[2]={0,0}; //the running sum of i_errors[]
  i_errors[i_index][0] = distance_err;
  i_errors[i_index][1] = angle_error;
  i_index =(i_index+1) % INTEGRATION_DEPTH;
  for(int i=0; i<INTEGRATION_DEPTH; i++){
    I_error[0]+=i_errors[i][0];
    I_error[1]+=i_errors[i][1];
  }
  // ---------------------------------------

  if(fabs(P_error[1]) > angle_bound){ // this PID stops motors if angle to steep
    BT_motor_port_stop(RIGHT_MOTOR, 0);
    BT_motor_port_stop(LEFT_MOTOR, 0);
    return 0;
  }

  double motorR_speed, motorL_speed;
  if(P_error[0] > 200){ 
    motorR_speed = 100;
    motorL_speed = 100;
// Args:
//   angle_error : if negative turn right motor more  crossie(heading_dir, target_dir) <0
//                if positive turn left motor more crossie(heading_dir, target_dir) > 0
//                     this is because y axis is flipped
    double turn_speed = (10*  P_error[1]/angle_bound );  
    motorR_speed -= turn_speed;
    motorL_speed += turn_speed;
  }else{ // PID slows bot if close enough and turns off other pid
    motorR_speed = 20;
    motorL_speed = 20;
  }


  BT_motor_port_start(RIGHT_MOTOR, motorR_speed);
  BT_motor_port_start(LEFT_MOTOR, motorL_speed);

  return 1; // todo: more here
}


double drive_straight_to_target_PID(double distance_err) {

  static double old_error = 0;
  static double i_errors[INTEGRATION_DEPTH] ;
  static int i_index; //index to change 

  double i_error=0;


  double p_error = distance_err;
  double d_error = (distance_err - old_error)/ 0.083;
  old_error = distance_err;

  i_errors[i_index] = distance_err;
  i_index =(i_index+1) % INTEGRATION_DEPTH;
  for(int i=0; i<INTEGRATION_DEPTH; i++){
    i_error+=i_errors[i];
  }


  double k_p = 1, k_i = 0, k_d = 0, k_s = 0.0025;
  double k_sum_pid = k_p + k_d + k_i;
  double normalized_k_p = k_p / k_sum_pid, normalized_k_i = k_i / k_sum_pid, normalized_k_d = k_d / k_sum_pid; 
  double output = k_s * (normalized_k_p * p_error + normalized_k_i * i_error + normalized_k_d * d_error);
  if (fabs(output) > 1) {
    return output / fabs(output);
  } else {
    return output;
  }

  
  
}

// void apply_PID_outputs(double distance_output, double lateral_output) {

//   double leftMotorInput = 0;
//   double rightMotorInput = 0;

//   if (lateral_output > 0) {

//   }
//   BT_motor_port_start(motorR, (distance_output + latera_ou)
// }

// int static main() {
//   double ball_pos[2] = {3, 2}, bot_pos[2] = {3,4}, target_pos[2] = {0,5};
//   double distance_err, lateral_err;
//   finding_errors(ball_pos, bot_pos, target_pos, &distance_err, &lateral_err);
//   printf("distance_err: %f  lateral_err:%f\n", distance_err, lateral_err);
//   return 0;
// }