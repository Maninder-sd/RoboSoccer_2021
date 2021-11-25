
#include <math.h>
#include <stdio.h>

static inline double det(double a_x, double a_y, double b_x, double b_y) {
  return a_x * b_y - a_y * b_x;
}

static inline double crossie_sign2(double vx, double vy, double ux, double uy)
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

static inline double dottie_vector(double v[2], double u[2])
{
 // Returns the dot product of the two vectors 
 return (v[0]*u[0])+(v[1]*u[1]);
}

static inline double boundAngle0To360_2(double theta) {
  if (theta >= M_PI*2) return theta-M_PI*2;
  if (theta < 0) return theta + M_PI*2;
  return theta;
}


static inline double dottie2(double vx, double vy, double ux, double uy)
{
 // Returns the dot product of the two vectors [vx,vy] and [ux,uy]
 return (vx*ux)+(vy*uy);
}

static double get_magnitude(double x, double y) {
  return pow(x*x + y*y, .5);
}

static double get_signed_angle_from_vectors(double ux, double uy, double vx, double vy) {
  return crossie_sign2(ux, uy, vx, vy) *
      acos(dottie2(ux, uy, vx, vy) /
         (get_magnitude(ux, uy) *
         get_magnitude(vx, vy)));
}

static double get_curr_angle_from_line(double ball_pos[2], double target_pos[2], double dx, double dy) {
  double ball_to_goal[2];
    ball_to_goal[0] =  target_pos[0] -ball_pos[0];
    ball_to_goal[1] =  target_pos[1] -ball_pos[1];

    double curr_angle_signed = get_signed_angle_from_vectors(dx, dy, ball_to_goal[0], ball_to_goal[1]);
    return boundAngle0To360_2(curr_angle_signed);

}

static double get_target_angle_from_line(double lateral_err, double ball_pos[2], double target_pos[2]) {

  double k1 = .01;

  double relative_target_y_heading, relative_target_x_heading;
  if (fabs(lateral_err) < 0.1) {
    relative_target_y_heading = 0;
    relative_target_x_heading = -1;
  } else {
    relative_target_y_heading = -(fabs(lateral_err)/lateral_err)*k1;
    relative_target_x_heading = -1 / fabs(.5*lateral_err);
  }
  //double relative_target_x_heading = (1 / (fabs(lateral_err) + (lateral_err == 0))) - (lateral_err == 0);

  double ball_to_goal[2];
  ball_to_goal[0] =  target_pos[0] -ball_pos[0];
  ball_to_goal[1] =  target_pos[1] -ball_pos[1];

  double target_angle_signed = get_signed_angle_from_vectors(relative_target_x_heading, relative_target_y_heading,
      ball_to_goal[0], ball_to_goal[1]);

  return  boundAngle0To360_2(target_angle_signed);


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
      * crossie_sign2(ball_to_goal[0], ball_to_goal[1], bot_to_ball[0], bot_to_ball[1]);

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