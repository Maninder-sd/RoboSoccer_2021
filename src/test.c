
#include <math.h>

double det(double a_x, double a_y, double b_x, double b_y) {
  return a_x * b_y - a_y * b_x;
}

double get_angle(double a_x, double a_y, double b_x, double b_y) {
  double angle;
  double dot = dottie(a_x, a_y, b_x, b_y);  // find dot product
  double norm_a = dottie(a_x, a_y, a_x, a_y);
  norm_a = sqrt(norm_a);
  double norm_b = dottie(b_x, b_y, b_x, b_y);
  norm_a = sqrt(norm_a);
  angle = acos(dot / (norm_a * norm_b));

  double determinant = det(a_x, a_y, b_x, b_y);
  if (determinant < 0) {
    angle += M_PI;
  }
  return angle;
}

double get_theta_from_alpha(double x) {
  // x in the alpha
  x = (M_PI - x);
  if (x < 0) {
    x += 2 * M_PI;
  }

  int n = 6;
  double theta;
  theta = pow(fabs((1 / M_PI) * (x - M_PI)), n) - 1;
  theta = (-M_PI / 2) * theta;
  return theta;
}
#define K_ALPHA 30
#define K_THETA 10

void align_bot_PID(double alpha, double theta) {
  double theta_err = theta - get_theta_from_alpha(alpha);
  double alpha_err = M_PI - alpha;  // target is M_PI

  int motorR = K_ALPHA * fabs(alpha_err) + (K_THETA * theta_err);
  int motorL = K_ALPHA * fabs(alpha_err) - (K_THETA * theta_err);
  printf("Motor_L : %f Motor_R : %f \n", motorL, motorR);

  BT_motor_port_start(MOTOR_A, motorR);  // set right motor speed
  BT_motor_port_start(MOTOR_B, motorL);  // set right motor speed
}

int main() {
  double d_r1[2];
  d_r1[0] = dir.x;
  d_r1[0] = dir.x;
  double v_b[2];
  double v_r1[2];
  double v_r2[2];
}



double dottie(double v[2], double u[2])
{
 // Returns the dot product of the two vectors 
 return (v[0]*u[0])+(v[1]*u[1]);
}


finding_errors( double ball_pos[2], double bot_pos[2], double target_pos[2] , double * distance_err, double* lateral_err){

    *distance_err = sqrt(  (ball_pos[0] - bot_pos[0])*(ball_pos[0] - bot_pos[0]) +   (ball_pos[1] - bot_pos[1])*(ball_pos[1] - bot_pos[1]) );
    
    double goal_to_ball[2];
    goal_to_ball[0] =  target_pos[0] -ball_pos[0];
    goal_to_ball[1] =  target_pos[1] -ball_pos[1];
    
    double bot_to_ball[2];
    bot_to_ball[0] =  ball_pos[0] -bot_pos[0];
    bot_to_ball[1] =  ball_pos[1] -bot_pos[1];
    
  
    double projected_dist= dottie(goal_to_ball, bot_to_ball) / sqrt(dottie(goal_to_ball, goal_to_ball));

    *lateral_err = sqrt((*distance_err)*(*distance_err) - projected_dist*projected_dist);

}





#define INTEGRATION_DEPTH 10

double drive_straight_to_target_PID(double distance_err) {
  int motorL,motorR;

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


  double k_p = 1, k_i = 0, k_d = 0, k_s = 0.25;
  double k_sum_pid = k_p + k_d + k_i;
  double normalized_k_p = k_p / k_sum_pid, normalized_k_i = k_i / k_sum_pid, normalized_k_d = k_d / k_sum_pid; 
  double output = k_s * (normalized_k_p * p_error + normalized_k_i * i_error + normalized_k_d * d_error);
  if (fabs(output) > 1) {
    return output / fabs(output);
  } else {
    return output;
  }
  
}