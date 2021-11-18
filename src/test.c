
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
#define K_ALPHA 50
#define K_THETA 10

void align_bot_PID(double alpha, double theta) {
  double theta_err = theta - get_theta_from_alpha(alpha);
  double alpha_err = M_PI - alpha;  // target is M_PI

  int motorR = K_ALPHA * fabs(alpha_err) + (K_THETA * theta_err);
  int motorL = K_ALPHA * fabs(alpha_err) - (K_THETA * theta_err);

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