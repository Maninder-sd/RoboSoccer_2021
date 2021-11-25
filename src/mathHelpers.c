#include <math.h>
#define IM_SIZE_X 1024
#define IM_SIZE_Y 768


inline double dottie(double vx, double vy, double ux, double uy)
{
 // Returns the dot product of the two vectors [vx,vy] and [ux,uy]
 return (vx*ux)+(vy*uy);
}

inline double crossie_sign(double vx, double vy, double ux, double uy)
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

inline double get_standard_y_postioin(double y_postion) {
  return IM_SIZE_Y - y_postion;
}

inline double dot_prod(double vx,double vy,double wx, double wy) {
    return vx*wx + vy*wy;
}


inline double get_degrees_for_radians(double rads) {
  return (rads / (2 * M_PI)) * 360;
}


inline double boundAngle0To360(double theta) {
  if (theta >= M_PI*2) return theta-M_PI*2;
  if (theta < 0) return theta + M_PI*2;
  return theta;
}

//input theta in [-180 - 359, 180 + 360]
inline double boundAngle180To180(double theta) {
  if (theta > M_PI) return theta-M_PI*2;
  if (theta <= -M_PI) return theta + M_PI*2;
  return theta;
}

double get_angle_btwn_vectors(double v1_x, double v1_y, double v2_x, double v2_y) {
  double dot = dot_prod(v1_x, v1_y, v2_x, v2_y);  // dot product
  double det = v1_x*v2_y - v1_y*v2_x;    // determinant
  double atan2_angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)
  
  if (atan2_angle < 0) {
      atan2_angle = (2 * M_PI) + atan2_angle;
  }
  
  return atan2_angle;
}

inline double get_standard_angle_for_vector(double v1_x, double v1_y) {
  static double STANDARD_X = 1, STANDARD_Y = 0;
  return get_angle_btwn_vectors(v1_x, v1_y, STANDARD_X, STANDARD_Y);
}