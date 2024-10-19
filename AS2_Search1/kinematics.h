// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H 
#include "encoders.h"
 
Encoders_c encodersk;
 
 
class Kinematics_c {
  public:
 
    float x;
    float y;
    float theta;
    int l = 48;
    int r = 16;

    // Constructor, must exist.
    Kinematics_c() {
 
    }
 
    void update(int e0,int e1) {
      float x_r = r*e0 * 3.14/180;
      float y_r = r*e1 * 3.14/180;
      float X_R = (x_r + y_r)/2;
      float t_R = (x_r - y_r)/(2*l);
      theta += t_R;
      x += X_R*(cos(theta));
      y += X_R*(sin(theta));
    }
 
    float xcord(){
      return x;
    }
 
    float ycord(){
      return y;
    }
 
    float theta_rad(){
      return theta;
    }
 
    
};
#endif
