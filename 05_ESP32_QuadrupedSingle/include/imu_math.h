#pragma once
#include <math.h>

namespace ImuMath {
constexpr float PI_F=3.14159265358979323846f,DEG=180.0f/PI_F;
inline float wrap(float a) { return remainderf(a,360.0f); }
inline float clamp(float x,float a,float b) { return fmaxf(a,fminf(x,b)); }
struct Attitude {
  float w=1,x=0,y=0,z=0;
  bool initialized=false;
  void reset(float ax,float ay,float az) {
    const float r=atan2f(ay,az),p=atan2f(-ax,sqrtf(ay*ay+az*az));
    w=cosf(r/2)*cosf(p/2); x=sinf(r/2)*cosf(p/2);
    y=cosf(r/2)*sinf(p/2); z=-sinf(r/2)*sinf(p/2); initialized=true;
  }
  void step(float ax,float ay,float az,float gx,float gy,float gz,float dt) {
    if(!initialized || dt<=0 || dt>0.1f) { reset(ax,ay,az); return; }
    gx/=DEG; gy/=DEG; gz/=DEG;
    const float norm=sqrtf(ax*ax+ay*ay+az*az);
    // Gravity correction is reduced during obvious linear acceleration.
    if(norm>0.85f && norm<1.15f) {
      ax/=norm; ay/=norm; az/=norm;
      const float vx=2*(x*z-w*y),vy=2*(y*z+w*x),vz=1-2*(x*x+y*y);
      gx+=2*(ay*vz-az*vy); gy+=2*(az*vx-ax*vz); gz+=2*(ax*vy-ay*vx);
    }
    const float a=w,b=x,c=y,d=z,h=dt/2;
    w+=(-b*gx-c*gy-d*gz)*h; x+=(a*gx+c*gz-d*gy)*h;
    y+=(a*gy-b*gz+d*gx)*h; z+=(a*gz+b*gy-c*gx)*h;
    const float n=sqrtf(w*w+x*x+y*y+z*z);
    if(!isfinite(n) || n<0.001f) { reset(ax,ay,az); return; }
    w/=n; x/=n; y/=n; z/=n;
  }
  float roll() const { return atan2f(2*(w*x+y*z),1-2*(x*x+y*y))*DEG; }
  float pitch() const { return -asinf(clamp(2*(w*y-z*x),-1,1))*DEG; } // Nose up positive.
  float yaw() const { return atan2f(2*(w*z+x*y),1-2*(y*y+z*z))*DEG; }
};
}
