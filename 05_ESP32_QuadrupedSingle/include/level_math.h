#pragma once
namespace LevelMath {
constexpr float FRACTION_RANGE=0.25f;
constexpr float MIN_FRACTION=0.5f-FRACTION_RANGE, MAX_FRACTION=0.5f+FRACTION_RANGE;
constexpr float abs(float x) { return x<0 ? -x:x; }
constexpr float clamp(float x,float lo,float hi) { return x<lo ? lo:x>hi ? hi:x; }
constexpr float advance(float correction,float error,float dt) {
  return abs(error)<=0.7f ? correction:clamp(correction+clamp(error*0.01f,-0.05f,0.05f)*clamp(dt,0,0.1f),-FRACTION_RANGE,FRACTION_RANGE);
}
struct Mix { float fraction[4]={}; bool saturated=false; };
// Body axes: X forward, Y left. Positive roll raises left; positive pitch raises front.
constexpr Mix mix(float roll,float pitch,int reference) {
  constexpr int r[4]={1,-1,1,-1},p[4]={1,1,-1,-1};
  Mix out{};
  if(reference<0 || reference>3) return out;
  float delta[4]={},maximum=0;
  for(int i=0;i<4;++i) {
    delta[i]=(r[i]-r[reference])*roll+(p[i]-p[reference])*pitch;
    if(abs(delta[i])>maximum) maximum=abs(delta[i]);
  }
  const float scale=maximum>FRACTION_RANGE ? FRACTION_RANGE/maximum:1.0f;
  out.saturated=maximum>=FRACTION_RANGE-0.0001f;
  for(int i=0;i<4;++i) out.fraction[i]=0.5f+delta[i]*scale;
  return out;
}
// On this chassis, a rear lift increases IMU roll and a right-side lift
// decreases IMU pitch. Transform the actuator corrections without changing IMU readings.
constexpr Mix mixForImu(float imuRoll,float imuPitch,int reference) {
  return mix(imuPitch,-imuRoll,reference);
}
constexpr bool invariants() {
  for(int ref=0;ref<4;++ref) for(int r=-10;r<=10;++r) for(int p=-10;p<=10;++p) {
    const auto out=mixForImu(r*0.02f,p*0.02f,ref);
    if(abs(out.fraction[ref]-0.5f)>0.00001f) return false;
    for(float v:out.fraction) if(v<MIN_FRACTION-0.00001f || v>MAX_FRACTION+0.00001f) return false;
  }
  return true;
}
static_assert(invariants(),"Reference leg must hold midpoint; other legs remain within the leveling range.");
static_assert(mixForImu(0.01f,0,0).fraction[2]>0.5f &&
              mixForImu(0.01f,0,0).fraction[1]==0.5f,"Positive IMU roll correction raises the rear.");
static_assert(mixForImu(0,0.01f,0).fraction[1]<0.5f &&
              mixForImu(0,0.01f,0).fraction[2]==0.5f,"Positive IMU pitch correction lowers the right side.");
// Compile-time ideal-plane simulation. This checks feedback signs and reference
// independence; real linkage gain/support still requires physical validation.
constexpr bool convergence() {
  for(int ref=0;ref<4;++ref) for(int sign=-1;sign<=1;sign+=2) {
    float r=0,p=0,roll=0,pitch=0;
    for(int tick=0;tick<200;++tick) {
      r=advance(r,sign*1-roll,0.1f); p=advance(p,-sign*1-pitch,0.1f);
      const auto target=mixForImu(r,p,ref);
      roll=30*(target.fraction[2]+target.fraction[3]-target.fraction[0]-target.fraction[1])/4;
      pitch=30*(target.fraction[0]-target.fraction[1]+target.fraction[2]-target.fraction[3])/4;
    }
    if(abs(roll)<0.25f || abs(pitch)<0.25f || abs(roll-sign*1)>0.71f || abs(pitch+sign*1)>0.71f) return false;
  }
  return true;
}
static_assert(convergence(),"Level feedback signs must converge for every reference leg in the ideal plane model.");
static_assert(advance(FRACTION_RANGE,100,1)==FRACTION_RANGE && advance(-FRACTION_RANGE,-100,1)==-FRACTION_RANGE,"Integral correction must remain bounded.");
}
