#include "north.h"
#include "north_calibration.h"
#include <math.h>

#define CHECK(x) do { if (!(x)) return 4000+__LINE__; } while (0)
static const north_calibration_t identity = {
  .accel_scale = {1,1,1}, .mag_matrix = {{1,0,0},{0,1,0},{0,0,1}}, .field = 50
};
static float error(float a, float b) { return remainderf(a-b, 360); }

/* Independent ZYX rotation: body readings from a known world orientation.
   World magnetic field has horizontal 30 uT and vertical -40 uT. */
static void rotation(float r, float p, float yaw, float R[3][3]) {
  float c = cosf(yaw), s = sinf(yaw), cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);
  float values[3][3] = {{c*cp,c*sp*sr-s*cr,c*sp*cr+s*sr},
                        {s*cp,s*sp*sr+c*cr,s*sp*cr-c*sr}, {-sp,cp*sr,cp*cr}};
  for (int i=0;i<3;++i) for (int j=0;j<3;++j) R[i][j]=values[i][j];
}
static imu_sample_t imu_sample(const north_calibration_t *cal, float R[3][3],
                               float gx, float gy, float gz) {
  float a[3] = {-R[2][1],R[2][0],R[2][2]};
  float rate[3] = {-gy,gx,gz};
  imu_sample_t s;
  for (int i=0;i<3;++i) {
    s.accel[i] = (int16_t)lroundf((a[i]/cal->accel_scale[i]+cal->accel_offset[i])/.000061f);
    s.gyro[i] = (int16_t)lroundf(rate[i]/.0175f);
  }
  return s;
}
static mag_sample_t mag_sample(const north_calibration_t *cal, float R[3][3]) {
  float b[3];
  for (int i=0;i<3;++i) b[i] = 30*R[0][i]-40*R[2][i];
  float sensor[3] = {b[1],b[0],-b[2]};
  /* Solve W*raw=corrected, independently of the production multiplication. */
  float aug[3][4];
  for (int i=0;i<3;++i) {
    for (int j=0;j<3;++j) aug[i][j] = cal->mag_matrix[i][j];
    aug[i][3] = sensor[i];
  }
  for (int k=0;k<3;++k) {
    float divisor = aug[k][k];
    for (int j=k;j<4;++j) aug[k][j] /= divisor;
    for (int i=0;i<3;++i) if (i!=k) {
      float factor = aug[i][k];
      for (int j=k;j<4;++j) aug[i][j] -= factor*aug[k][j];
    }
  }
  mag_sample_t sample = {.status=1};
  for (int i=0;i<3;++i) sample.xyz[i] = (int16_t)lroundf((aug[i][3]+cal->mag_offset[i])*37.5f);
  return sample;
}
static uint32_t reference(north_t *n, const north_calibration_t *cal, float yaw, uint32_t tick) {
  north_init(n, cal); north_reference(n);
  float R[3][3]; rotation(0,0,yaw,R);
  imu_sample_t s = imu_sample(cal,R,0,0,0);
  mag_sample_t m = mag_sample(cal,R);
  for (unsigned i=0;i<120;++i,tick+=20) { north_imu(n,&s,tick); north_mag(n,&m,tick); }
  return tick;
}

int north_test(void) {
  north_t n;
  /* Reference at a real tilt, then align north without levelling the board. */
  for (unsigned pose=0;pose<2;++pose) {
    float R[3][3]; rotation(pose ? .65f : .3f, -.15f, 1.0f, R);
    north_init(&n,&north_calibration); north_reference(&n);
    imu_sample_t s=imu_sample(&north_calibration,R,0,0,0);
    mag_sample_t m=mag_sample(&north_calibration,R);
    for (unsigned i=0;i<130;++i) { north_imu(&n,&s,i*20); north_mag(&n,&m,i*20); }
    if (pose) CHECK(n.imu.state==HEADING_CALIBRATING);
    else {
      CHECK(north_state(&n,2580)==NORTH_TRACKING);
      CHECK(fabsf(error(n.fused,-57.29578f))<.2f);
    }
  }
  north_init(&n, &identity);
  CHECK(north_state(&n,0)==NORTH_WAITING);
  CHECK(north_display(&n,0).red==0x525 && north_display(&n,0).green==0);
  /* Full pipeline with identity and the actual measured calibration. */
  for (unsigned calibration=0;calibration<2;++calibration) {
    const north_calibration_t *cal = calibration ? &north_calibration : &identity;
    uint32_t tick = reference(&n,cal,-1.57079633f,0);
    CHECK(north_state(&n,tick)==NORTH_TRACKING);
    CHECK(fabsf(error(n.fused,90))<.1f);
    CHECK(north_display(&n,tick).green==matrix_pointer(2).green);
    /* One minute stationary with +1 dps residual Z bias: gyro drifts,
       magnetic correction bounds the fused error near tau*rate = 5 degrees. */
    float R[3][3]; rotation(0,0,-1.57079633f,R);
    imu_sample_t s = imu_sample(cal,R,0,0,1);
    mag_sample_t m = mag_sample(cal,R);
    for (unsigned i=0;i<3000;++i,tick+=20) { north_imu(&n,&s,tick); north_mag(&n,&m,tick); }
    CHECK(fabsf(error(n.gyro,90))>55);
    CHECK(fabsf(error(n.fused,90))<5.2f);
    CHECK(n.mag_used);
    s = imu_sample(cal,R,0,0,0);
    for (unsigned i=0;i<1500;++i,tick+=20) { north_imu(&n,&s,tick); north_mag(&n,&m,tick); }
    CHECK(fabsf(error(n.fused,90))<.1f);
    /* Normal field magnitude but a 90-degree jump must be rejected. */
    float badR[3][3]; rotation(0,0,0,badR);
    mag_sample_t spike = mag_sample(cal,badR);
    north_mag(&n,&spike,tick);
    CHECK(n.compass_valid && !n.mag_used);
    CHECK(fabsf(error(n.fused,90))<.1f);
    /* Missing/overflow compass: bounded gyro fallback, then a visible X. */
    m.status = 3;
    for (unsigned i=0;i<40;++i,tick+=20) { north_imu(&n,&s,tick); north_mag(&n,&m,tick); }
    CHECK(north_state(&n,tick)==NORTH_COASTING);
    matrix_frame_t amber = north_display(&n,tick);
    CHECK((amber.red & amber.green)==amber.green && amber.green);
    for (unsigned i=0;i<80;++i,tick+=20) north_imu(&n,&s,tick);
    CHECK(north_state(&n,tick)==NORTH_UNRELIABLE);
    CHECK(north_display(&n,tick).red==0x525);
    m.status=1; north_mag(&n,&m,tick-20);
    CHECK(north_state(&n,tick)==NORTH_TRACKING);
    CHECK(north_state(&n,tick+101)==NORTH_UNRELIABLE);
  }
  /* Smooth tilted turn through north, testing CW/CCW and all transforms. */
  uint32_t tick = reference(&n,&north_calibration,0,0);
  for (unsigned i=0;i<2000;++i,tick+=20) {
    float t = (i+1)*.02f, r = .3f*sinf(t*.4f), p = .35f*sinf(t*.3f);
    float rd=.12f*cosf(t*.4f), pd=.105f*cosf(t*.3f), yd=.15f;
    float R[3][3]; rotation(r,p,yd*t,R);
    float gx=(rd-yd*sinf(p))*57.29578f;
    float gy=(pd*cosf(r)+yd*sinf(r)*cosf(p))*57.29578f;
    float gz=(-pd*sinf(r)+yd*cosf(r)*cosf(p))*57.29578f;
    imu_sample_t s=imu_sample(&north_calibration,R,gx,gy,gz);
    mag_sample_t m=mag_sample(&north_calibration,R);
    north_imu(&n,&s,tick); north_mag(&n,&m,tick);
    CHECK(north_state(&n,tick)==NORTH_TRACKING);
    CHECK(fabsf(error(n.compass,-yd*t*57.29578f))<1);
    CHECK(fabsf(error(n.fused,-yd*t*57.29578f))<1);
  }
  /* Calibration and timestamp wrap, plus no false north before alignment. */
  tick=reference(&n,&identity,0,UINT32_MAX-2100);
  CHECK(north_state(&n,tick)==NORTH_TRACKING);
  /* Very small residual rates must never produce 360000 mdeg telemetry. */
  tick=reference(&n,&identity,0,0);
  n.imu.bias_raw[2]=-.001f;
  float flat[3][3]; rotation(0,0,0,flat);
  imu_sample_t tiny=imu_sample(&identity,flat,0,0,0);
  north_imu(&n,&tiny,tick);
  CHECK(n.fused>=0 && n.fused<360 && n.gyro>=0 && n.gyro<360);
  north_reference(&n);
  CHECK(!n.aligned && north_state(&n,0)==NORTH_UNRELIABLE);
  float R[3][3]; rotation(0,0,0,R);
  imu_sample_t s=imu_sample(&identity,R,0,0,0);
  for (unsigned i=0;i<110;++i) north_imu(&n,&s,i*20);
  CHECK(!n.aligned && north_state(&n,2180)==NORTH_ALIGNING);
  s.gyro[0]=30000; north_imu(&n,&s,2200);
  CHECK(north_state(&n,2200)==NORTH_UNRELIABLE);
  return 0;
}
