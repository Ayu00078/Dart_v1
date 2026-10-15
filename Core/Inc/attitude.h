#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_x;
  float integral_y;
  float integral_z;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
} Attitude;

void Attitude_Init(Attitude *attitude, const float accel_mps2[3]);
void Attitude_Update(Attitude *attitude, const float gyro_rps[3],
                     const float accel_mps2[3], float dt);

#ifdef __cplusplus
}
#endif

#endif