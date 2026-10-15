#include "attitude.h"
#include <math.h>

#define ATTITUDE_RAD_TO_DEG       57.2957795131f
#define ATTITUDE_GRAVITY          9.80665f
#define ATTITUDE_KP               2.0f
#define ATTITUDE_KI               0.05f
#define ATTITUDE_ACCEL_MIN_SQ     (0.70f * 0.70f * ATTITUDE_GRAVITY * ATTITUDE_GRAVITY)
#define ATTITUDE_ACCEL_MAX_SQ     (1.30f * 1.30f * ATTITUDE_GRAVITY * ATTITUDE_GRAVITY)

static void UpdateEuler(Attitude *attitude)
{
  float sin_pitch = 2.0f * (attitude->q0 * attitude->q2 -
                            attitude->q3 * attitude->q1);
  if (sin_pitch > 1.0f)
  {
    sin_pitch = 1.0f;
  }
  else if (sin_pitch < -1.0f)
  {
    sin_pitch = -1.0f;
  }
  attitude->roll_deg = atan2f(2.0f * (attitude->q0 * attitude->q1 +
                                      attitude->q2 * attitude->q3),
                              1.0f - 2.0f * (attitude->q1 * attitude->q1 +
                                             attitude->q2 * attitude->q2)) *
                       ATTITUDE_RAD_TO_DEG;
  attitude->pitch_deg = asinf(sin_pitch) * ATTITUDE_RAD_TO_DEG;
  attitude->yaw_deg = atan2f(2.0f * (attitude->q0 * attitude->q3 +
                                    attitude->q1 * attitude->q2),
                            1.0f - 2.0f * (attitude->q2 * attitude->q2 +
                                           attitude->q3 * attitude->q3)) *
                      ATTITUDE_RAD_TO_DEG;
}

void Attitude_Init(Attitude *attitude, const float accel_mps2[3])
{
  float roll = atan2f(accel_mps2[1], accel_mps2[2]);
  float pitch = atan2f(-accel_mps2[0],
                       sqrtf(accel_mps2[1] * accel_mps2[1] +
                             accel_mps2[2] * accel_mps2[2]));
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);

  attitude->q0 = cr * cp;
  attitude->q1 = sr * cp;
  attitude->q2 = cr * sp;
  attitude->q3 = -sr * sp;
  attitude->integral_x = 0.0f;
  attitude->integral_y = 0.0f;
  attitude->integral_z = 0.0f;
  UpdateEuler(attitude);
}

void Attitude_Update(Attitude *attitude, const float gyro_rps[3],
                     const float accel_mps2[3], float dt)
{
  float gx = gyro_rps[0];
  float gy = gyro_rps[1];
  float gz = gyro_rps[2];
  float accel_sq = accel_mps2[0] * accel_mps2[0] +
                   accel_mps2[1] * accel_mps2[1] +
                   accel_mps2[2] * accel_mps2[2];
  float q0 = attitude->q0;
  float q1 = attitude->q1;
  float q2 = attitude->q2;
  float q3 = attitude->q3;
  float reciprocal_norm;

  if ((accel_sq > ATTITUDE_ACCEL_MIN_SQ) && (accel_sq < ATTITUDE_ACCEL_MAX_SQ))
  {
    float ax;
    float ay;
    float az;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;

    reciprocal_norm = 1.0f / sqrtf(accel_sq);
    ax = accel_mps2[0] * reciprocal_norm;
    ay = accel_mps2[1] * reciprocal_norm;
    az = accel_mps2[2] * reciprocal_norm;
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    attitude->integral_x += ATTITUDE_KI * ex * dt;
    attitude->integral_y += ATTITUDE_KI * ey * dt;
    attitude->integral_z += ATTITUDE_KI * ez * dt;
    gx += ATTITUDE_KP * ex + attitude->integral_x;
    gy += ATTITUDE_KP * ey + attitude->integral_y;
    gz += ATTITUDE_KP * ez + attitude->integral_z;
  }

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  attitude->q0 += -q1 * gx - q2 * gy - q3 * gz;
  attitude->q1 +=  q0 * gx + q2 * gz - q3 * gy;
  attitude->q2 +=  q0 * gy - q1 * gz + q3 * gx;
  attitude->q3 +=  q0 * gz + q1 * gy - q2 * gx;
  reciprocal_norm = 1.0f / sqrtf(attitude->q0 * attitude->q0 +
                                  attitude->q1 * attitude->q1 +
                                  attitude->q2 * attitude->q2 +
                                  attitude->q3 * attitude->q3);
  attitude->q0 *= reciprocal_norm;
  attitude->q1 *= reciprocal_norm;
  attitude->q2 *= reciprocal_norm;
  attitude->q3 *= reciprocal_norm;
  UpdateEuler(attitude);
}