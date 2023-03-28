/**
 * @file quatfrom6a.c
 * @author York Fu (york-fu@outlook.com)
 * @brief Quaternion from 6-axis
 * @version 0.1
 * @date 2023-03-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "quatfrom6a.h"

int16_t quat_update(quaternion *q, vector3f w, vector3f a, float dt,
                         float kp, float ki, vector3f *ei)
{
  if ((w.x == 0.0f) && (w.y == 0.0f) && (w.z == 0.0f))
  {
    return -1;
  }

  float norm;
  float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
  float half_dt = dt * 0.5;
  vector3f v, e;

  norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  a.x = a.x / norm;
  a.y = a.y / norm;
  a.z = a.z / norm;

  // 重力的方向
  v.x = 2 * (q1 * q3 - q0 * q2);
  v.y = 2 * (q0 * q1 + q2 * q3);
  v.z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // 计算误差，测量值和参考值的叉积
  e.x = (a.y * v.z - a.z * v.y);
  e.y = (a.z * v.x - a.x * v.z);
  e.z = (a.x * v.y - a.y * v.x);

  // 误差积分
  ei->x += e.x * ki;
  ei->y += e.y * ki;
  ei->z += e.z * ki;

  // 更新w
  w.x = w.x + kp * e.x + ei->x;
  w.y = w.y + kp * e.y + ei->y;
  w.z = w.z + kp * e.z + ei->z;

  // 更新q
  q0 = q0 + (-q1 * w.x - q2 * w.y - q3 * w.z) * half_dt;
  q1 = q1 + (q0 * w.x + q2 * w.z - q3 * w.y) * half_dt;
  q2 = q2 + (q0 * w.y - q1 * w.z + q3 * w.x) * half_dt;
  q3 = q3 + (q0 * w.z + q1 * w.y - q2 * w.x) * half_dt;

  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  q->w = q0;
  q->x = q1;
  q->y = q2;
  q->z = q3;

  return 0;
}

vector3f quat2euler(const quaternion q)
{
  vector3f rpy;
  rpy.x = atan2(2 * q.y * q.z + 2 * q.w * q.x, -2 * q.x * q.x - 2 * q.y * q.y + 1);
  rpy.y = asin(-2 * q.x * q.z + 2 * q.w * q.y);
  rpy.z = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  return rpy;
}

quaternion euler2quat(const vector3f rpy)
{
  double c1 = cos(rpy.x / 2.0);
  double s1 = sin(rpy.x / 2.0);
  double c2 = cos(rpy.y / 2.0);
  double s2 = sin(rpy.y / 2.0);
  double c3 = cos(rpy.z / 2.0);
  double s3 = sin(rpy.z / 2.0);

  quaternion q;
  q.w = c1 * c2 * c3 + s1 * s2 * s3;
  q.x = s1 * c2 * c3 - c1 * s2 * s3;
  q.y = c1 * s2 * c3 + s1 * c2 * s3;
  q.z = c1 * c2 * s3 - s1 * s2 * c3;
  return q;
}
