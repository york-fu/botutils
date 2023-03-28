/**
 * @file quatfrom6a.h
 * @author York Fu (york-fu@outlook.com)
 * @brief Quaternion from 6-axis
 * @version 0.1
 * @date 2023-03-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _quatfrom6a_H_
#define _quatfrom6a_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <math.h>

typedef struct
{
  float x, y, z;
} vector3f;

typedef struct
{
  float w, x, y, z;
} quaternion;

int16_t quat_update(quaternion *q, vector3f w, vector3f a, float dt,
                         float kp, float ki, vector3f *ei);
vector3f quat2euler(const quaternion q);
quaternion euler2quat(const vector3f rpy);

#ifdef __cplusplus
}
#endif

#endif
