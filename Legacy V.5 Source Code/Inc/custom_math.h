# pragma once
# define _USE_MATH_DEFINES

float Euler_Distance(const float[3], const float[3]);

void Vector3Lerp(float[3], const float[3], const float[3], float);

void Vector3Cross(float[3], const float[3], const float[3]);

void Vector3Normalize(float[3]);
