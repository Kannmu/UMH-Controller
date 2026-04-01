# define _USE_MATH_DEFINES
# include "custom_math.h"
# include <math.h>

float Euler_Distance(const float From[3], const float To[3])
{
    float TempDistance = 0.0;
    for (int i = 0; i < 3; i++)
    {
        float diff = From[i] - To[i];
        TempDistance += diff * diff; // Avoid calling pow() for exponent 2, use direct multiplication
    }
    return (float)sqrt(TempDistance);
}

void Vector3Lerp(float Result[3], const float From[3], const float To[3], float t)
{
    for (int i = 0; i < 3; i++)
    {
        Result[i] = From[i] + (To[i] - From[i]) * t;
    }
}

void Vector3Cross(float Result[3], const float A[3], const float B[3])
{
    Result[0] = A[1] * B[2] - A[2] * B[1];
    Result[1] = A[2] * B[0] - A[0] * B[2];
    Result[2] = A[0] * B[1] - A[1] * B[0];
}

void Vector3Normalize(float A[3])
{
    float length = (float)sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
    if (length > 0.0f)
    {
        A[0] /= length;
        A[1] /= length;
        A[2] /= length;
    }
}


