
#pragma once

#include <cmath>
using namespace std;

namespace GM_Code
{

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Here are some small, but always useful, functions.
//............................................................

inline int clamp(int v, int vMin, int vMax)
{
    return (v < vMin) ? vMin : ((v > vMax) ? vMax : v);
}

inline float clamp(float v, float vMin, float vMax)
{
    return (v < vMin) ? vMin : ((v > vMax) ? vMax : v);
}

inline float lerp(float v0, float v1, float a)
{
    if (v0 > v1)
        return (1.0f - a) * (v0 - v1) + v1;
    return a * (v1 - v0) + v0;
}

inline float min(float a, float b) { return (a < b) ? a : b; }
inline int min(int a, int b) { return (a < b) ? a : b; }

inline float max(float a, float b) { return (a > b) ? a : b; }
inline int max(int a, int b) { return (a > b) ? a : b; }

inline float maxMag(float a, float b) { return (fabs(a) > fabs(b)) ? a : b; }

inline float ramp(float edge0, float edge1, float x)
{
    // Scale, and clamp x to 0..1 range
    x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x;
}

inline float smoothstep(float edge0, float edge1, float x)
{
    x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    // Evaluate polynomial
    return x * x * (3.0f - 2.0f * x);
}

inline float smootherstep(float edge0, float edge1, float x)
{
    x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
}

inline float deadband(float input, float band) {
	return (fabs(input) > fabs(band)) ? input : 0;
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Over time, we'll flesh-out the following types to include
// appropriate operations, like dot and cross products, etc.
//............................................................

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// simple 3D point type
//........................................
struct point_t
{
    float x, y, z;

    point_t() : x(0), y(0), z(0) {}
    point_t(float arg_x, float arg_y, float arg_z) : x(arg_x), y(arg_y), z(arg_z) {}

    void set(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void set(point_t &right)
    {
        this->x = right.x;
        this->y = right.y;
        this->z = right.z;
    }
};

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// simple homogeneous vector type
// can also represent a quaternion
//........................................
struct hvector_t
{
    float x, y, z, w;

    hvector_t() : x(0), y(0), z(0), w(1) {}
    hvector_t(float arg_x, float arg_y, float arg_z, float arg_w) : x(arg_x), y(arg_y), z(arg_z), w(arg_w) {}

    void set(float x, float y, float z, float w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    void set(hvector_t &right)
    {
        this->x = right.x;
        this->y = right.y;
        this->z = right.z;
        this->w = right.w;
    }
};

} // namespace GM_Code
