#ifndef QUATERNION_H
#define QUATERNION_H

#include <Arduino.h>
#include <Printable.h>

class Quaternion {
public:
    float w, x, y, z;

    Quaternion(float w = 1, float x = 0, float y = 0, float z = 0);

    Quaternion operator*(const Quaternion& other) const;
    Quaternion operator*(float scalar) const;
    Quaternion operator+(const Quaternion& other) const;

    Quaternion conjugate() const;
    Quaternion inverse() const;
    Quaternion normalized() const;

    float dot(const Quaternion& other) const;
    Quaternion slerp(const Quaternion& other, float t) const;

    Quaternion rotate(const float v[3]) const;
    Quaternion constrainXY() const;
    
    void toVector(float result[3]) const;
};

// Converts azimuth and altitude (in degrees) to a Quaternion rotation
Quaternion azi_alt_to_rot(float azi_deg, float alt_deg);
Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);

#endif
