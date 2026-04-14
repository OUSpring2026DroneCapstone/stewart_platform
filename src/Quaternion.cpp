#include "Quaternion.h"

Quaternion::Quaternion(float w, float x, float y, float z)
    : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator*(const Quaternion& other) const {
    return Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w
    );
}

Quaternion Quaternion::operator*(float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

Quaternion Quaternion::operator+(const Quaternion& other) const {
    return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::inverse() const {
    float norm = w * w + x * x + y * y + z * z;
    return conjugate() * (1.0f / norm);
}

Quaternion Quaternion::rotate(const float v[3]) const {
    Quaternion vq(0, v[0], v[1], v[2]);
    Quaternion result = (*this) * vq * this->inverse();
    return result;
}

Quaternion Quaternion::constrainXY() const {
    float a = x * w - y * z;
    float b = y * w + x * z;
    float r = x * x + y * y;

    float w_new = sqrt((a * a + b * b) / r);
    float x_new = a / w_new;
    float y_new = b / w_new;
    float z_new = 0;

    return Quaternion(w_new, x_new, y_new, z_new);
}

Quaternion Quaternion::normalized() const {
    float norm = sqrt(w * w + x * x + y * y + z * z);
    return Quaternion(w / norm, x / norm, y / norm, z / norm);
}

float Quaternion::dot(const Quaternion& other) const {
    return w * other.w + x * other.x + y * other.y + z * other.z;
}

void Quaternion::toVector(float result[3]) const {
    result[0] = x;
    result[1] = y;
    result[2] = z;
}

Quaternion azi_alt_to_rot(float azi_deg, float alt_deg) {
    float azi = radians(azi_deg);  // convert degrees to radians
    float alt = radians(alt_deg);

    float w = cos(alt / 2);
    float x = sin(alt / 2) * -sin(azi);
    float y = sin(alt / 2) * cos(azi);
    float z = 0;

    return Quaternion(w, x, y, z);
}

Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    float cos_theta = q1.dot(q2);
    Quaternion end = q2;

    if (cos_theta < 0.0f) {
        end = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
        cos_theta = -cos_theta;
    }

    if (cos_theta > 0.9995f) {
        // If the quaternions are very close, use linear interpolation
        return (q1 * (1.0f - t) + end * t).normalized();
    }

    float theta = acos(cos_theta);
    float sin_theta = sin(theta);

    float scale1 = sin((1 - t) * theta) / sin_theta;
    float scale2 = sin(t * theta) / sin_theta;

    return (q1 * scale1 + end * scale2).normalized();
}