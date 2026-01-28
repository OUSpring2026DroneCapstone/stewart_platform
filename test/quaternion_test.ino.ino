#include "Quaternion.h"

void printQuaternion(const char* label, const Quaternion& q) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print("w=");
  Serial.print(q.w, 4);
  Serial.print(", x=");
  Serial.print(q.x, 4);
  Serial.print(", y=");
  Serial.print(q.y, 4);
  Serial.print(", z=");
  Serial.println(q.z, 4);
}

void printVector(const char* label, float v[3]) {
  Serial.print(label);
  Serial.print(": [");
  Serial.print(v[0], 4);
  Serial.print(", ");
  Serial.print(v[1], 4);
  Serial.print(", ");
  Serial.print(v[2], 4);
  Serial.println("]");
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== Quaternion Library Test ===");

  Quaternion q1(1, 0, 1, 0);
  Quaternion q2(1, 0.5, 0.5, 0.75);

  // Test normalization
  Quaternion q_norm = q1.normalized();
  printQuaternion("Normalized q1", q_norm);

  // Test conjugate
  Quaternion q_conj = q1.conjugate();
  printQuaternion("Conjugate of q1", q_conj);

  // Test inverse
  Quaternion q_inv = q1.inverse();
  printQuaternion("Inverse of q1", q_inv);

  // Test multiplication
  Quaternion q_mul = q1 * q2;
  printQuaternion("q1 * q2", q_mul);

  // Test scalar multiplication
  Quaternion q_scaled = q1 * 2.0f;
  printQuaternion("q1 * 2.0", q_scaled);

  // Test addition
  Quaternion q_sum = q1 + q2;
  printQuaternion("q1 + q2", q_sum);

  // Test rotate
  float v[3] = {1, 0, 0};
  Quaternion rot_quat = azi_alt_to_rot(90, 0);  // 90 deg yaw
  Quaternion v_rotated = rot_quat.rotate(v);
  float result[3];
  v_rotated.toVector(result);
  printVector("Rotated [1, 0, 0] by 90° yaw", result);

  // Test constrainXY
  Quaternion q_cxy = q1.constrainXY();
  printQuaternion("q1.constrainXY()", q_cxy);

  // Test dot product
  float dot_val = q1.dot(q2);
  Serial.print("Dot(q1, q2): ");
  Serial.println(dot_val, 4);

  // Test slerp
  Quaternion q_slerp = slerp(q1.normalized(), q2.normalized(), 0.5f);
  printQuaternion("slerp(q1, q2, 0.5)", q_slerp);

  // Test azi_alt_to_rot
  Quaternion rot_test = azi_alt_to_rot(45, 45);
  printQuaternion("azi_alt_to_rot(45, 45)", rot_test);
}

void loop() {
  // Nothing
}
