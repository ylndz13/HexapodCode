#include <Arduino.h>
#include "Vector3.h"
#include <math.h>

// Vector operations
float Vector3::xyProj() const {
    return sqrt(x * x + y * y);
}

Vector3 Vector3::distance(const Vector3& other) const {
    return Vector3(other.x - x, other.y - y, other.z - z);
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        (y * other.z - z * other.y),
        -(x * other.z - z * other.x),
        (x * other.y - y * other.x)
    );
}

float Vector3::dot(const Vector3& other) const {
    return (x * other.x + y * other.y + z * other.z);
}

// Operator overloads
Vector3 Vector3::operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
}

Vector3 Vector3::operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
}

Vector3 Vector3::operator/(int divisor) const {
    return (divisor == 0) ? Vector3(0, 0, 0) : Vector3(x / divisor, y / divisor, z / divisor);
}

void Vector3::print() const {
    Serial.print("Vector3 object: ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
}