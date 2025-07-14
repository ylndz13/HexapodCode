#include "Vector3.h"
#include <math.h>

// Constructors
Vector3::Vector3() : x(0), y(0), z(0) {}
Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

// Vector operations
float Vector3::magnitude() const {
    return sqrt(x * x + y * y + z * z);
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