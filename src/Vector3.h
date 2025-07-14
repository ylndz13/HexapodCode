#ifndef VECTOR3_H
#define VECTOR3_H

class Vector3 {
    public: 
        float x, y, z;

    // Constructors
    Vector3(); 
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Vector operations
    float magnitude() const;
    Vector3 distance(const Vector3& other) const;

    Vector3 cross(const Vector3& other) const;
    float dot(const Vector3& other) const;

    // Operator overloads
    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
};

#endif // VECTOR3_H
