// UGVec3.h
#ifndef UGVEC3_H
#define UGVEC3_H

#include <cmath>

template <class T>
class UGVec3 {
public:
	T x, y, z;
	
	UGVec3() :x(0), y(0), z(0) {}
	UGVec3(T x, T y, T z) : x(x), y(y), z(z) {}
	UGVec3(const UGVec3& v) : x(v.x), y(v.y), z(v.z) {}
	
	UGVec3& operator=(const UGVec3& v) {
		x = v.x;
		y = v.y;
        z = v.z;
		return *this;
	}
	
	UGVec3 operator+(const UGVec3& v) const {
		return UGVec3(x + v.x, y + v.y, z + v.z);
	}
	UGVec3 operator-(const UGVec3& v) const {
		return UGVec3(x - v.x, y - v.y, z - v.z);
	}
	
	UGVec3& operator+=(const UGVec3& v) {
		x += v.x;
		y += v.y;
        z += v.z;
		return *this;
	}
	UGVec3& operator-=(const UGVec3& v) {
		x -= v.x;
		y -= v.y;
        z -= v.z;
		return *this;
	}
	
	UGVec3 operator+(T s) const {
		return UGVec3(x + s, y + s, z + s);
	}
	UGVec3 operator-(T s) const {
		return UGVec3(x - s, y - s, z - s);
	}
	UGVec3 operator*(T s) const {
		return UGVec3(x * s, y * s, z * s);
	}
	UGVec3 operator/(T s) const {
		return UGVec3(x / s, y / s, z / s);
	}
	
	
	UGVec3& operator+=(T s) {
		x += s;
		y += s;
        z += s;
		return *this;
	}
	UGVec3& operator-=(T s) {
		x -= s;
		y -= s;
        z -= s;
		return *this;
	}
	UGVec3& operator*=(T s) {
		x *= s;
		y *= s;
        z *= s;
		return *this;
	}
	UGVec3& operator/=(T s) {
		x /= s;
		y /= s;
        z /= s;
		return *this;
	}
	
	void set(T x, T y, T z) {
		this->x = x;
		this->y = y;
        this->z = z;
	}
	
	UGVec3& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}
	
	T dist(UGVec3 v) const {
		UGVec3 d(v.x - x, v.y - y, v.z - z);
		return d.length();
	}
	T length() const {
		return std::sqrt(length2());
	}
    T length2() const {
		return x * x + y * y + z * z;
	}
	
	static T dot(UGVec3 v1, UGVec3 v2) {
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
	
};

typedef UGVec3<float> UGVec3f;
typedef UGVec3<double> UGVec3d;

#endif
