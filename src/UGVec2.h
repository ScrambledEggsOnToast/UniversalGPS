// UGVec2.h
#ifndef UGVEC2_H
#define UGVEC2_H

#include <cmath>

template <class T>
class UGVec2 {
public:
	T x, y;
	
	UGVec2() :x(0), y(0) {}
	UGVec2(T x, T y) : x(x), y(y) {}
	UGVec2(const UGVec2& v) : x(v.x), y(v.y) {}
    UGVec2(T theta) : x(cos(theta)), y(sin(theta)) {}
	
	UGVec2& operator=(const UGVec2& v) {
		x = v.x;
		y = v.y;
		return *this;
	}
	
	UGVec2 operator+(const UGVec2& v) const {
		return UGVec2(x + v.x, y + v.y);
	}
	UGVec2 operator-(const UGVec2& v) const {
		return UGVec2(x - v.x, y - v.y);
	}
	
	UGVec2& operator+=(const UGVec2& v) {
		x += v.x;
		y += v.y;
		return *this;
	}
	UGVec2& operator-=(const UGVec2& v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}
	
	UGVec2 operator+(T s) const {
		return UGVec2(x + s, y + s);
	}
	UGVec2 operator-(T s) const {
		return UGVec2(x - s, y - s);
	}
	UGVec2 operator*(T s) const {
		return UGVec2(x * s, y * s);
	}
	UGVec2 operator/(T s) const {
		return UGVec2(x / s, y / s);
	}
	
	
	UGVec2& operator+=(T s) {
		x += s;
		y += s;
		return *this;
	}
	UGVec2& operator-=(T s) {
		x -= s;
		y -= s;
		return *this;
	}
	UGVec2& operator*=(T s) {
		x *= s;
		y *= s;
		return *this;
	}
	UGVec2& operator/=(T s) {
		x /= s;
		y /= s;
		return *this;
	}
	
	void set(T x, T y) {
		this->x = x;
		this->y = y;
	}
	
	UGVec2 rotate(T theta) const {
		T c = cos(theta);
		T s = sin(theta);
		T tx = x * c - y * s;
		T ty = x * s + y * c;
		return UGVec2(tx,ty);
	}
	
	UGVec2& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}
	
	T dist(UGVec2 v) const {
		UGVec2 d(v.x - x, v.y - y);
		return d.length();
	}
	T length() const {
		return std::sqrt(length2());
	}
    T length2() const {
		return x * x + y * y;
	}
	void truncate(T length) {
		T angle = atan2f(y, x);
		x = length * cos(angle);
		y = length * sin(angle);
	}

    T angle() {
        return atan2f(y,x);
    }
	
	UGVec2 ortho() const {
		return UGVec2(y, -x);
	}
	
	static T dot(UGVec2 v1, UGVec2 v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}
	static T cross(UGVec2 v1, UGVec2 v2) {
		return (v1.x * v2.y) - (v1.y * v2.x);
	}
	
};

typedef UGVec2<float> UGVec2f;
typedef UGVec2<double> UGVec2d;

#endif
