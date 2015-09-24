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
	
	UGVec2& operator=(const UGVec2& v) {
		x = v.x;
		y = v.y;
		return *this;
	}
	
	UGVec2 operator+(UGVec2& v) {
		return UGVec2(x + v.x, y + v.y);
	}
	UGVec2 operator-(UGVec2& v) {
		return UGVec2(x - v.x, y - v.y);
	}
	
	UGVec2& operator+=(UGVec2& v) {
		x += v.x;
		y += v.y;
		return *this;
	}
	UGVec2& operator-=(UGVec2& v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}
	
	UGVec2 operator+(double s) {
		return UGVec2(x + s, y + s);
	}
	UGVec2 operator-(double s) {
		return UGVec2(x - s, y - s);
	}
	UGVec2 operator*(double s) {
		return UGVec2(x * s, y * s);
	}
	UGVec2 operator/(double s) {
		return UGVec2(x / s, y / s);
	}
	
	
	UGVec2& operator+=(double s) {
		x += s;
		y += s;
		return *this;
	}
	UGVec2& operator-=(double s) {
		x -= s;
		y -= s;
		return *this;
	}
	UGVec2& operator*=(double s) {
		x *= s;
		y *= s;
		return *this;
	}
	UGVec2& operator/=(double s) {
		x /= s;
		y /= s;
		return *this;
	}
	
	void set(T x, T y) {
		this->x = x;
		this->y = y;
	}
	
	void rotate(double deg) {
		double theta = deg / 180.0 * M_PI;
		double c = cos(theta);
		double s = sin(theta);
		double tx = x * c - y * s;
		double ty = x * s + y * c;
		x = tx;
		y = ty;
	}
	
	UGVec2& normalize() {
		if (length() == 0) return *this;
		*this *= (1.0 / length());
		return *this;
	}
	
	float dist(UGVec2 v) const {
		UGVec2 d(v.x - x, v.y - y);
		return d.length();
	}
	float length() const {
		return std::sqrt(length2());
	}
    float length2() const {
		return x * x + y * y;
	}
	void truncate(double length) {
		double angle = atan2f(y, x);
		x = length * cos(angle);
		y = length * sin(angle);
	}
	
	UGVec2 ortho() const {
		return UGVec2(y, -x);
	}
	
	static float dot(UGVec2 v1, UGVec2 v2) {
		return v1.x * v2.x + v1.y * v2.y;
	}
	static float cross(UGVec2 v1, UGVec2 v2) {
		return (v1.x * v2.y) - (v1.y * v2.x);
	}
	
};

typedef UGVec2<float> UGVec2f;
typedef UGVec2<double> UGVec2d;

#endif
