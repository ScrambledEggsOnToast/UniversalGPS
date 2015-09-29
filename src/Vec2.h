// Vec2.h
#ifndef VEC2_H
#define VEC2_H
#include "_ug.h"

#include <armadillo>

namespace ugps
{
    class Vec2 
    {
    public:
        num_ug x, y;

        Vec2() : x(0), y(0) {}
        Vec2(num_ug x, num_ug y) : x(x), y(y) {}

        Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
        Vec2& operator-=(const Vec2& v) { x -= v.x; y -= v.y; return *this; }
        Vec2& operator*=(const Vec2& v) { x *= v.x; y *= v.y; return *this; }
        Vec2& operator/=(const Vec2& v) { x /= v.x; y /= v.y; return *this; }

        friend bool operator==(const Vec2& L, const Vec2& R)
            { return L.x==R.x && L.y==R.y; }
        friend bool operator!=(const Vec2& L, const Vec2& R) { return !(L==R); }

        Vec2 operator-() const { return Vec2(-x,-y); }

        Vec2& operator*=(const num_ug& s) { x *= s; y *= s; return *this; }
        Vec2& operator/=(const num_ug& s) { x /= s; y /= s; return *this; }
        
        num_ug length() const;
        num_ug length2() const;

        arma::Col<num_ug> col() const;
        arma::Row<num_ug> row() const;

        friend Vec2 operator+(const Vec2& L, const Vec2& R) { return Vec2(L) += R; }
        friend Vec2 operator-(const Vec2& L, const Vec2& R) { return Vec2(L) -= R; }
        friend Vec2 operator*(const Vec2& L, const Vec2& R) { return Vec2(L) *= R; }
        friend Vec2 operator/(const Vec2& L, const Vec2& R) { return Vec2(L) /= R; }

        friend Vec2 operator*(const num_ug& s, const Vec2& v) { return Vec2(v) *= s; }
        friend Vec2 operator*(const Vec2& v, const num_ug& s) { return Vec2(v) *= s; }
        friend Vec2 operator/(const num_ug& s, const Vec2& v) { return Vec2(v) /= s; }
        friend Vec2 operator/(const Vec2& v, const num_ug& s) { return Vec2(v) /= s; }
    
        friend num_ug dot(const Vec2& L, const Vec2& R) { return L.x*R.x + L.y*R.y; }

        Vec2 rotate(const num_ug& angle) const;
        num_ug angle() const;
    };

}

#endif
