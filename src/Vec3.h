// Vec3.h
#ifndef VEC3_H
#define VEC3_H
#include "_ug.h"

#include <tuple>
#include <armadillo>

namespace ugps
{
    class Vec3 
    {
    public:
        num_ug x, y, z;

        Vec3() : x(0), y(0), z(0) {}
        Vec3(num_ug x, num_ug y, num_ug z) : x(x), y(y), z(z) {}

        Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
        Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
        Vec3& operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
        Vec3& operator/=(const Vec3& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }

        friend bool operator==(const Vec3& L, const Vec3& R)
            { return tie(L.x, L.y, L.z) == tie(R.x, R.y, R.z); }
        friend bool operator!=(const Vec3& L, const Vec3& R) { return !(L==R); }

        Vec3 operator-() const { return Vec3(-x,-y,-z); }

        Vec3& operator*=(const num_ug& s) { x *= s; y *= s; z *= s; return *this; }
        Vec3& operator/=(const num_ug& s) { x /= s; y /= s; z /= s; return *this; }
        
        num_ug length() const;
        num_ug length2() const;

        arma::Col<num_ug> col() const;
        arma::Row<num_ug> row() const;

        friend Vec3 operator+(const Vec3& L, const Vec3& R) { return Vec3(L) += R; }
        friend Vec3 operator-(const Vec3& L, const Vec3& R) { return Vec3(L) -= R; }
        friend Vec3 operator*(const Vec3& L, const Vec3& R) { return Vec3(L) *= R; }
        friend Vec3 operator/(const Vec3& L, const Vec3& R) { return Vec3(L) /= R; }

        friend Vec3 operator*(const num_ug& s, const Vec3& v) { return Vec3(v) *= s; }
        friend Vec3 operator*(const Vec3& v, const num_ug& s) { return Vec3(v) *= s; }
        friend Vec3 operator/(const num_ug& s, const Vec3& v) { return Vec3(v) /= s; }
        friend Vec3 operator/(const Vec3& v, const num_ug& s) { return Vec3(v) /= s; }
    };

}

#endif
