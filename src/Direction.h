// Direction.h
#ifndef DIRECTION_H
#define DIRECTION_H
#include "_ug.h"

#include <vector>

#include "Vec3.h"

namespace ugps
{
    class Direction
    {
    public:
        num_ug theta, phi;

        Direction() : theta(0), phi(0) {}
        Direction(num_ug theta, num_ug phi) : theta(theta), phi(phi) {}
        Direction(const Vec3& v);

        Vec3 unit() const;
    };

    num_ug dist(const Direction& a, const Direction& b);

    std::vector<Direction> fibonacciDirections(size_t n);
}

#endif
