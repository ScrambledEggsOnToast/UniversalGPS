// Direction.cpp
#include "Direction.h"

namespace ugps
{
    Direction::Direction(const Vec3& v) : 
        theta(acos(v.z/v.length())), phi(atan2(v.y,v.x)) {}
    
    Vec3 Direction::unit() const
    {
        num_ug ct, cp, st, sp;
        ct = cos(theta);
        cp = cos(phi);
        st = sin(theta);
        sp = sin(phi);
        return Vec3(cp*st,sp*st,ct);        
    }

    num_ug dist(const Direction& a, const Direction& b)
    {
        return acos(cos(a.theta)*cos(b.theta) + sin(a.theta)*sin(b.theta)*cos(a.phi-b.phi));
    }

    std::vector<Direction> fibonacciDirections(size_t n)
    {
        std::vector<Direction> dirs;
        dirs.resize(n);
        num_ug offset = 2/(num_ug)n;
        num_ug increment = M_PI * (3 - sqrt(5));

        num_ug th, ph;
        
        for(size_t i = 0; i < n; i++)
        {
            th = acos(((num_ug)i * offset) - 1 + (offset / 2));
            ph = (num_ug)i * increment;
            
            dirs[i] = Direction(th,ph);
        }
        return dirs;
    }
}

