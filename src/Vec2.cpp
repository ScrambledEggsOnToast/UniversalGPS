// Vec2.cpp
#include "Vec2.h"

#include <armadillo>

namespace ugps
{
    num_ug Vec2::length() const
    {
        return sqrt(length2());
    }

    num_ug Vec2::length2() const
    {
        return x*x + y*y;
    }


    arma::Col<num_ug> Vec2::col() const 
    {
        arma::Col<num_ug> c = {x,y};
        return c;
    }
	
    arma::Row<num_ug> Vec2::row() const 
    {
        arma::Row<num_ug> r = {x,y};
        return r;
    }
    
    Vec2 Vec2::rotate(const num_ug& angle) const
    {
        return Vec2(
                cos(angle)*x - sin(angle)*y,
                sin(angle)*y + cos(angle)*x
                );
    }

    num_ug Vec2::angle() const
    {
        return atan2(y,x);
    }
}
