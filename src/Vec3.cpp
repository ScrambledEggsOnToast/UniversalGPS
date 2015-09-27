// Vec3.cpp
#include "Vec3.h"

namespace ugps
{
    num_ug Vec3::length() const
    {
        return sqrt(length2());
    }

    num_ug Vec3::length2() const
    {
        return x*x + y*y + z*z;
    }


    arma::Col<num_ug> Vec3::col() const 
    {
        arma::Col<num_ug> c = {x,y,z};
        return c;
    }
	
    arma::Row<num_ug> Vec3::row() const 
    {
        arma::Row<num_ug> r = {x,y,z};
        return r;
    }
}
