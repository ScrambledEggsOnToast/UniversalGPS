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
    
    void Vec3::rotateX(num_ug angle)
    {
        *this = Vec3(
                x,
                cos(angle)*y-sin(angle)*z,
                sin(angle)*y+cos(angle)*z);
    }

    void Vec3::rotateY(num_ug angle)
    {
        *this = Vec3(
                cos(angle)*x+sin(angle)*z,
                y,
                -sin(angle)*x+cos(angle)*z);
    }

    void Vec3::rotateZ(num_ug angle)
    {
        *this = Vec3(
                cos(angle)*x-sin(angle)*y,
                sin(angle)*x+cos(angle)*y,
                z);
    }
    
    ostream& operator<<(ostream& ost, const Vec3& v) 
    {
        ost << v.x << "," << v.y << "," << v.z;
        return ost;
    }

    istream& operator>>(istream& is, Vec3& v)
    {
        std::string line;
        char comma1, comma2;
        std::getline(is, line);
        std::istringstream iss(line);

        iss >> v.x >> comma1 >> v.y >> comma2 >> v.z;
        return is;
    }

}
