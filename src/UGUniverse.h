// UGUniverse.h
#ifndef UGUNIVERSE_H
#define UGUNIVERSE_H

#include <cstdlib>

#include "UGVec3.h"
#include "UGVec2.h"
#include "UGImage.h"

template <class T>
class UGUniverse
{
public:
    std::vector<UGVec3<T> > stars;

    UGUniverse(): stars(std::vector<UGVec3<T> >()) {}
    UGUniverse(std::vector<UGVec3<T> > stars): stars(stars) {}

    UGImage<T> project(T theta, T phi) 
    {
        UGImage<T> img;
        img.stars.resize(stars.size());
        
        T u, v;

        for(typename std::vector<UGVec3<T> >::size_type i = 0; i != stars.size(); i++) {
            u = cos(phi)*cos(theta)*stars[i].x + sin(phi)*cos(theta)*stars[i].y - sin(theta)*stars[i].z;
            v = -sin(phi)*stars[i].x + cos(phi)*stars[i].y;
            img.stars[i] = UGVec2<T>(u,v);
        }
        return img;
    }

    void randomise(int n, T r)
    {
        stars.resize(n);
        for(typename std::vector<UGVec3<T> >::size_type i = 0; i != stars.size(); i++) {
            stars[i].x = r * rand() / T(RAND_MAX);
            stars[i].y = r * rand() / T(RAND_MAX);
            stars[i].z = r * rand() / T(RAND_MAX);
        }
    }   

};

#endif
