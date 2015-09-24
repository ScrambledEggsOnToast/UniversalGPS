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
    UGUniverse(size_t n, T r): stars(std::vector<UGVec3<T> >()) 
        { randomise(n,r); }

    UGImage<T> project(T theta, T phi) 
    {
        UGImage<T> img;
        
        T u, v;
        
#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++) {
            u = cos(phi)*cos(theta)*star->x 
                + sin(phi)*cos(theta)*star->y
                - sin(theta)*star->z;
            v = - sin(phi)*star->x 
                + cos(phi)*star->y;
#pragma omp critical(addStarToImageVector)
            {
                img.stars.push_back(UGVec2<T>(u,v));
            }
        }
        return img;
    }

    void randomise(size_t n, T r)
    {
        stars.resize(n);
        for(UGVec3<T> star : stars) {
            star.x = r * rand() / T(RAND_MAX);
            star.y = r * rand() / T(RAND_MAX);
            star.z = r * rand() / T(RAND_MAX);
        }
    }   

};

#endif
