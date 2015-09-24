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
        
#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++) {
            T u = cos(phi)*cos(theta)*star->x 
                + sin(phi)*cos(theta)*star->y
                - sin(theta)*star->z;
            T v = - sin(phi)*star->x 
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
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<T> dist (-r,r);
        for(auto star = stars.begin(); star < stars.end(); star++) {
            star->x = dist(mt);
            star->y = dist(mt);
            star->z = dist(mt);
        }

    }   

};

#endif
