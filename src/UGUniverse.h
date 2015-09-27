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

    UGImage<T> project(const T& theta, const T& phi, size_t offset = 0, size_t skip = 1) const 
    {
        UGImage<T> img;
        
#pragma omp parallel for
        for(auto star = stars.begin() + offset; star < stars.end(); star=star+skip) {
            T u = cos(phi)*cos(theta)*star->x 
                + sin(phi)*cos(theta)*star->y
                - sin(theta)*star->z;
            T v = - sin(phi)*star->x 
                + cos(phi)*star->y;

            UGImageStar<T> is;
            is.pos = UGVec2<T>(u,v);
            is.ref = *star;
#pragma omp critical(addStarToImageVector)
            {
                img.stars.push_back(is);
            }
        }
        return img;
    }

    UGImage<T> observe(const UGVec3<T>& position, const UGDirection<T>& direction, const T& roll) const
    {
        UGImage<T> img;
        
        //TODO: this

        return img;
    }


    void randomise(const size_t& n, const T& r)
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
