// UGLibrary.h
#ifndef UGLIBRARY_H
#define UGLIBRARY_H

#include <cstdlib>

#include <nanoflann.hpp>

#include "UGQuad.h"
#include "UGUniverse.h"

using namespace nanoflann;

template <class T>
class UGDirection
{
public:
    T theta, phi;

    UGDirection(): theta(0), phi(0) {};
    UGDirection(T theta, T phi): theta(theta), phi(phi) {};

    static std::vector<UGDirection<T> > fibonacci(size_t n) {
        std::vector<UGDirection<T> > dirs;
        dirs.resize(n);
        T offset = 2/(T)n;
        T increment = M_PI * (3 - sqrt(5));

        T th, ph;

        for(size_t i = 0; i != n; i++)
        {
            th = acos(((T)i * offset) - 1 + (offset / 2));
            ph = (T)i * increment;
            
            dirs[i] = UGDirection(th,ph);
        }
        return dirs;
    }
};

template <class T>
class UGLibrary
{
public:
    UGLibrary() {}

    UGLibrary(UGUniverse<T> &univ, std::vector<UGDirection <T> > &dirs) {
        UGQuadSet<T> qs;

        for(typename std::vector<UGDirection <T> >::iterator it = dirs.begin(); it != dirs.end(); ++it) {
            univ.project(it->theta, it->phi).calculateQuads(qs, it->theta, it->phi);

            std::cout << it->theta << std::endl;
        }

        quadKdtree = new typename UGQuadSet<T>::kdtree(7,qs,KDTreeSingleIndexAdaptorParams(10));
        quadKdtree->buildIndex();
    }

    

private:
    typename UGQuadSet<T>::kdtree* quadKdtree;
};


#endif
