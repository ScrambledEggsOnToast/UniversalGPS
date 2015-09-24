// UGQuad.h
#ifndef UGQUAD_H
#define UGQUAD_H

#include <nanoflann.hpp>

#include <cmath>

#include "UGVec2.h"

using namespace nanoflann;

template <class T>
class UGQuad {
public:
	T h1, h2, h3, h4, theta, phi;
    bool o1, o2, o3;
    UGVec2<T> a, b;
	
    UGQuad() : h1(0), h2(0), h3(0), h4(0), theta(0), phi(0), o1(false), o2(false), o3(false), a(UGVec2<T>()), b(UGVec2<T>()) {}
	UGQuad(T h1, T h2, T h3, T h4, T theta, T phi, bool o1, bool o2, bool o3, UGVec2<T> a, UGVec2<T> b) : h1(h1), h2(h2), h3(h3), h4(h4), theta(theta), phi(phi), o1(o1), o2(o2), o3(o3), a(a), b(b) {}
	UGQuad(const UGQuad& v) : h1(v.h1), h2(v.h2), h3(v.h3), h4(v.h4), theta(v.theta), phi(v.phi), o1(v.o1), o2(v.o2), o3(v.o3), a(v.a), b(v.b) {}

    UGQuad(UGVec2<T> a0, UGVec2<T> b0, UGVec2<T> c0, UGVec2<T> d0, T theta0, T phi0) {
        UGVec2<T> db = b0 - a0;
        UGVec2<T> dc = c0 - a0;
        UGVec2<T> dd = d0 - a0;
        UGVec2<T> yAxis(0.5 * (db.x - db.y), 0.5 * (db.x + db.y));
        UGVec2<T> xAxis = yAxis.ortho();
        T xy = xAxis.length();

        h1 = UGVec2<T>::dot(dc,xAxis) / xy;
        h2 = UGVec2<T>::dot(dc,yAxis) / xy;
        h3 = UGVec2<T>::dot(dd,xAxis) / xy;
        h4 = UGVec2<T>::dot(dd,yAxis) / xy;

        theta = theta0;
        phi = phi0;
        
        o1 = orient(db,dc);
        o2 = orient(dc,dd);
        o3 = orient(dd,db);

        a = a0;
        b = b0;
    }

	UGQuad& operator=(const UGQuad& v) {
		h1 = v.h1;
        h2 = v.h2;
        h3 = v.h3;
        h4 = v.h4;
        theta = v.theta;
        phi = v.phi;
        o1 = v.o1;
        o2 = v.o2;
        o3 = v.o3;
        a = v.a;
        b = v.b;
		return *this;
	}
	
	void set(T h1, T h2, T h3, T h4, bool o1, bool o2, bool o3, UGVec2<T> a, UGVec2<T> b) {
		this->h1 = h1;
		this->h2 = h2;
        this->h3 = h3;
        this->h4 = h4;
        this->theta = theta;
        this->phi = phi;
        this->o1 = o1;
        this->o2 = o2;
        this->o3 = o3;
        this->a = a;
        this->b = b;
	}
		
private:
    const bool orient(UGVec2<T>u, UGVec2<T>v) { return u.x*v.y - v.x*u.y > 0; }
};


template <class T>
class UGQuadSet
{
public:
    std::vector<UGQuad<T> > quads;

    UGQuadSet(): quads(std::vector<UGQuad<T> >()) {}
    UGQuadSet(std::vector<UGQuad<T> > quads): quads(quads) {}

    inline size_t kdtree_get_point_count() const { return quads.size(); }

    inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t) const 
    { 
        if((bool)p1[0] == quads[idx_p2].o1 && (bool)p1[1] == quads[idx_p2].o2 && (bool)p1[2] == quads[idx_p2].o3)
        {
            const T d1 = p1[3] - quads[idx_p2].h1; 
            const T d2 = p1[4] - quads[idx_p2].h2; 
            const T d3 = p1[5] - quads[idx_p2].h3; 
            const T d4 = p1[6] - quads[idx_p2].h4; 
            return d1*d1 + d2*d2 + d3*d3 + d4*d4;
        }
        else
        {
            return std::numeric_limits<T>::max();
        }
    }

    inline T kdtree_get_pt(const size_t idx, int dim) const 
    {
        if(dim == 0) return (T)quads[idx].o1;
        else if(dim == 1) return (T)quads[idx].o2;
        else if(dim == 2) return (T)quads[idx].o3;
        else if(dim == 3) return quads[idx].h1;
        else if(dim == 4) return quads[idx].h2;
        else if(dim == 5) return quads[idx].h3;
        else return quads[idx].h4;
    }
        
    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<T, UGQuadSet<T> >,
            UGQuadSet<T>,
            7
            > kdtree;
};

#endif
