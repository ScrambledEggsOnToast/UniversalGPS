// UGQuad.h
#ifndef UGQUAD_H
#define UGQUAD_H

#include <nanoflann.hpp>

#include <cmath>

#include "UGVec2.h"
#include "UGVec3.h"

using namespace nanoflann;

template <class T>
class UGDirection
{
public:
    T theta, phi;

    UGDirection(): theta(0), phi(0) {};
    UGDirection(T theta, T phi): theta(theta), phi(phi) {};
    UGDirection(const UGVec3<T>& v): theta(acos(v.z/v.length())), phi(atan2(v.y,v.x)) {};

    static std::vector<UGDirection<T> > fibonacci(size_t n)
    {
        std::vector<UGDirection<T> > dirs;
        dirs.resize(n);
        T offset = 2/(T)n;
        T increment = M_PI * (3 - sqrt(5));

        T th, ph;
        
        for(size_t i = 0; i < n; i++)
        {
            th = acos(((T)i * offset) - 1 + (offset / 2));
            ph = (T)i * increment;
            
            dirs[i] = UGDirection(th,ph);
        }
        return dirs;
    }

    UGVec3<T> unit(){
        T ct,cp,st,sp;
        ct = cos(theta);
        cp = cos(phi);
        st = sin(theta);
        sp = sin(phi);
        return UGVec3<T>(
                cp*st,
                sp*st,
                ct
                );
    }

};

template <class T>
class UGResult
{
public:
    UGVec3<T> loc;
    UGDirection<T> dir;
    T roll, err;

    UGResult(): loc(UGVec3<T>()), dir(UGDirection<T>()), roll(0), err(0) {};
    UGResult(UGVec3<T> loc, UGDirection<T> dir, T roll, T err):
        loc(loc), dir(dir), roll(roll), err(err) {};
};

template <class T>
class UGQuad {
public:
	T h1, h2, h3, h4, theta, phi;
    bool o1, o2, o3;
    UGVec2<T> a, b;
	
    UGQuad() : h1(0), h2(0), h3(0), h4(0), 
        theta(0), phi(0), o1(false), o2(false), o3(false), 
        a(UGVec2<T>()), b(UGVec2<T>()) {}

	UGQuad(T h1, T h2, T h3, T h4, 
            T theta, T phi, bool o1, bool o2, bool o3, 
            UGVec2<T> a, UGVec2<T> b) : 
        h1(h1), h2(h2), h3(h3), h4(h4), 
        theta(theta), phi(phi), 
        o1(o1), o2(o2), o3(o3), a(a), b(b) {}

	UGQuad(const UGQuad& v) : 
        h1(v.h1), h2(v.h2), h3(v.h3), h4(v.h4), 
        theta(v.theta), phi(v.phi), o1(v.o1), o2(v.o2), o3(v.o3), 
        a(v.a), b(v.b) {}

    UGQuad(UGVec2<T> a0, UGVec2<T> b0, UGVec2<T> c0, UGVec2<T> d0,
            T theta0, T phi0) {
        UGVec2<T> db,dc,dd,xAxis,yAxis;

        db = b0 - a0;
        dc = c0 - a0;
        dd = d0 - a0;
        yAxis.set(0.5 * (db.x - db.y), 0.5 * (db.x + db.y));
        xAxis = yAxis.ortho();
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
	
	void set(T h1, T h2, T h3, T h4, bool o1, bool o2, bool o3, 
            UGVec2<T> a, UGVec2<T> b) {
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

    static const UGResult<T> quadResult(const UGQuad<T>& qI, const UGQuad<T>& qP)
    {
        T dI, dP, aI, aP, scale, rot, ct, cp, st, sp;
        UGVec2<T> pos2;
        UGVec3<T> pos3;

        dI = (qI.a - qI.b).length();
        dP = (qP.a - qP.b).length();
        scale = dP / dI;
        
        aI = (qI.a - qI.b).angle();
        aP = (qP.a - qP.b).angle();
        rot = aP - aI;

        pos2 = qP.a - qI.a.rotate(rot)/scale;

        ct = cos(qP.theta);
        cp = cos(qP.phi);
        st = sin(qP.theta);
        sp = sin(qP.phi);

        pos3 = UGVec3<T>(
                cp*ct*pos2.x - sp*pos2.y + cp*st*scale,
                sp*ct*pos2.x + cp*pos2.y + sp*st*scale,
                - st*pos2.x + ct*scale
                );

        return UGResult<T>(pos3,UGDirection<T>(qP.theta,qP.phi),rot, 0.0);
    }
		
private:
    inline const bool orient(UGVec2<T>u, UGVec2<T>v) 
        { return u.x*v.y - v.x*u.y > 0; }
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
        if((bool)p1[0] == quads[idx_p2].o1 
                && (bool)p1[1] == quads[idx_p2].o2 
                && (bool)p1[2] == quads[idx_p2].o3)
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
