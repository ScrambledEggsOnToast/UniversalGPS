// UGQuad.h
#ifndef UGQUAD_H
#define UGQUAD_H

#include <nanoflann.hpp>

#include <cmath>

#include <armadillo>

#include "UGVec2.h"
#include "UGVec3.h"
#include "UGImage.h"

using namespace nanoflann;
using namespace arma;

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

    UGVec3<T> unit() const{
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

    static T dist(const UGDirection<T>& a, const UGDirection<T>& b)
    {
        return acos(cos(a.theta)*cos(b.theta) + sin(a.theta)*sin(b.theta)*cos(a.phi-b.phi));
    }

    static void refine(std::vector<UGDirection<T> >& dirs)
    {
        //TODO

        const std::vector<UGDirection<T> > dirscp = dirs;

        for(size_t i = 0; i < dirscp.size(); i++)
        {
            
        }
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
    UGVec3<T> ba, ca, da;
	
    UGQuad(const UGImageStar<T>& a0, const UGImageStar<T>& b0, const UGImageStar<T>& c0, const UGImageStar<T>& d0,
            T theta0, T phi0) {
        UGVec2<T> db,dc,dd,xAxis,yAxis;

        db = b0.pos - a0.pos;
        dc = c0.pos - a0.pos;
        dd = d0.pos - a0.pos;
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

        a = a0.pos;
        b = b0.pos;
        
        ba = b0.ref - a0.ref;
        ca = c0.ref - a0.ref;
        da = d0.ref - a0.ref;
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
        
        //Start first order direction correction
        T th = qP.theta;
        T ph = qP.phi;
        T Dth, Dph;

        Mat<T> Bth(3,3);
        Bth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Bth(0,1) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0+1.0);
        Bth(0,2) = -cos(ph)+cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(1,0) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0-1.0);
        Bth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Bth(1,2) = -sin(ph)-cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,0) = -cos(ph)-cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(2,1) = -sin(ph)+cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,2) = sin(th*2.0);

        Mat<T> Bph(3,3);
        Bth(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Bth(0,1) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(0,2) = sin(th)*(cos(ph)-cos(th)*sin(ph));
        Bth(1,0) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Bth(1,2) = sin(th)*(sin(ph)+cos(ph)*cos(th));
        Bth(2,0) = -sin(th)*(cos(ph)+cos(th)*sin(ph));
        Bth(2,1) = -sin(th)*(sin(ph)-cos(ph)*cos(th));

        Mat<T> Cth(3,3);
        Cth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Cth(0,1) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(0,2) = cos(th*2.0)*cos(ph);
        Cth(1,0) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Cth(1,2) = cos(th*2.0)*sin(ph);
        Cth(2,0) = cos(th*2.0)*cos(ph);
        Cth(2,1) = cos(th*2.0)*sin(ph);
        Cth(2,2) = sin(th*2.0);

        Mat<T> Cph(3,3);
        Cph(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Cph(0,1) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(0,2) = -cos(th)*sin(ph)*sin(th);
        Cph(1,0) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Cph(1,2) = cos(ph)*cos(th)*sin(th);
        Cph(2,0) = -cos(th)*sin(ph)*sin(th);
        Cph(2,1) = cos(ph)*cos(th)*sin(th);

        T dh1,dh2,dh3,dh4;
        dh1 = qI.h1-qP.h1;
        dh2 = qI.h2-qP.h2;
        dh3 = qI.h3-qP.h3;
        dh4 = qI.h4-qP.h4;

        Col<T> dhs = {dh1/qP.h1,dh2/qP.h2,dh3/qP.h3,dh4/qP.h4};
        dhs *= dP*dP;

        Mat<T> M(4,2);

        M(0,0) = ((qP.ca.row()*Bth+qP.ba.row()*Cth)*qP.ba.col()).eval()[0];
        M(0,1) = ((qP.ca.row()*Bph+qP.ba.row()*Cph)*qP.ba.col()).eval()[0];
        M(1,0) = ((qP.ca.row()*Bth.t()+qP.ba.row()*Cth.t())*qP.ba.col()).eval()[0];
        M(1,1) = ((qP.ca.row()*Bph.t()+qP.ba.row()*Cph.t())*qP.ba.col()).eval()[0];

        M(2,0) = ((qP.da.row()*Bth+qP.ba.row()*Cth)*qP.ba.col()).eval()[0];
        M(2,1) = ((qP.da.row()*Bph+qP.ba.row()*Cph)*qP.ba.col()).eval()[0];
        M(3,0) = ((qP.da.row()*Bth.t()+qP.ba.row()*Cth.t())*qP.ba.col()).eval()[0];
        M(3,1) = ((qP.da.row()*Bph.t()+qP.ba.row()*Cph.t())*qP.ba.col()).eval()[0];

        Mat<T> mult = {{sqrt(2),0},{0,1/sin(th)}};

        M = M*mult;

        Mat<T> I = mult*pinv(M);

        Mat<T> res = (I*dhs).eval();

        Dth = res[0];
        Dph = res[1];

        UGDirection<T> dir(th+Dth, ph+Dph);
        //end first order direction correction

        ct = cos(dir.theta);
        cp = cos(dir.phi);
        st = sin(dir.theta);
        sp = sin(dir.phi);

        pos3 = UGVec3<T>(
                cp*ct*pos2.x - sp*pos2.y + cp*st*scale,
                sp*ct*pos2.x + cp*pos2.y + sp*st*scale,
                - st*pos2.x + ct*scale
                );
        


        return UGResult<T>(pos3,dir,rot, 0.0);
    }
		
private:
    inline bool orient(const UGVec2<T>& u, const UGVec2<T>& v) const
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
