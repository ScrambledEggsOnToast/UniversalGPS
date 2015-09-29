// Quad.h
#ifndef QUAD_H
#define QUAD_H
#include "_ug.h"

#include <armadillo>

#include "Vec2.h"
#include "Pose.h"
#include "ProjectedStar.h"
#include "Orientation.h"

namespace ugps
{
    template <class star_t>
    using starToVec2Fn = Vec2(const star_t&);

    template<class star_t, starToVec2Fn<star_t> vec2>
    class Quad
    {
    public:
        Quad(
                const star_t& a,
                const star_t& b,
                const star_t& c,
                const star_t& d
                );

        num_ug dimension(int dim) const;
        num_ug distance(const num_ug* p1) const;

        Pose2 measure(const shared_ptr<const Quad>& q) const;

        num_ug q1, q2, q3, q4;
        Orientation orientation;
        star_t a, b, c, d;

    private:
        inline bool orient(const Vec2& u, const Vec2& v) const
            { return u.x*v.y - v.x*u.y > 0; }    
    };

    template<class star_t, starToVec2Fn<star_t> vec2>
    Quad<star_t,vec2>::Quad(
            const star_t& a,
            const star_t& b,
            const star_t& c,
            const star_t& d
            ) : a(a), b(b), c(c), d(d), orientation(Orientation())
    {
        Vec2 db, dc, dd, xAxis, yAxis;

        db = vec2(b) - vec2(a);
        dc = vec2(c) - vec2(a);
        dd = vec2(d) - vec2(a);

        yAxis = Vec2(0.5 * (db.x - db.y), 0.5 * (db.x + db.y));
        xAxis = Vec2(yAxis.y, -yAxis.x);

        num_ug xy = xAxis.length2();

        q1 = dot(dc,xAxis) / xy;
        q2 = dot(dc,yAxis) / xy;
        q3 = dot(dd,xAxis) / xy;
        q4 = dot(dd,yAxis) / xy;

        orientation.o1 = orient(db,dc);
        orientation.o2 = orient(dc,dd);
        orientation.o3 = orient(dd,db);
    }    

    Vec2 projectionVec2(const shared_ptr<const ProjectedStar>& proj);

    typedef Quad<shared_ptr<const ProjectedStar>, projectionVec2> ProjectionQuad;

    template<class star_t, starToVec2Fn<star_t> vec2>
    Pose2 measure(const shared_ptr<const ProjectionQuad>& qP, const shared_ptr<const Quad<star_t,vec2> >& qI)
    {
        num_ug dI, dP, aI, aP, scale, rot;
        Vec2 pos2;
        Vec3 pos3;

        dI = (vec2(qI->a) - vec2(qI->b)).length();
        dP = (qP->a->pos - qP->b->pos).length();
        scale = dP / dI;

        aI = (vec2(qI->a) - vec2(qI->b)).angle();
        aP = (qP->a->pos - qP->b->pos).angle();
        rot = aP - aI;

        pos2 = qP->a->pos - vec2(qI->a).rotate(rot)/scale;

        num_ug th = qP->a->dir->theta;
        num_ug ph = qP->a->dir->phi;

        //Start first order direction correction
        arma::Mat<num_ug> Bth(3,3);
        Bth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Bth(0,1) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0+1.0);
        Bth(0,2) = -cos(ph)+cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(1,0) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0-1.0);
        Bth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Bth(1,2) = -sin(ph)-cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,0) = -cos(ph)-cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(2,1) = -sin(ph)+cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,2) = sin(th*2.0);

        arma::Mat<num_ug> Bph(3,3);
        Bth(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Bth(0,1) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(0,2) = sin(th)*(cos(ph)-cos(th)*sin(ph));
        Bth(1,0) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Bth(1,2) = sin(th)*(sin(ph)+cos(ph)*cos(th));
        Bth(2,0) = -sin(th)*(cos(ph)+cos(th)*sin(ph));
        Bth(2,1) = -sin(th)*(sin(ph)-cos(ph)*cos(th));

        arma::Mat<num_ug> Cth(3,3);
        Cth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Cth(0,1) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(0,2) = cos(th*2.0)*cos(ph);
        Cth(1,0) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Cth(1,2) = cos(th*2.0)*sin(ph);
        Cth(2,0) = cos(th*2.0)*cos(ph);
        Cth(2,1) = cos(th*2.0)*sin(ph);
        Cth(2,2) = sin(th*2.0);

        arma::Mat<num_ug> Cph(3,3);
        Cph(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Cph(0,1) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(0,2) = -cos(th)*sin(ph)*sin(th);
        Cph(1,0) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Cph(1,2) = cos(ph)*cos(th)*sin(th);
        Cph(2,0) = -cos(th)*sin(ph)*sin(th);
        Cph(2,1) = cos(ph)*cos(th)*sin(th);

        num_ug dq1,dq2,dq3,dq4;
        dq1 = qI->q1-qP->q1;
        dq2 = qI->q2-qP->q2;
        dq3 = qI->q3-qP->q3;
        dq4 = qI->q4-qP->q4;

        arma::Col<num_ug> dqs = {dq1/qP->q1,dq2/qP->q2,dq3/qP->q3,dq4/qP->q4};
        dqs *= dP*dP;

        arma::Mat<num_ug> M(4,2);

        Vec3 ba = *(qP->b->pos3D) - *(qP->a->pos3D);
        Vec3 ca = *(qP->c->pos3D) - *(qP->a->pos3D);
        Vec3 da = *(qP->d->pos3D) - *(qP->a->pos3D);

        M(0,0) = ((ca.row()*Bth+ba.row()*Cth)*ba.col()).eval()[0];
        M(0,1) = ((ca.row()*Bph+ba.row()*Cph)*ba.col()).eval()[0];
        M(1,0) = ((ca.row()*Bth.t()+ba.row()*Cth.t())*ba.col()).eval()[0];
        M(1,1) = ((ca.row()*Bph.t()+ba.row()*Cph.t())*ba.col()).eval()[0];

        M(2,0) = ((da.row()*Bth+ba.row()*Cth)*ba.col()).eval()[0];
        M(2,1) = ((da.row()*Bph+ba.row()*Cph)*ba.col()).eval()[0];
        M(3,0) = ((da.row()*Bth.t()+ba.row()*Cth.t())*ba.col()).eval()[0];
        M(3,1) = ((da.row()*Bph.t()+ba.row()*Cph.t())*ba.col()).eval()[0];

        arma::Mat<num_ug> mult = {{sqrt(2),0},{0,1/sin(th)}};

        M = M*mult;

        arma::Mat<num_ug> I = mult*arma::pinv(M);

        arma::Mat<num_ug> res = (I*dqs).eval();

        Direction dir(th+res[0], ph+res[1]);
        
        
        //end first order direction correction
        
        return Pose2(pos2,dir,rot,scale);
    }        

}

#endif
