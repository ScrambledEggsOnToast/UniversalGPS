// Quad.cpp
#include "Quad.h"

namespace ugps
{
    Quad::Quad(
            const shared_ptr<const ProjectedStar>& a,
            const shared_ptr<const ProjectedStar>& b,
            const shared_ptr<const ProjectedStar>& c,
            const shared_ptr<const ProjectedStar>& d
            ) : a(a), b(b), c(c), d(d)
    {
        Vec2 db, dc, dd, xAxis, yAxis;

        db = b->pos - a->pos;
        dc = c->pos - a->pos;
        dd = d->pos - a->pos;

        yAxis = Vec2(0.5 * (db.x - db.y), 0.5 * (db.x + db.y));
        xAxis = Vec2(yAxis.y, -yAxis.x);

        num_ug xy = xAxis.length();

        q1 = dot(dc,xAxis) / xy;
        q2 = dot(dc,yAxis) / xy;
        q3 = dot(dd,xAxis) / xy;
        q4 = dot(dd,yAxis) / xy;

        o1 = orient(db,dc);
        o2 = orient(dc,dd);
        o3 = orient(dd,db);
    }

    Pose2 Quad::measure(const shared_ptr<const Quad>& qI) const
    {
        num_ug dI, dP, aI, aP, scale, rot;
        Vec2 pos2;
        Vec3 pos3;

        dI = (qI->a->pos - qI->b->pos).length();
        dP = (a->pos - b->pos).length();
        scale = dP / dI;

        aI = (qI->a->pos - qI->b->pos).angle();
        aP = (a->pos - b->pos).angle();
        rot = aP - aI;

        pos2 = a->pos - qI->a->pos.rotate(rot)/scale;

        num_ug th = a->dir->theta;
        num_ug ph = a->dir->phi;

        //Start first order direction correction
        Mat<num_ug> Bth(3,3);
        Bth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Bth(0,1) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0+1.0);
        Bth(0,2) = -cos(ph)+cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(1,0) = -sin(th)*(cos(ph)*cos(th)*sin(ph)*2.0-1.0);
        Bth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Bth(1,2) = -sin(ph)-cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,0) = -cos(ph)-cos(th)*sin(ph)+cos(ph)*pow(cos(th),2.0)*2.0;
        Bth(2,1) = -sin(ph)+cos(ph)*cos(th)+pow(cos(th),2.0)*sin(ph)*2.0;
        Bth(2,2) = sin(th*2.0);

        Mat<num_ug> Bph(3,3);
        Bth(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Bth(0,1) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(0,2) = sin(th)*(cos(ph)-cos(th)*sin(ph));
        Bth(1,0) = pow(sin(th),2.0)*(pow(sin(ph),2.0)*2.0-1.0);
        Bth(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Bth(1,2) = sin(th)*(sin(ph)+cos(ph)*cos(th));
        Bth(2,0) = -sin(th)*(cos(ph)+cos(th)*sin(ph));
        Bth(2,1) = -sin(th)*(sin(ph)-cos(ph)*cos(th));

        Mat<num_ug> Cth(3,3);
        Cth(0,0) = pow(cos(ph),2.0)*cos(th)*sin(th)*-2.0;
        Cth(0,1) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(0,2) = cos(th*2.0)*cos(ph);
        Cth(1,0) = sin(ph*2.0)*sin(th*2.0)*(-1.0/2.0);
        Cth(1,1) = cos(th)*pow(sin(ph),2.0)*sin(th)*-2.0;
        Cth(1,2) = cos(th*2.0)*sin(ph);
        Cth(2,0) = cos(th*2.0)*cos(ph);
        Cth(2,1) = cos(th*2.0)*sin(ph);
        Cth(2,2) = sin(th*2.0);

        Mat<num_ug> Cph(3,3);
        Cph(0,0) = cos(ph)*sin(ph)*pow(sin(th),2.0)*2.0;
        Cph(0,1) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(0,2) = -cos(th)*sin(ph)*sin(th);
        Cph(1,0) = cos(ph*2.0)*(cos(th*2.0)*(1.0/2.0)-1.0/2.0);
        Cph(1,1) = cos(ph)*sin(ph)*pow(sin(th),2.0)*-2.0;
        Cph(1,2) = cos(ph)*cos(th)*sin(th);
        Cph(2,0) = -cos(th)*sin(ph)*sin(th);
        Cph(2,1) = cos(ph)*cos(th)*sin(th);

        num_ug dq1,dq2,dq3,dq4;
        dq1 = qI->q1-q1;
        dq2 = qI->q2-q2;
        dq3 = qI->q3-q3;
        dq4 = qI->q4-q4;

        Col<num_ug> dqs = {dq1/q1,dq2/q2,dq3/q3,dq4/q4};
        dqs *= dP*dP;

        Mat<num_ug> M(4,2);

        Vec3 ba = *(b->pos3D) - *(a->pos3D);
        Vec3 ca = *(c->pos3D) - *(a->pos3D);
        Vec3 da = *(d->pos3D) - *(a->pos3D);

        M(0,0) = ((ca.row()*Bth+ba.row()*Cth)*ba.col()).eval()[0];
        M(0,1) = ((ca.row()*Bph+ba.row()*Cph)*ba.col()).eval()[0];
        M(1,0) = ((ca.row()*Bth.t()+ba.row()*Cth.t())*ba.col()).eval()[0];
        M(1,1) = ((ca.row()*Bph.t()+ba.row()*Cph.t())*ba.col()).eval()[0];

        M(2,0) = ((da.row()*Bth+ba.row()*Cth)*ba.col()).eval()[0];
        M(2,1) = ((da.row()*Bph+ba.row()*Cph)*ba.col()).eval()[0];
        M(3,0) = ((da.row()*Bth.t()+ba.row()*Cth.t())*ba.col()).eval()[0];
        M(3,1) = ((da.row()*Bph.t()+ba.row()*Cph.t())*ba.col()).eval()[0];

        Mat<num_ug> mult = {{sqrt(2),0},{0,1/sin(th)}};

        M = M*mult;

        Mat<num_ug> I = mult*pinv(M);

        Mat<num_ug> res = (I*dqs).eval();

        Direction dir(th+res[0], ph+res[1]);
        
        
        //end first order direction correction
        
        return Pose2(pos2,dir,scale,rot);
    }

    num_ug Quad::dimension(int dim) const
    {
        switch(dim)
        {
            case 0:
                return (num_ug)o1;
                break;
            case 1:
                return (num_ug)o2;
                break;
            case 2:
                return (num_ug)o3;
                break;
            case 3:
                return q1;
                break;
            case 4:
                return q2;
                break;
            case 5:
                return q3;
                break;
            default:
                return q4;
                break;
        }
    }

    num_ug Quad::distance(const num_ug* p1) const
    {
        if((bool)p1[0] == o1 
                && (bool)p1[1] == o2 
                && (bool)p1[2] == o3)
        {
            const num_ug d1 = p1[3] - q1; 
            const num_ug d2 = p1[4] - q2; 
            const num_ug d3 = p1[5] - q3; 
            const num_ug d4 = p1[6] - q4; 
            return d1*d1 + d2*d2 + d3*d3 + d4*d4;
        }
        else
        {
            return std::numeric_limits<num_ug>::max();
        }
    }        
}
