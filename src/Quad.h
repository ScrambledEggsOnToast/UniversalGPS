// Quad.h
#ifndef QUAD_H
#define QUAD_H
#include "_ug.h"

#include <armadillo>

#include "Vec2.h"
#include "Pose.h"
#include "ProjectedStar.h"

using namespace arma;

namespace ugps
{
    class Quad
    {
    public:
        Quad(
                const shared_ptr<const ProjectedStar>& a,
                const shared_ptr<const ProjectedStar>& b,
                const shared_ptr<const ProjectedStar>& c,
                const shared_ptr<const ProjectedStar>& d
                );

        num_ug dimension(int dim) const;
        num_ug distance(const num_ug* p1) const;

        Pose2 measure(const shared_ptr<const Quad>& q) const;

        num_ug q1, q2, q3, q4;
        bool o1, o2, o3, o4;
        shared_ptr<const ProjectedStar> a, b, c, d;

    private:
        inline bool orient(const Vec2& u, const Vec2& v) const
            { return u.x*v.y - v.x*u.y > 0; }    
    };
}

#endif
