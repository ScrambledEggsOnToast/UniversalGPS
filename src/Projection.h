// Projection.h
#ifndef PROJECTION_H
#define PROJECTION_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Vec3.h"
#include "ProjectedStar.h"
#include "Quad.h"

#include "Picture.h"

namespace ugps
{

    Vec2 projectionVec2(const ProjectedStar* const& proj);

    class Projection : public Picture<ProjectedStar*, projectionQuadVec2>
    {
    public:
        Projection(const Direction& dir, const vector<unique_ptr<const Vec3> >& stars3D);
        unique_ptr<Direction> direction;
    private:
        vector<unique_ptr<ProjectedStar> > uniqueStars;
    };
}

#endif
