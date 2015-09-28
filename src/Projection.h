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

    class Projection : public Picture<shared_ptr<const ProjectedStar>, projectionVec2>
    {
    public:
        Projection(const Direction& dir, const vector<shared_ptr<const Vec3> >& stars3D);
        shared_ptr<Direction> direction;
    };
}

#endif
