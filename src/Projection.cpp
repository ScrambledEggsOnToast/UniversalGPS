// Projection.cpp
#include "Projection.h"

namespace ugps
{
    Projection::Projection(const Direction& dir, const vector<shared_ptr<const Vec3> >& stars3D)
    {
        direction = make_shared<Direction>(dir);
        for(auto star3D : stars3D)
        {
            const ProjectedStar proj(star3D, direction);
            auto sharedProj = make_shared<const ProjectedStar>(proj);
            stars.push_back(sharedProj);
        }

        buildIndex();
    }

}
