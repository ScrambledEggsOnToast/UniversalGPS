// Projection.cpp
#include "Projection.h"

namespace ugps
{
    Vec2 projectionVec2(const ProjectedStar* const& proj) { return proj->pos; }

    Projection::Projection(const Direction& dir, const vector<unique_ptr<const Vec3> >& stars3D)
    {
        direction = make_unique<Direction>(dir);
        for(auto& star3D : stars3D)
        {
            ProjectedStar proj(star3D.get(), direction.get());
            auto uniqueStar = make_unique<ProjectedStar>(proj);
            stars.push_back(uniqueStar.get());
            uniqueStars.push_back(move(uniqueStar));
        }

        buildIndex();
    }

}
