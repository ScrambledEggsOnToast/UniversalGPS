// Index.cpp
#include "Index.h"

#include <cstdlib>

using namespace std;

namespace ugps
{
    Index::Index(
            const std::vector<Vec3>& stars,
            const std::vector<Direction>& directions
            )
    {
        shared_ptr<const Vec3> sharedStar;
        shared_ptr<const Projection> sharedProjection;

        std::cout << "Collecting stars..." << std::endl;
        for(auto star : stars)
        {
            sharedStar = make_shared<const Vec3>(star);
            sharedStars.push_back(sharedStar);
        }

        std::cout << "Projecting stars..." << std::endl;
        for(auto direction : directions)
        {
            sharedProjection = make_shared<const Projection>(direction, sharedStars);
            sharedProjections.push_back(sharedProjection);
        }

        quads = vector<shared_ptr<const ProjectionQuad> >();
        std::cout << "Calculating quads..." << std::endl;
        for(auto proj : sharedProjections)
        {
            proj->calculateQuads(quads);
        }

        std::cout << "Building quad kdtree..." << std::endl;
        tree = make_unique<kdtree>(7,*this,KDTreeSingleIndexAdaptorParams(1000));
        tree->buildIndex();
    }

    // start nanoflann interface
    inline size_t Index::kdtree_get_point_count() const
    {
        return quads.size();
    }
    template <class BBOX> bool Index::kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
    // end nanoflann interface
}
