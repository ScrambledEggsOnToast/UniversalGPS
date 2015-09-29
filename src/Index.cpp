// Index.cpp
#include "Index.h"

#include <cstdlib>

using namespace std;

namespace ugps
{
    Index::Index(
            const std::vector<Vec3>& stars,
            const std::vector<Direction>& directions
            ) : orientedIndex(Oriented<IndexEighth>())
    {
        shared_ptr<const Vec3> sharedStar;

        std::cout << "Collecting stars..." << std::endl;
        for(auto star : stars)
        {
            sharedStar = make_shared<const Vec3>(star);
            sharedStars.push_back(sharedStar);
        }

        std::cout << "Projecting stars..." << std::endl;
#pragma omp parallel for
        for(auto direction = directions.begin(); direction < directions.end(); direction++)
        {
            auto sharedProjection = make_shared<const Projection>(*direction, sharedStars);
#pragma omp critical(addSharedProjection)
            {
                sharedProjections.push_back(sharedProjection);
            }
        }

        std::cout << "Calculating quads..." << std::endl;
        for(auto proj : sharedProjections)
        {
            proj->calculateQuads(orientedIndex);
        }

        std::cout << "Building quad kdtrees..." << std::endl;
        buildTrees();
    }

}
