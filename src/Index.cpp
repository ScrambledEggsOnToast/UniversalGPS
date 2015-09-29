// Index.cpp
#include "Index.h"

#include <cstdlib>
#include <cmath>

namespace ugps
{
    Index::Index(
            const std::vector<Vec3>& stars,
            const std::vector<Direction>& directions
            ) : orientedIndex(Oriented<IndexEighth>())
    {
        std::cout << "Collecting " << stars.size() << " stars..." << std::endl;
        for(auto star : stars)
        {
            uniqueStars.push_back(make_unique<const Vec3>(star));
        }
        std::cout << quadErrorRadius() << std::endl;

        std::cout << "Projecting stars in " << directions.size() << " directions..." << std::endl;
        uniqueProjections.resize(directions.size());
#pragma omp parallel for
        for(int i = 0; i < directions.size(); i++)
        {
            uniqueProjections[i] = make_unique<const Projection>(directions[i], uniqueStars);
        }

        std::cout << "Calculating quads for " << uniqueProjections.size() << " projections..." << std::endl;
        for(auto proj = uniqueProjections.begin(); proj < uniqueProjections.end(); proj++)
        {
            (*proj)->calculateQuads(orientedIndex);
        }

        std::cout << "Building quad kdtrees..." << std::endl;
        buildTrees();

    }

    num_ug Index::quadErrorRadius() const
    {
        const num_ug D = 2;
        const num_ug N = static_cast<num_ug>(uniqueStars.size());

        const num_ug ba = meanDist(D,N,1);
        const num_ug ca = meanDist(D,N,2);
        const num_ug da = meanDist(D,N,3);

        return 2*(pow(ca,2)+pow(da,2))/pow(ba,2);
    }

    num_ug meanDist(const num_ug& D, const num_ug& N, const num_ug& k)
    {
        return pow(tgamma(1+D/2),1/D)*tgamma(k+1/D)*tgamma(N)/(sqrt(M_PI)*tgamma(k)*tgamma(N+1/D));
    }


}
