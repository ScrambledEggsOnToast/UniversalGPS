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

        quads = vector<shared_ptr<const Quad> >();
        std::cout << "Calculating quads..." << std::endl;
        for(auto proj : sharedProjections)
        {
            proj->calculateQuads(quads);
        }

        std::cout << "Building quad kdtree..." << std::endl;
        tree = make_unique<kdtree>(7,*this,KDTreeSingleIndexAdaptorParams(1000));
        tree->buildIndex();
    }

    Pose3 Index::search(const Projection& proj) const
    {
        vector<shared_ptr<const Quad> > projectionQuads;
        proj.calculateQuads(projectionQuads);

        vector<shared_ptr<const Pose2> > pose2s;
        for(auto q : projectionQuads)
        {
            vector<shared_ptr<const Quad> > qms = nearestNeighbours(*q,1);
            for(auto qm : qms)
            {
                pose2s.push_back(make_shared<const Pose2>(qm->measure(q)));
            }
        }

        num_ug filterRadius = sqrt(4*M_PI/(num_ug)sharedProjections.size());
        vector<shared_ptr<const Pose2> > pose2sfiltered;

        for(auto pose2 = pose2s.begin(); pose2 < pose2s.end(); pose2++)
        {
            for(auto pose2n = pose2s.begin(); pose2n < pose2s.end(); pose2n++)
            {
                if (pose2 == pose2n) continue;
                if (dist((*pose2)->dir,(*pose2n)->dir)<filterRadius)
                {
                    pose2sfiltered.push_back(*pose2);
                    break;
                }
            }
        }

        Vec2 aLoc;
        Vec2 aRollVec;
        Vec3 aDirVec;
        num_ug aScale;

        for(auto pose2 : pose2sfiltered)
        {
            aLoc += pose2->loc;
            aRollVec += Vec2(cos(pose2->roll), sin(pose2->roll));
            aDirVec += pose2->dir.unit();
            aScale += pose2->scale;
        }

        aLoc /= (num_ug)pose2sfiltered.size();
        num_ug aRoll = aRollVec.angle();
        Direction aDir = Direction(aDirVec);
        aScale /= (num_ug)pose2sfiltered.size();

        Pose2 aPose2(aLoc, aDir, aRoll, aScale);

        return aPose2.pose3();

    }

    vector<shared_ptr<const Quad> > Index::nearestNeighbours(const Quad& query, size_t n) const
    {
        vector<size_t> ret_indexes(n);
        vector<num_ug> out_dists_sqr(n);
        KNNResultSet<num_ug> resultSet(n);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        num_ug queryPoint[7];
        queryPoint[0] = query.dimension(0);
        queryPoint[1] = query.dimension(1);
        queryPoint[2] = query.dimension(2);
        queryPoint[3] = query.dimension(3);
        queryPoint[4] = query.dimension(4);
        queryPoint[5] = query.dimension(5);
        queryPoint[6] = query.dimension(6);

        tree->findNeighbors(
                resultSet,
                queryPoint,
                SearchParams());

        vector<shared_ptr<const Quad> > returnQuads;

        for(auto idx : ret_indexes)
        {
            returnQuads.push_back(quads[idx]);
        }

        return returnQuads;
    }

    vector<shared_ptr<const Quad> > Index::radiusSearch(const Quad& query, const num_ug& r) const
    {
        vector<pair<size_t,num_ug> > indices_dists;
		RadiusResultSet<num_ug,size_t> resultSet(r,indices_dists);

        num_ug queryPoint[7];
        queryPoint[0] = query.dimension(0);
        queryPoint[1] = query.dimension(1);
        queryPoint[2] = query.dimension(2);
        queryPoint[3] = query.dimension(3);
        queryPoint[4] = query.dimension(4);
        queryPoint[5] = query.dimension(5);
        queryPoint[6] = query.dimension(6);

		tree->findNeighbors(
                resultSet,
                queryPoint,
                SearchParams());
        
        vector<shared_ptr<const Quad> > returnQuads;

        for(auto idx_dist : indices_dists)
        {
            returnQuads.push_back(quads[idx_dist.first]);
        }
        
        return returnQuads;
    }

    // start nanoflann interface
    inline size_t Index::kdtree_get_point_count() const
    {
        return quads.size();
    }
    inline num_ug Index::kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const
    {
        return quads[idx_p2]->distance(p1);    
    }
    inline num_ug Index::kdtree_get_pt(const size_t idx, int dim) const
    {
        return quads[idx]->dimension(dim);
    }
    template <class BBOX> bool Index::kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
    // end nanoflann interface

}
