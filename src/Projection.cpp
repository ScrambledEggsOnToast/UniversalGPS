// Projection.cpp
#include "Projection.h"

namespace ugps
{
    Projection::Projection(Direction dir, vector<shared_ptr<const Vec3> > stars)
    {
        direction = make_shared<Direction>(dir);
        for(auto star : stars)
        {
            const ProjectedStar proj(star, direction);
            auto sharedProj = make_shared<const ProjectedStar>(proj);
            projectedStars.push_back(sharedProj);
        }

        tree = make_unique<kdtree>(2,*this,KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();
    }

    vector<shared_ptr<const ProjectedStar> > Projection::nearestNeighbours(const Vec2& query, size_t n) const
    {
        vector<size_t> ret_indexes(n);
        vector<num_ug> out_dists_sqr(n);
        KNNResultSet<num_ug> resultSet(n);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        num_ug queryPoint[2];
        queryPoint[0] = query.x;
        queryPoint[1] = query.y;

        tree->findNeighbors(
                resultSet,
                &queryPoint[0],
                SearchParams());

        vector<shared_ptr<const ProjectedStar> > returnStars;

        for(auto idx : ret_indexes)
        {
            returnStars.push_back(projectedStars[idx]);
        }

        return returnStars;
    }

    void Projection::calculateQuads(vector<shared_ptr<const Quad> >& quads) const
    {
        vector<shared_ptr<const ProjectedStar> > fourNearest;

        for(auto projectedStar : projectedStars)
        {
            fourNearest = nearestNeighbours(projectedStar->pos,4);
            auto sharedQuad = make_shared<const Quad>(fourNearest[0],fourNearest[1],fourNearest[2],fourNearest[3]);
            quads.push_back(sharedQuad);
        }
    }

    // start nanoflann interface
    inline size_t Projection::kdtree_get_point_count() const { return projectedStars.size(); }

    inline num_ug Projection::kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const
    { 
        const num_ug dx = p1[0] - projectedStars[idx_p2]->pos.x; 
        const num_ug dy = p1[1] - projectedStars[idx_p2]->pos.y; 
        return dx*dx + dy*dy;
    }

    inline num_ug Projection::kdtree_get_pt(const size_t idx, int dim) const 
        { return dim == 0 ? projectedStars[idx]->pos.x : projectedStars[idx]->pos.y; }
        
    template <class BBOX>
	bool Projection::kdtree_get_bbox(BBOX& bb) const 
    { 
        bb[0].low = projectedStars[0]->pos.x;
        bb[0].high = projectedStars[0]->pos.x;
        bb[1].low = projectedStars[0]->pos.y;
        bb[1].high = projectedStars[0]->pos.y;

        for (auto star : projectedStars)
        {
            if(star->pos.x < bb[0].low) bb[0].low = star->pos.x;
            else if(star->pos.x > bb[0].high) bb[0].high = star->pos.x;
            if(star->pos.y < bb[1].low) bb[1].low = star->pos.y;
            else if(star->pos.y > bb[1].high) bb[1].high = star->pos.y;
        }
        return true; 
    }
    // end nanoflann interface

        
}
