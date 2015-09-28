// Index.h
#ifndef INDEX_H
#define INDEX_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Vec3.h"
#include "Direction.h"
#include "Picture.h"
#include "Projection.h"
#include "ProjectedStar.h"
#include "Quad.h"
#include "Pose.h"

using namespace nanoflann;

namespace ugps
{
    class Index
    {
    public:
        Index(const vector<Vec3>& universe3D, const vector<Direction>& directions);

        // start nanoflann interface
        inline size_t kdtree_get_point_count() const;
        inline num_ug kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const;
        inline num_ug kdtree_get_pt(const size_t idx, int dim) const;
        template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const;
        typedef KDTreeSingleIndexAdaptor<
                L2_Simple_Adaptor<num_ug, Index>,
                Index,
                7
                > kdtree;
        // end nanoflann interface

        template <class star_t, starToVec2Fn<star_t> vec2>
        Pose3 search(const Picture<star_t,vec2>& pic) const
        {
            return searchQuads(pic.calculateQuads());
        }

    private:
        vector<shared_ptr<const ProjectionQuad> > quads;
        unique_ptr<kdtree> tree;

        template<class star_t, starToVec2Fn<star_t> vec2>
        vector<shared_ptr<const ProjectionQuad> > nearestNeighbours(const Quad<star_t,vec2>& query, size_t n) const;

        template<class star_t, starToVec2Fn<star_t> vec2>
        vector<shared_ptr<const ProjectionQuad> > radiusSearch(const Quad<star_t,vec2>& query, const num_ug& r) const;
        
        template<class star_t, starToVec2Fn<star_t> vec2>
        Pose3 searchQuads(const vector<shared_ptr<const Quad<star_t, vec2> > >& pictureQuads) const;

        vector<shared_ptr<const Vec3> > sharedStars;
        vector<shared_ptr<const Projection> > sharedProjections;

    };
    
    template<class star_t, starToVec2Fn<star_t> vec2>
    vector<shared_ptr<const ProjectionQuad> > Index::nearestNeighbours(const Quad<star_t, vec2>& query, size_t n) const
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

        vector<shared_ptr<const ProjectionQuad> > returnQuads;

        for(auto idx : ret_indexes)
        {
            returnQuads.push_back(quads[idx]);
        }

        return returnQuads;
    }

    template<class star_t, starToVec2Fn<star_t> vec2>
    vector<shared_ptr<const ProjectionQuad> > Index::radiusSearch(const Quad<star_t,vec2>& query, const num_ug& r) const
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
        
        vector<shared_ptr<const ProjectionQuad> > returnQuads;

        for(auto idx_dist : indices_dists)
        {
            returnQuads.push_back(quads[idx_dist.first]);
        }
        
        return returnQuads;
    }

    template<class star_t, starToVec2Fn<star_t> vec2>
    Pose3 Index::searchQuads(const vector<shared_ptr<const Quad<star_t,vec2> > >& pictureQuads) const
    {

        vector<shared_ptr<const Pose2> > pose2s;
        for(auto q : pictureQuads)
        {
            vector<shared_ptr<const ProjectionQuad> > qms = nearestNeighbours(*q,1);
            for(auto qm : qms)
            {
                pose2s.push_back(make_shared<const Pose2>(measure(qm,q)));
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
                    //std::cout << (*pose2)->dir.theta << std::endl;
                    break;
                }
            }
        }

        //std::cout << pose2sfiltered.size() << "/" << pose2s.size() << std::endl;

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

    inline num_ug Index::kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const
    {
        return quads[idx_p2]->distance(p1);    
    }
    inline num_ug Index::kdtree_get_pt(const size_t idx, int dim) const
    {
        return quads[idx]->dimension(dim);
    }
}

#endif
