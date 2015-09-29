// Index.h
#ifndef INDEX_H
#define INDEX_H
#include "_ug.h"

#include <nanoflann.hpp>
#include <tuple>

#include "Vec3.h"
#include "Direction.h"
#include "Picture.h"
#include "Projection.h"
#include "ProjectedStar.h"
#include "Quad.h"
#include "Pose.h"
#include "IndexEighth.h"
#include "Orientation.h"

using namespace nanoflann;

namespace ugps
{
    class Index
    {
    public:
        Index(const vector<Vec3>& universe3D, const vector<Direction>& directions);

        template <class star_t, starToVec2Fn<star_t> vec2>
        Pose3 search(const Picture<star_t,vec2>& pic) const
        {
            return searchQuads(pic.calculateQuads());
        }

    private:
        
        num_ug quadErrorRadius() const;

        void buildTrees() const
        {
#pragma omp parallel for
            for(int i = 0; i < 8; i++)
            {
                orientedIndex[i].tree->buildIndex();
            }
        }

        Oriented<IndexEighth> orientedIndex;

        template<class star_t, starToVec2Fn<star_t> vec2>
        vector<const ProjectionQuad*> nearestNeighbours(const Quad<star_t,vec2>& query, size_t n) const;

        template<class star_t, starToVec2Fn<star_t> vec2>
        vector<const ProjectionQuad*> radiusSearch(const Quad<star_t,vec2>& query, const num_ug& r) const;
        
        template<class star_t, starToVec2Fn<star_t> vec2>
        Pose3 searchQuads(const vector<unique_ptr<const Quad<star_t, vec2> > >& pictureQuads) const;

        vector<unique_ptr<const Vec3> > uniqueStars;
        vector<unique_ptr<const Projection> > uniqueProjections;

    };
    
    template<class star_t, starToVec2Fn<star_t> vec2>
    vector<const ProjectionQuad*> Index::nearestNeighbours(const Quad<star_t, vec2>& query, size_t n) const
    {
        vector<size_t> ret_indexes(n);
        vector<num_ug> out_dists_sqr(n);
        KNNResultSet<num_ug> resultSet(n);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        num_ug queryPoint[4];
        queryPoint[0] = query.q1;
        queryPoint[1] = query.q2;
        queryPoint[2] = query.q3;
        queryPoint[3] = query.q4;

        orientedIndex[query.orientation].tree->findNeighbors(resultSet,queryPoint,SearchParams());

        vector<const ProjectionQuad*> returnQuads;

        for(auto idx : ret_indexes)
        {
            returnQuads.push_back(orientedIndex[query.orientation].quads[idx].get());
        }

        return returnQuads;
    }

    template<class star_t, starToVec2Fn<star_t> vec2>
    vector<const ProjectionQuad*> Index::radiusSearch(const Quad<star_t,vec2>& query, const num_ug& r) const
    {
        vector<pair<size_t,num_ug> > indices_dists;
		RadiusResultSet<num_ug,size_t> resultSet(r,indices_dists);

        num_ug queryPoint[4];
        queryPoint[0] = query.q1;
        queryPoint[1] = query.q2;
        queryPoint[2] = query.q3;
        queryPoint[3] = query.q4;

        orientedIndex[query.orientation].tree->findNeighbors(resultSet,queryPoint,SearchParams());
        
        vector<const ProjectionQuad*> returnQuads;

        for(auto idx_dist : indices_dists)
        {
            returnQuads.push_back(orientedIndex[query.orientation].quads[idx_dist.first].get());
        }
        
        return returnQuads;
    }

    template<class star_t, starToVec2Fn<star_t> vec2>
    Pose3 Index::searchQuads(const vector<unique_ptr<const Quad<star_t,vec2> > >& pictureQuads) const
    {

        vector<unique_ptr<const Pose2> > pose2s;
#pragma omp parallel for
        for(auto q = pictureQuads.begin(); q < pictureQuads.end(); q++)
        {
            vector<const ProjectionQuad*> qms = nearestNeighbours(**q,1);//radiusSearch(**q,0.05);
            for(auto qm : qms)
            {
#pragma omp critical(addPose2)
                {
                    pose2s.push_back(make_unique<const Pose2>(measure(*qm,**q)));
                }
            }
        }

        num_ug filterRadius = sqrt(4*M_PI/(num_ug)uniqueProjections.size());
        vector<const Pose2*> pose2sfiltered;

        for(auto pose2 = pose2s.begin(); pose2 < pose2s.end(); pose2++)
        {
            for(auto pose2n = pose2s.begin(); pose2n < pose2s.end(); pose2n++)
            {
                if (pose2 == pose2n) continue;
                if (dist((*pose2)->dir,(*pose2n)->dir)<filterRadius)
                {
                    pose2sfiltered.push_back(pose2->get());
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

    num_ug meanDist(const num_ug& D, const num_ug& N, const num_ug& k);

}

#endif
