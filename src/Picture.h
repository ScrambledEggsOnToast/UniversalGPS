// Picture.h
#ifndef PICTURE_H
#define PICTURE_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Vec2.h"
#include "Quad.h"
#include "IndexEighth.h"

namespace ugps
{
    template <class star_t, starToVec2Fn<star_t> vec2>
    class Picture
    {
    public:
        vector<star_t> stars;

        vector<star_t> nearestNeighbours(const Vec2& query, size_t n) const;
        void calculateQuads(Oriented<IndexEighth>& orientedIndex) const;
        void calculateQuads(vector<shared_ptr<const Quad<star_t, vec2> > >& quads) const;
        vector<shared_ptr<const Quad<star_t, vec2> > > calculateQuads() const;
        void buildIndex();

        // start nanoflann interface
        inline size_t kdtree_get_point_count() const;
        inline num_ug kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const;
        inline num_ug kdtree_get_pt(const size_t idx, int dim) const;
        template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const;
        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<num_ug, Picture>,
                Picture,
                7
                > kdtree;
        // end nanoflann interface
    protected:
        Picture();
    private:
        unique_ptr<kdtree> tree;
    };

    template <class star_t, starToVec2Fn<star_t> vec2>
    Picture<star_t,vec2>::Picture()
    {
        tree = make_unique<kdtree>(2,*this,nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    template <class star_t, starToVec2Fn<star_t> vec2>
    vector<star_t> Picture<star_t,vec2>::nearestNeighbours(const Vec2& query, size_t n) const
    {
        vector<size_t> ret_indexes(n);
        vector<num_ug> out_dists_sqr(n);
        nanoflann::KNNResultSet<num_ug> resultSet(n);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

        num_ug queryPoint[2];
        queryPoint[0] = query.x;
        queryPoint[1] = query.y;

        tree->findNeighbors(
                resultSet,
                &queryPoint[0],
                nanoflann::SearchParams());

        vector<star_t> returnStars;

        for(auto idx : ret_indexes)
        {
            returnStars.push_back(stars[idx]);
        }

        return returnStars;
    }
    
    template <class star_t, starToVec2Fn<star_t> vec2>
    vector<shared_ptr<const Quad<star_t, vec2> > > Picture<star_t,vec2>::calculateQuads() const
    {
        vector<shared_ptr<const Quad<star_t, vec2> > > quads;
        calculateQuads(quads);
        return quads;
    }

    template <class star_t, starToVec2Fn<star_t> vec2>
    void Picture<star_t,vec2>::calculateQuads(vector<shared_ptr<const Quad<star_t, vec2> > >& quads) const
    {
#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++)
        {
            vector<star_t> fourNearest = nearestNeighbours(vec2(*star),4);
            auto sharedQuad = make_shared<const Quad<star_t, vec2> >(fourNearest[0],fourNearest[1],fourNearest[2],fourNearest[3]);
#pragma omp critical(addQuad)
            {
                quads.push_back(sharedQuad);
            }
        }
    }

    template <class star_t, starToVec2Fn<star_t> vec2>
    void Picture<star_t,vec2>::calculateQuads(Oriented<IndexEighth>& orientedIndex) const
    {
#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++)
        {
            vector<star_t> fourNearest = nearestNeighbours(vec2(*star),4);
            auto sharedQuad = make_shared<const Quad<star_t, vec2> >(fourNearest[0],fourNearest[1],fourNearest[2],fourNearest[3]);
            Orientation o = sharedQuad->orientation;
            if(o == Orientation(false,false,false))
            {
#pragma omp critical(addQuad000)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(false,false,true))
            {
#pragma omp critical(addQuad001)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(false,true,false))
            {
#pragma omp critical(addQuad010)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(false,true,true))
            {
#pragma omp critical(addQuad011)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(true,false,false))
            {
#pragma omp critical(addQuad100)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(true,false,true))
            {
#pragma omp critical(addQuad101)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else if(o == Orientation(true,true,false))
            {
#pragma omp critical(addQuad110)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
            else 
            {
#pragma omp critical(addQuad111)
                {
                    orientedIndex[o].quads.push_back(sharedQuad);
                }
            }
        }
    }

    template <class star_t, starToVec2Fn<star_t> vec2>
    void Picture<star_t,vec2>::buildIndex()
    {
        tree->buildIndex();
    }

    // start nanoflann interface
    template <class star_t, starToVec2Fn<star_t> vec2>
    inline size_t Picture<star_t,vec2>::kdtree_get_point_count() const { return stars.size(); }

    template <class star_t, starToVec2Fn<star_t> vec2>
    inline num_ug Picture<star_t,vec2>::kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const
    { 
        const Vec2 starVec = vec2(stars[idx_p2]);
        const num_ug dx = p1[0] - starVec.x; 
        const num_ug dy = p1[1] - starVec.y; 
        return dx*dx + dy*dy;
    }

    template <class star_t, starToVec2Fn<star_t> vec2>
    inline num_ug Picture<star_t,vec2>::kdtree_get_pt(const size_t idx, int dim) const 
        { return dim == 0 ? vec2(stars[idx]).x : vec2(stars[idx]).y; }
        
    template <class star_t, starToVec2Fn<star_t> vec2>
    template <class BBOX>
	bool Picture<star_t,vec2>::kdtree_get_bbox(BBOX& bb) const 
    { 
        bb[0].low = vec2(stars[0]).x;
        bb[0].high = vec2(stars[0]).x;
        bb[1].low = vec2(stars[0]).y;
        bb[1].high = vec2(stars[0]).y;

        for (auto star : stars)
        {
            const Vec2 starVec = vec2(star);
            if(starVec.x < bb[0].low) bb[0].low = starVec.x;
            else if(starVec.x > bb[0].high) bb[0].high = starVec.x;
            if(starVec.y < bb[1].low) bb[1].low = starVec.y;
            else if(starVec.y > bb[1].high) bb[1].high = starVec.y;
        }
        return true; 
    }
    // end nanoflann interface
}

#endif
