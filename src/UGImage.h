// UGImage.h
#ifndef UGIMAGE_H
#define UGIMAGE_H

#include <nanoflann.hpp>

#include <cstdlib>

#include "UGVec2.h"
#include "UGQuad.h"

using namespace nanoflann;


template <class T>
class UGImage
{
public:
    std::vector<UGImageStar<T> > stars;

    UGImage(): stars(std::vector<UGImageStar<T> >()) {}
    UGImage(std::vector<UGImageStar<T> > stars): stars(stars) {}

    inline size_t kdtree_get_point_count() const { return stars.size(); }

    inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t) const 
    { 
        const T dx = p1[0] - stars[idx_p2].pos.x; 
        const T dy = p1[1] - stars[idx_p2].pos.y; 
        return dx*dx + dy*dy;
    }

    inline T kdtree_get_pt(const size_t idx, int dim) const 
        { return dim == 0 ? stars[idx].pos.x : stars[idx].pos.y; }
        
    template <class BBOX>
	bool kdtree_get_bbox(BBOX& bb) const 
    { 
        bb[0].low = stars[0].pos.x;
        bb[0].high = stars[0].pos.x;
        bb[1].low = stars[0].pos.y;
        bb[1].high = stars[0].pos.y;

        for (auto star = stars.begin() + 1; star < stars.end(); star++)
        {
            if(star->pos.x < bb[0].low) bb[0].low = star->pos.x;
            else if(star->pos.x > bb[0].high) bb[0].high = star->pos.x;
            if(star->pos.y < bb[1].low) bb[1].low = star->pos.y;
            else if(star->pos.y > bb[1].high) bb[1].high = star->pos.y;
        }
        return true; 
    }

    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<T, UGImage<T> >,
            UGImage<T>,
            2
            > kdtree;

    void calculateQuads(UGQuadSet<T> &quadset, const T& theta = 0, const T& phi = 0) const
    {

        kdtree index(2,*this,KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();

#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++) {
            std::vector<size_t> ret_indexes(4);
            std::vector<T> out_dists_sqr(4);

            nanoflann::KNNResultSet<T> resultSet(4);
            resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
            
            T queryPoint[2];

            queryPoint[0] = star->pos.x;
            queryPoint[1] = star->pos.y;

            index.findNeighbors(
                    resultSet,
                    &queryPoint[0],
                    nanoflann::SearchParams());
            
            UGImageStar<T> a = stars[ret_indexes[0]];
            UGImageStar<T> b = stars[ret_indexes[1]];
            UGImageStar<T> c = stars[ret_indexes[2]];
            UGImageStar<T> d = stars[ret_indexes[3]];

            UGQuad<T>quad = UGQuad<T>(a,b,c,d,theta,phi);

#pragma omp critical(addQuadToQuadset)
            {
                quadset.quads.push_back(quad);
            }
        }

    }

};


#endif
