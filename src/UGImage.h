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
    std::vector<UGVec2<T> > stars;

    UGImage(): stars(std::vector<UGVec2<T> >()) {}
    UGImage(std::vector<UGVec2<T> > stars): stars(stars) {}

    inline size_t kdtree_get_point_count() const { return stars.size(); }

    inline T kdtree_distance(const T* p1, const size_t idx_p2, size_t) const 
    { 
        const T dx = p1[0] - stars[idx_p2].x; 
        const T dy = p1[1] - stars[idx_p2].y; 
        return dx*dx + dy*dy;
    }

    inline T kdtree_get_pt(const size_t idx, int dim) const 
        { return dim == 0 ? stars[idx].x : stars[idx].y; }
        
    template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<T, UGImage<T> >,
            UGImage<T>,
            2
            > kdtree;

    void calculateQuads(UGQuadSet<T> &quadset, T theta = 0, T phi = 0)
    {

        kdtree index(2,*this,KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();

        T queryPoint[2];

        std::vector<size_t> ret_indexes(4);
        std::vector<T> out_dists_sqr(4);

        nanoflann::KNNResultSet<T> resultSet(4);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
            
#pragma omp parallel for
        for(auto star = stars.begin(); star < stars.end(); star++) {
            queryPoint[0] = star->x;
            queryPoint[1] = star->y;

            index.findNeighbors(
                    resultSet,
                    &queryPoint[0],
                    nanoflann::SearchParams(10));
            
            UGVec2<T> a = stars[ret_indexes[0]];
            UGVec2<T> b = stars[ret_indexes[1]];
            UGVec2<T> c = stars[ret_indexes[2]];
            UGVec2<T> d = stars[ret_indexes[3]];

            UGQuad<T>quad = UGQuad<T>(a,b,c,d,theta,phi);

#pragma omp critical(addQuadToQuadset)
            {
                quadset.quads.push_back(quad);
            }
        }

    }

};


#endif
