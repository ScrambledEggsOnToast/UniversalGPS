// IndexEighth.h
#ifndef INDEXEIGHTH_H
#define INDEXEIGHTH_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Quad.h"

namespace ugps
{
    class IndexEighth 
    {
    public:
        IndexEighth();

        void buildTree();

        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<num_ug, IndexEighth>,
                IndexEighth,
                4
                > kdtree;

        unique_ptr<kdtree> tree;
        vector<shared_ptr<const ProjectionQuad> > quads;

        // start nanoflann interface
        inline size_t kdtree_get_point_count() const
        {
            return quads.size();
        }
        inline num_ug kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const
        {
            num_ug dq1 = quads[idx_p2]->q1-p1[0];
            num_ug dq2 = quads[idx_p2]->q2-p1[1];
            num_ug dq3 = quads[idx_p2]->q3-p1[2];
            num_ug dq4 = quads[idx_p2]->q4-p1[3];
            return dq1*dq1 + dq2*dq2 + dq3*dq3 + dq4*dq4;
        }
        inline num_ug kdtree_get_pt(const size_t idx, int dim) const
        {
            switch(dim)
            {
                case 0:
                    return quads[idx]->q1;
                case 1:
                    return quads[idx]->q2;
                case 2:
                    return quads[idx]->q3;
                case 3:
                    return quads[idx]->q4;
                default:
                    throw std::out_of_range("Quad hash index outside bounds");
            }
        }
        template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const
        {
            return false;
        }
        // end nanoflann interface
    };
}

#endif
