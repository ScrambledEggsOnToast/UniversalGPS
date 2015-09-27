// Index.h
#ifndef INDEX_H
#define INDEX_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Vec3.h"
#include "Direction.h"
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

        Pose3 search(const Projection& proj) const;

    private:
        vector<shared_ptr<const Quad> > quads;
        unique_ptr<kdtree> tree;

        vector<shared_ptr<const Quad> > nearestNeighbours(const Quad& query, size_t n) const;
        vector<shared_ptr<const Quad> > radiusSearch(const Quad& query, const num_ug& r) const;

        vector<shared_ptr<const Vec3> > sharedStars;
        vector<shared_ptr<const Projection> > sharedProjections;

    };
}

#endif
