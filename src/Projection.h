// Projection.h
#ifndef PROJECTION_H
#define PROJECTION_H
#include "_ug.h"

#include <nanoflann.hpp>

#include "Vec3.h"
#include "ProjectedStar.h"
#include "Quad.h"

using namespace nanoflann;

namespace ugps
{
    class Projection
    {
    public:
        Projection(Direction dir, vector<shared_ptr<const Vec3> > stars);

        shared_ptr<Direction> direction;
        vector<shared_ptr<const ProjectedStar> > projectedStars;

        vector<shared_ptr<const ProjectedStar> > nearestNeighbours(const Vec2& query, size_t n) const;

        void calculateQuads(vector<shared_ptr<const Quad> >& quads) const;

        // start nanoflann interface
        inline size_t kdtree_get_point_count() const;
        inline num_ug kdtree_distance(const num_ug* p1, const size_t idx_p2, size_t) const;
        inline num_ug kdtree_get_pt(const size_t idx, int dim) const;
        template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const;
        typedef KDTreeSingleIndexAdaptor<
                L2_Simple_Adaptor<num_ug, Projection>,
                Projection,
                7
                > kdtree;
        // end nanoflann interface
    private:
        unique_ptr<kdtree> tree;
    };
}

#endif
