// ProjectedStar.h
#ifndef PROJECTEDSTAR_H
#define PROJECTEDSTAR_H
#include "_ug.h"

#include "Vec2.h"
#include "Vec3.h"
#include "Direction.h"

namespace ugps
{
    class ProjectedStar
    {
    public:
        ProjectedStar(
                const shared_ptr<const Vec3>& pos3D,
                const shared_ptr<const Direction>& dir
                );

        Vec2 pos;
        shared_ptr<const Vec3> pos3D;
        shared_ptr<const Direction> dir;

    };
}

#endif
