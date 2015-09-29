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
                const Vec3* pos3D,
                const Direction* dir
                );

        Vec2 pos;
        const Vec3* pos3D;
        const Direction* dir;

    };
}

#endif
