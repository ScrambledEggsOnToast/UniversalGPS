// ProjectedStar.cpp
#include "ProjectedStar.h"

namespace ugps
{
    ProjectedStar::ProjectedStar(
            const Vec3* pos3D,
            const Direction* dir
            ) : pos3D(pos3D), dir(dir)
    {
        num_ug u = cos(dir->phi)*cos(dir->theta)*pos3D->x 
            + sin(dir->phi)*cos(dir->theta)*pos3D->y
            - sin(dir->theta)*pos3D->z;
        num_ug v = - sin(dir->phi)*pos3D->x 
            + cos(dir->phi)*pos3D->y;

        pos = Vec2(u,v);
    }
}

