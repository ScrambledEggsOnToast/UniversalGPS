// Image.cpp
#include "Image.h"

namespace ugps
{
    Vec2 imageVec2(const Direction& dir)
    {
        return Vec2(cos(dir.phi)*dir.theta, sin(dir.phi)*dir.theta);
    }

    Image::Image(const vector<Direction>& dirs)
    {
        stars = dirs;
        buildIndex();
    }

    Image::Image(const Pose3& pose3, const vector<shared_ptr<const Vec3> >& stars3D)
    {
#pragma omp parallel for
        for(auto star3D = stars3D.begin(); star3D < stars3D.end(); star3D++)
        {
            Direction dir = pose3.where(**star3D);
#pragma omp critical(addDirection)
            {
                stars.push_back(dir);
            }
        }

        buildIndex();
    }
}
