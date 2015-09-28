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
        for(auto star3D : stars3D)
        {
            stars.push_back(pose3.where(*star3D));
        }

        buildIndex();
    }
}
