// Image.h
#ifndef IMAGE_H
#define IMAGE_H
#include "_ug.h"

#include "Picture.h"
#include "Direction.h"
#include "Vec2.h"
#include "Pose.h"

namespace ugps
{

    Vec2 imageVec2(const Direction& dir);

    class Image : public Picture<Direction,imageVec2>
    {
    public:
        Image(const vector<Direction>& stars);
        Image(const Pose3& pose3, const vector<Vec3>& stars3D);
    };

}

#endif
