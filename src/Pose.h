// Pose.h
#ifndef POSE_H
#define POSE_H
#include "_ug.h"

#include "Vec3.h"
#include "Direction.h"

namespace ugps
{

    struct Pose3
    {
        Pose3() : loc(Vec3()), dir(Direction()), roll(0) {}
        Pose3(Vec3 loc, Direction dir, num_ug roll) : loc(loc), dir(dir), roll(roll) {}
        
        Vec3 loc;
        Direction dir;
        num_ug roll;

        Direction where(const Vec3& star3D) const
        {
            Vec3 relPos = star3D-loc;
            relPos.rotateZ(-roll);
            relPos.rotateY(-dir.theta);
            relPos.rotateZ(-dir.phi);
            return relPos;
        }
    };

    struct Pose2
    {
        Pose2() : loc(Vec2()), dir(Direction()), scale(0), roll(0) {}
        Pose2(Vec2 loc, Direction dir, num_ug roll, num_ug scale) : loc(loc), dir(dir), scale(scale), roll(roll) {}
        
        Vec2 loc;
        Direction dir;
        num_ug scale;
        num_ug roll;
        
        Pose3 pose3() const
        {
            num_ug ct = cos(dir.theta);
            num_ug cp = cos(dir.phi);
            num_ug st = sin(dir.theta);
            num_ug sp = sin(dir.phi);

            Vec3 pos3(
                    cp*ct*loc.x - sp*loc.y + cp*st*scale,
                    sp*ct*loc.x + cp*loc.y + sp*st*scale,
                    - st*loc.x + ct*scale
                    );
            return Pose3(pos3, dir, roll);
        }

    };
}

#endif
