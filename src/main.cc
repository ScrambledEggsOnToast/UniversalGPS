#include "appinfo.h"

#include <iostream>

#include "_ug.h"

#include "Vec3.h"
#include "Direction.h"
#include "Image.h"
#include "Index.h"
#include "Quad.h"

#include "KdTreeBig.h"

using namespace ugps;

num_ug getVec3Value(const Vec3& v, const size_t& n)
{
    switch(n)
    {
        case 0:
            return v.x;
        case 1:
            return v.y;
        case 2:
            return v.z;
        default: return 0;
    }
}

int main(int argc, char *argv[]) {

    /*const size_t numStars = 10;
    const size_t numDirs = 1000;
    const num_ug universeRadius = 1;
    
    std::vector<Vec3> stars;
    
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        
        std::uniform_real_distribution<num_ug> dist (-universeRadius,universeRadius);
        for(size_t i = 0; i < numStars; i++) {
            Vec3 star;
            star.x = dist(mt);
            star.y = dist(mt);
            star.z = dist(mt);
            stars.push_back(star);
        }
    }

    std::vector<Direction> dirs = fibonacciDirections(numDirs);

    Index index(stars, dirs);

    int tests = 10;

    for(int i = 0; i<tests; i++)
    {
        num_ug theta = M_PI*(num_ug)i/(num_ug)tests;

        Direction spacecraftDir(theta,theta);
        Pose3 spacecraftPose(-1000*spacecraftDir.unit(), spacecraftDir, theta);

        Image img(spacecraftPose, stars);
        
        Pose3 pose = index.search(img);

        std::cout << theta << "," << dist(spacecraftDir, pose.dir) << "," << pose.roll << std::endl;

    }*/


    const size_t numStars = 20000;
    const num_ug universeRadius = 1;
    
    std::vector<Vec3> stars;
    
    {
        std::random_device rd;
        std::mt19937 mt(rd());
        
        std::uniform_real_distribution<num_ug> dist (-universeRadius,universeRadius);
        for(size_t i = 0; i < numStars; i++) {
            Vec3 star;
            star.x = dist(mt);
            star.y = dist(mt);
            star.z = dist(mt);
            stars.push_back(star);
        }
    }
    {
        KdTree<Vec3,getVec3Value,3,5000,64> tree("", stars);
    }
    {
        array<num_ug,3> q{0,0,0};
        KdTree<Vec3,getVec3Value,3,5000,64> tree("");
        
        auto res = tree.knn(q,10);

        for(auto r : res) LOG(r.point.length2());

    }
    LOG("");
    {
        nth_element(stars.begin(), stars.begin() + 10, stars.end(), [](const Vec3& L, const Vec3& R) -> bool
            {
                return L.length2() < R.length2();
            });

        for(auto s = stars.begin(); s < stars.begin()+10; s++) LOG(s->length2());
    }
}
