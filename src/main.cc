#include "appinfo.h"

#include <iostream>

#include "_ug.h"

#include "Vec3.h"
#include "Direction.h"
#include "Image.h"
#include "Index.h"
#include "Quad.h"

using namespace ugps;

int main(int argc, char *argv[]) {

    const size_t numStars = 10;
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

    }


}
