#include "appinfo.h"

#include <iostream>

#include "_ug.h"

#include "Vec3.h"
#include "Direction.h"
#include "Index.h"

using namespace ugps;

int main(int argc, char *argv[]) {

    const size_t numStars = 10;
    const size_t numDirs = 100000;
    const num_ug universeRadius = 10;
    
    std::vector<Vec3> stars;
    vector<shared_ptr<const Vec3> > starPtrs;

    std::random_device rd;
    std::mt19937 mt(rd());

    std::uniform_real_distribution<num_ug> dist (-universeRadius,universeRadius);
    for(size_t i = 0; i < numStars; i++) {
        Vec3 star;
        star.x = dist(mt);
        star.y = dist(mt);
        star.z = dist(mt);
        stars.push_back(star);
        starPtrs.push_back(make_shared<const Vec3>(star));
    }

    std::vector<Direction> dirs = fibonacciDirections(numDirs);

    Index index(stars, dirs);

    int tests = 100;

    for(int i = 0; i<tests; i++)
    {
        num_ug theta = M_PI*(num_ug)i/(num_ug)tests;

        Projection img(Direction(theta,0), starPtrs);

        Pose3 pose = index.search(img);

        std::cout << theta << "," << pose.dir.theta << std::endl;
    }

}
