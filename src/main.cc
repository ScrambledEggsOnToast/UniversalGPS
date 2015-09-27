#include "appinfo.h"

#include <iostream>

#include <armadillo>

#include "UGVec2.h"
#include "UGVec3.h"
#include "UGUniverse.h"
#include "UGImage.h"
#include "UGQuad.h"
#include "UGLibrary.h"

using namespace arma;

typedef double ug;
typedef Mat<ug> ugmat;

int main(int argc, char *argv[]) {
    srand (static_cast <unsigned> (time(0)));

    size_t numStars = 10;
    std::cout << "Generating " << numStars << " stars..." << std::endl;
    UGUniverse<ug> *univ = new UGUniverse<ug>();
    univ->randomise(numStars,1);

    size_t numDirs = 100000;
    
    std::cout << "Generating " << numDirs << " directions..." << std::endl;
    auto dirs = UGDirection<ug>::fibonacci(numDirs);

    std::cout << "Building library..." << std::endl;
    UGLibrary<ug> *lib = new UGLibrary<ug>(*univ, dirs, 1);

    UGDirection<ug> imgDir(0,0);
    UGImage<ug> img;
    UGResult<ug> r;

    for(int i = 0; i<1000; i++)
    {
        imgDir.theta = (ug)i;
        img = univ->project((ug)i,0);
        r = lib->search(img, sqrt(4*M_PI/(ug)numDirs));

        std::cout << (r.loc-imgDir.unit()).length() << ",";
    }

    delete lib;
    delete univ;

}
