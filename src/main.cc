#include "appinfo.h"

#include <iostream>

#include "UGLibrary.h"

typedef double ug;

int main(int argc, char *argv[]) {
    srand (static_cast <unsigned> (time(0)));

    UGUniverse<ug> univ;
    univ.randomise(100,10);
    
    auto dirs = UGDirection<ug>::fibonacci(100000);
    std::cout << "Directions generated" << std::endl;

    UGLibrary<ug> lib(univ, dirs);
    std::cout << "Library built" << std::endl;

    for(int i = 0; i<20; i++)
    {
        UGDirection<ug> imgDir((ug)i,0);
        UGImage<ug> img = univ.project((ug)i,0);
        UGResult<ug> r = lib.search(img, 0.001);

        std::cout << UGDirection<ug>::dist(r.dir,imgDir) << std::endl;
    }

}
