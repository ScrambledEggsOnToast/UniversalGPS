#include "appinfo.h"

#include <iostream>

#include "UGLibrary.h"

typedef double ug;

int main(int argc, char *argv[]) {
    srand (static_cast <unsigned> (time(0)));

    UGUniverse<ug> univ;
    univ.randomise(10,10);
    
    auto dirs = UGDirection<ug>::fibonacci(100000);
    std::cout << "Directions generated" << std::endl;

    UGLibrary<ug> lib(univ, dirs);
    std::cout << "Library built" << std::endl;

    UGImage<ug> img = univ.project(0.123,1.2);
    std::cout << "Trial image taken" << std::endl;

    UGResult<ug> r = lib.search(img);

    std::cout << r.dir.theta << ", " << r.dir.phi << std::endl;

}
