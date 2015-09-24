#include "appinfo.h"

#include <iostream>

#include "UGQuad.h"
#include "UGUniverse.h"
#include "UGImage.h"
#include "UGLibrary.h"

int main(int argc, char *argv[]) {
    srand(time(0));

    UGUniverse<double> univ;
    univ.randomise(100,10);
    
    std::vector<UGDirection<double> > dirs = UGDirection<double>::fibonacci(10000);

    UGQuadSet<double> qs;

    UGLibrary<double> lib(univ, dirs);

}
