#include "appinfo.h"

#include <iostream>

#include "UGUniverse.h"
#include "UGImage.h"
#include "UGLibrary.h"

typedef double ug;

int main(int argc, char *argv[]) {
    srand(time(0));

    UGUniverse<ug> univ(1000,10);
    
    auto dirs = UGDirection<ug>::fibonacci(10000);

    UGLibrary<ug> lib(univ, dirs);

}
