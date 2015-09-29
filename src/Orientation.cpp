// Orientation.cpp
#include "Orientation.h"

#include "IndexEighth.h"

namespace ugps
{
    template<class T>
    Oriented<T>::Oriented() : t000(T()),t001(T()),t010(T()),t011(T()),t100(T()),t101(T()),t110(T()),t111(T()) {}

    template class Oriented<IndexEighth>;
}
