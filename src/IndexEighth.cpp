#include "IndexEighth.h"

using namespace nanoflann;

namespace ugps
{
    IndexEighth::IndexEighth() :
        quads(vector<shared_ptr<const ProjectionQuad> >())
    {
        tree = make_unique<kdtree>(4,*this,KDTreeSingleIndexAdaptorParams(1000));
    }

    void IndexEighth::buildTree()
    {
        tree->buildIndex();
    }


}
