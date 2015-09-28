// Quad.cpp
#include "Quad.h"

using namespace arma;

namespace ugps
{
    Vec2 projectionVec2(const shared_ptr<const ProjectedStar>& proj) { return proj->pos; }
}
