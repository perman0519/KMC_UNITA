#pragma once

#include "pch.hpp"

struct CollisionBoxSize : Resource {
    real LF;
    real LR;
    real LW;
};

struct CollisionMesh {
    std::vector<vec2> points;
};
