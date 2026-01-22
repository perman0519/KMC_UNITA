#pragma once

#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"

namespace VehicleFunctions {
    real findDistanceSquared(const vec3 &a, const vec3 &b);
    bool isWithinLaneDistance(const Lane &lane, const vec3 &position, float maxDistanceSquared);
    vec3 findClosestPoint(const vec3 &point, const vec3 &start, const vec3 &end, real &distanceSquared);
    quat findRotation(const vec3 &heading, const vec3 &up = vec3(0, 1, 0));
    bool findClosestLane(ECS &ecs, vec3 &position, quat &rotation);
} // namespace VehicleFunctions