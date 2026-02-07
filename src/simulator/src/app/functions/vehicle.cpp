#include "app/functions/vehicle.hpp"

namespace VehicleFunctions {

    real findDistanceSquared(const vec3 &a, const vec3 &b) {
        vec3 diff = a - b;
        return dot(diff, diff);
    }
    bool isWithinLaneDistance(const Lane &lane, const vec3 &position, float maxDistanceSquared) {
        for (const vec3 &point : lane.points)
            if (findDistanceSquared(position, point) <= maxDistanceSquared) {
                return true;
            }
        return false;
    }
    vec3 findClosestPoint(const vec3 &point, const vec3 &start, const vec3 &end, real &distanceSquared) {
        vec3 segmentVec = end - start;
        vec3 pointVec = point - start;
        float segmentLength = length(segmentVec);
        if (segmentLength < 1e-6f) {
            distanceSquared = findDistanceSquared(point, start);
            return start;
        }
        float t = dot(pointVec, segmentVec) / (segmentLength * segmentLength);
        t = std::clamp(t, 0.0f, 1.0f);
        vec3 closestPoint = start + t * segmentVec;
        distanceSquared = findDistanceSquared(point, closestPoint);
        return closestPoint;
    }
    quat findRotation(const vec3 &heading, const vec3 &up) {
        vec2 dir2D = normalize(vec2(heading.x, heading.y));
        float angle = atan2(dir2D.y, dir2D.x); // Rotation around Z-axis
        return angleAxis(angle, vec3(0.0f, 0.0f, 1.0f));
    }
    bool findClosestLane(ECS &ecs, vec3 &position, quat &rotation) {
        bool found = false;
        vec3 tangent;
        vec3 globalClosestPoint;
        real globalClosestDistanceSquared = std::numeric_limits<float>::max();

        auto lanes = ecs.write<Lane>();
        lanes.iterate([&](EntityID id, Lane &lane) {
            if (!isWithinLaneDistance(lane, position, 4.0f))
                return;
            for (int i = 0; i < lane.points.size() - 1; ++i) {
                const vec3 &p1 = lane.points[i];
                const vec3 &p2 = lane.points[i + 1];
                real distanceSquared;
                vec3 closestPoint = findClosestPoint(position, p1, p2, distanceSquared);
                if (distanceSquared < globalClosestDistanceSquared) {
                    globalClosestDistanceSquared = distanceSquared;
                    globalClosestPoint = closestPoint;

                    vec3 segmentDir = p2 - p1;
                    float segmentLength = sqrt(dot(segmentDir, segmentDir));
                    if (segmentLength > 1e-6f)
                        tangent = segmentDir / segmentLength;
                    else
                        tangent = vec3(1, 0, 0);
                    found = true;
                }
                if (distanceSquared < 0.01f) {
                    break;
                }
            } });
        if (found) {
            position = globalClosestPoint;
            rotation = findRotation(tangent);
        }
        return found;
    }

} // namespace VehicleFunctions