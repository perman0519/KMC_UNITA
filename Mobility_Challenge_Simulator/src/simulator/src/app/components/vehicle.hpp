#pragma once

#include <json.hpp>
#include "pch.hpp"

struct VehicleData {
    vec2 position;
    bool av;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(VehicleData, position, av)
};

struct VehicleSaveData {
    std::vector<VehicleData> vehicles;
    std::string eventName;
    std::string timestamp;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(VehicleSaveData, vehicles, eventName, timestamp)
};

struct Vehicle {
    bool AV = false;
};

struct DesiredVelocity {
    real lon = 0.0f;
    real ang = 0.0f;
};

struct StartingPosition {
    vec3 position = vec3(0.0f);
    quat rotation = quat(1.0f, 0.0f, 0.0f, 0.0f);
};

struct AVQueue : Resource {
    std::deque<EntityID> queue;
};

struct HVQueue : Resource {
    std::deque<EntityID> queue;
};

struct SpawnVehicleEvent : Event {
    bool manual = true;
    bool AV = true;
    VehicleData data;
};

struct RemoveVehicleEvent : Event {
    bool AV = true;
};

struct ResetVehiclesEvent : Event {};

struct ClearVehiclesEvent : Event {
    int savedCAVs = 0;
};

struct TextTag {
    EntityID id;
};
