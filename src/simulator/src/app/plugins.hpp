#pragma once

#include "engine/defaults.hpp"
#include "components/map.hpp"
#include "components/ros.hpp"
#include "components/user.hpp"
#include "components/vehicle.hpp"
#include "components/collision.hpp"
#include "components/worldsetting.hpp"

class Engine;

void CollisionPlugin(Engine &engine);
void MapLoadPlugin(Engine &engine);
void ROSPlusgin(Engine &engine);
void UserPlugin(Engine &engine);
void VehiclePlugin(Engine &engine);
void WorldSettingPlugin(Engine &engine);