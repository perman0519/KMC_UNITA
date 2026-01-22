#pragma once

#include "pch.hpp"
#include "engine/engine.hpp"
#include "engine/components/events.hpp"

#include "engine/components/camera.hpp"
#include "engine/components/mouse.hpp"
#include "engine/components/physics.hpp"
#include "engine/components/render.hpp"

class Engine;

void PhysicsPlugin(Engine &engine);
void MousePlugin(Engine &engine);
void CameraPlugin(Engine &engine);
void RenderPlugin(Engine &engine);
