#pragma once

#include "pch.hpp"

enum Layer : int {
    LAYER_NONE = 0,
    LAYER_MOUSE = 1 << 0,
    LAYER_PLAYER = 1 << 1,
    LAYER_ENEMY = 1 << 2,
};

struct Collider {
    Layer type = LAYER_NONE;
    Layer mask = LAYER_NONE;
    real width = 1.0f;
    real height = 1.0f;
};

struct Detector {
    Layer type = LAYER_NONE;
    Layer mask = LAYER_NONE;
    real width = 1.0f;
    real height = 1.0f;
};

struct State {
    vec3 position = vec3(0.0f);
    vec3 previous = vec3(0.0f);
    quat rotation = quat(1.0f, 0.0f, 0.0f, 0.0f);
};

struct Motion {
    vec3 speed = vec3(0.0f);
    vec3 accel = vec3(0.0f);
};

struct Physics {
    bool on = true;
};

struct Time : Resource {
    real now = 0.0f;
};

struct Timer {
    real life = 10.0f;
};