#pragma once

#include "pch.hpp"

struct MouseMesh {
    vec3 bound = vec3(1.0f);
    vec3 half = bound / 2.0f;
    real radiusSqrd = glm::length(half) * glm::length(half);
};

struct MouseQuad {
    vec2 bound = vec2(1.0f);
    vec2 half = bound / 2.0f;
    real radiusSqrd = glm::length(half) * glm::length(half);
};

struct Interactive {
    bool clickable = false;
    bool hoverable = false;
    bool draggable = false;

    bool clicked = false;
    bool hovered = false;
    bool dragged = false;
};

enum class MouseState {
    IDLE,
    CLICKED,
    PRESSED,
    RELEASED,
};

struct Mouse : Resource {
    vec3 origin = vec3(0.0f);
    vec3 vecDir = vec3(0.0f);
    vec3 vecRight = vec3(0.0f);
    vec3 vecUp = vec3(0.0f);
    vec2 current = vec2(0.0f);
    vec2 last = vec2(0.0f);
    MouseState left = MouseState::IDLE;
    MouseState right = MouseState::IDLE;
    MouseState scroll = MouseState::IDLE;
    MouseState sideU = MouseState::IDLE;
    MouseState sideD = MouseState::IDLE;

    EntityID selected = 0;
    bool selecting = false;
    bool detecting = true;

    bool editing = true;
};