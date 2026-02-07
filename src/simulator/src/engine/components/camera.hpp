#pragma once

#include "pch.hpp"

struct Camera : Resource {
    vec3 target = vec3(0.0f);
    vec3 pos = vec3(0.0);
    real yaw = glm::radians(-90.0f);
    real pitch = glm::radians(60.0f);
    real planeN = 50.0f;
    real planeF = 500.0f;
    real zoom = 50.0f;
    bool pivot = true;
};

struct CamData : Resource {
    mat4 view;
    mat4 projection;
    vec3 position;
    vec3 right;
    vec3 up = vec3(0.0f, 0.0f, 1.0f);
    vec3 origin;
    vec3 direction;
};

struct CameraMovedEvent : public Event {};
