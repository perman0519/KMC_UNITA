#pragma once

#include "pch.hpp"

struct Text {
    std::string name = "default";
    std::string text = "default text";
    vec3 color = vec3(1.0f);
    real size = 1.0f;
    bool base = true;
    bool render = true;
};

struct Mesh {
    std::string name = "object";
    vec3 color = vec3(1.0f);
    bool base = true;
    bool render = true;
};

struct Sprite {
    std::string name;
    real size = 1.0f;
    size_t cycle = 0;
    size_t frame = 0;
    bool render = true;
};
