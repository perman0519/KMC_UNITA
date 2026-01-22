#pragma once

#include "pch.hpp"

struct ROSTopic {
    std::string publisher;
    std::string subscriber;
    bool AV;
};

struct ROSData {
    real linear_velocity;
    real angular_velocity;
    bool updated;
};

