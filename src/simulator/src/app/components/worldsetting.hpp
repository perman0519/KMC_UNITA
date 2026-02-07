#pragma once

#include "pch.hpp"

struct WorldState : Resource {
    enum State {
        setup,
        run,
        pause,
    };
    State state = setup;
    bool changing = false;
};

struct ConfigState : Resource {
    bool save = false;
    bool load = false;
};

struct ConfigChangeEvent : Event {
    bool save = false;
    bool load = false;
};

struct SaveLoadWorldEvent : Event {
    int profile = 0;
    int savedCAVs = 0;
};

struct SpriteType {
    EntityID id;
    bool fixed;
};

struct CurrentScene : Resource {
    int number = 10;
};