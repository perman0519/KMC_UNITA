#pragma once

#include "pch.hpp"

enum UserEventType {
    addAV,
    subAV,
    addHV,
    subHV,
    saveConfig,
    loadConfig,
    startSim,
    stopSim,
    resetSim,
    clearSim,
};

struct WorldStateEvent : Event {};

struct UserEvent : public Event {
    EventType type;
    SDL_Keycode key;
    UserEvent(const SDL_Event &event) {
        type = event.type;
        key = event.key.key;
    }
};