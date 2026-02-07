#pragma once

#include "pch.hpp"
struct MouseMotionEvent : public Event {
    EventType type;
    real x;
    real y;
    MouseMotionEvent(const SDL_Event &event) {
        type = event.type;
        x = event.motion.x;
        y = event.motion.y;
    }
};

struct MouseButtonEvent : public Event {
    EventType type;
    Uint8 button;
    real x;
    real y;
    MouseButtonEvent(const SDL_Event &event) {
        type = event.type;
        button = event.button.button;
        x = event.button.x;
        y = event.button.y;
    }
};

struct MouseWheelEvent : public Event {
    EventType type;
    real y;
    MouseWheelEvent(const SDL_Event &event) {
        type = event.type;
        y = event.wheel.y;
    }
};

struct KeyDownEvent : public Event {
    EventType type;
    SDL_Keycode key;
    KeyDownEvent(const SDL_Event &event) {
        type = event.type;
        key = event.key.key;
    }
};

struct KeyUpEvent : public Event {
    EventType type;
    SDL_Keycode key;
    KeyUpEvent(const SDL_Event &event) {
        type = event.type;
        key = event.key.key;
    }
};

struct WindowResizeEvent : Event {
    EventType type;
    ivec2 resolution;
    WindowResizeEvent(const SDL_Event &event) {
        type = event.type;
        resolution = ivec2(event.window.data1, event.window.data2);
    }
};