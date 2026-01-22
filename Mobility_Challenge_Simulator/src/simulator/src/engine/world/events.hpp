#pragma once

#include "pch.hpp"
#include "engine/components/events.hpp"

class ECS;
class World;

class Events {
public:
    using Handler = std::function<void(ECS &, World &, Event &)>;

private:
    std::unordered_map<size_t, std::vector<Handler>> logicEventHandler;
    std::unordered_map<size_t, std::vector<Handler>> renderEventHandler;

    std::mutex logicMutex;
    std::queue<SDL_Event> logicQueue[2];
    std::queue<SDL_Event> *readLogic = nullptr;
    std::queue<SDL_Event> *writeLogic = nullptr;

    std::mutex renderMutex;
    std::queue<SDL_Event> renderQueue[2];
    std::queue<SDL_Event> *readRender = nullptr;
    std::queue<SDL_Event> *writeRender = nullptr;

public:
    Events() = default;

    void init() {
        readLogic = &logicQueue[0];
        writeLogic = &logicQueue[1];

        readRender = &renderQueue[0];
        writeRender = &renderQueue[1];
    }

    template <typename T>
    void addLogicEventHandler(Handler &&handler) {
        size_t type = getStructHash<T>();
        logicEventHandler[type].emplace_back(std::move(handler));
    }

    template <typename T>
    void addRenderEventHandler(Handler &&handler) {
        size_t type = getStructHash<T>();
        renderEventHandler[type].emplace_back(std::move(handler));
    }

    template <typename T>
    void writeLogicEvent(ECS &ecs, World &world, T &&event) {
        size_t type = getStructHash<std::decay_t<T>>();
        for (auto &handler : logicEventHandler[type])
            handler(ecs, world, event);
    }

    template <typename T>
    void writeRenderEvent(ECS &ecs, World &world, T &&event) {
        size_t type = getStructHash<std::decay_t<T>>();
        for (auto &handler : renderEventHandler[type])
            handler(ecs, world, event);
    }

    void pushLogicSDLEvent(SDL_Event &event) {
        std::lock_guard<std::mutex> lock(logicMutex);
        writeLogic->push(event);
    }

    void flushLogicSDLEvents(ECS &ecs, World &world) {
        {
            std::lock_guard<std::mutex> lock(logicMutex);
            std::swap(writeLogic, readLogic);
        }
        while (!readLogic->empty()) {
            SDL_Event &sdlEvent = readLogic->front();
            // Convert SDL events to custom events
            switch (sdlEvent.type) {
            case SDL_EVENT_MOUSE_BUTTON_DOWN: {
                MouseButtonEvent mouseEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(mouseEvent));
                break;
            }
            case SDL_EVENT_MOUSE_BUTTON_UP: {
                MouseButtonEvent mouseEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(mouseEvent));
                break;
            }
            case SDL_EVENT_MOUSE_WHEEL: {
                MouseWheelEvent wheelEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(wheelEvent));
                break;
            }
            case SDL_EVENT_KEY_DOWN: {
                KeyDownEvent keyEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(keyEvent));
                break;
            }
            case SDL_EVENT_KEY_UP: {
                KeyUpEvent keyEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(keyEvent));
                break;
            }
            case SDL_EVENT_MOUSE_MOTION: {
                MouseMotionEvent motionEvent(readLogic->front());
                writeLogicEvent(ecs, world, std::move(motionEvent));
                break;
            }
            default:
                // Handle other events or ignore
                break;
            }
            readLogic->pop();
        }
    }

    void pushRenderSDLEvent(SDL_Event &event) {
        std::lock_guard<std::mutex> lock(renderMutex);
        writeRender->push(event);
    }
    void flushRenderSDLEvents(ECS &ecs, World &world) {
        {
            std::lock_guard<std::mutex> lock(renderMutex);
            std::swap(writeRender, readRender);
        }
        while (!readRender->empty()) {
            SDL_Event &sdlEvent = readRender->front();
            // Convert SDL events to custom events
            switch (sdlEvent.type) {
            case SDL_EVENT_WINDOW_RESIZED: {
                WindowResizeEvent resizeEvent(readRender->front());
                writeRenderEvent(ecs, world, std::move(resizeEvent));
                break;
            }
            default:
                break;
            }
            readRender->pop();
        }
    }

    // void dispatchLogicEvents(ECS& ecs, Source& source) {
    //     std::vector<std::unique_ptr<Event>> queue;
    //     {
    //         std::lock_guard lock(logicQueueMutex);
    //         queue.swap(logicQueue);
    //     }

    //    for (auto& e : queue) {
    //        size_t type = getStructHash<std::decay_t<decltype(*e)>>(); // RTTI
    //        or your hash for (auto& handler : logicHandlers[type]) {
    //            handler(ecs, source, *e);
    //        }
    //    }
    //}
};