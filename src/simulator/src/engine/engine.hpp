#pragma once

#include "pch.hpp"
#include "ecs.hpp"
#include "world.hpp"
#include "scenes.hpp"

class CountDownLatch {
    std::mutex mtx;
    std::condition_variable cv;
    int count;

public:
    explicit CountDownLatch(int initial) : count(initial) {}

    void count_down() {
        std::unique_lock<std::mutex> lock(mtx);
        if (--count <= 0) {
            cv.notify_all();
        }
    }

    void wait() {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return count <= 0; });
    }
};

enum class SystemType {
    INIT,
    UPDATE,
    EVENT,
};

enum ThreadType {
    logic,
    render,
};

class Engine {
protected:
    ECS ecs;
    World world;
    Scenes scenes;

private:
    using System = std::function<void(ECS &, World &, real)>;
    std::vector<System> logicInits;
    std::vector<System> logicUpdates;
    std::vector<System> renderInits;
    std::vector<System> renderUpdates;

    using Plugin = std::function<void(Engine &)>;
    std::vector<Plugin> plugins;

    std::chrono::steady_clock::time_point lastLogic;
    std::chrono::steady_clock::time_point lastRender;
    real logicTime;
    real renderTime;
    real step = 1.0f / 100.0f;
    real frame = 1.0f / 120.0f;
    bool isRunning = true;
    size_t frameCount;

    CountDownLatch logicLatch;
    CountDownLatch renderLatch;

public:
    Engine() : logicLatch(1), renderLatch(1) {
        ecs.init();
        scenes.init();
    }

    void run() {
        initPlugins();
        world.init("config.ini");
        std::thread logicThread(&Engine::logicThread, this);
        std::thread renderThread(&Engine::renderThread, this);
        renderLatch.wait();
        SDL_Event event;
        while (isRunning) {
            SDL_PollEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                isRunning = false;
            }
            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                world.events.pushRenderSDLEvent(event);
            } else {
                world.events.pushLogicSDLEvent(event);
            }
        }
        logicThread.join();
        renderThread.join();
        world.window.shutdown();
    }

    void addPlugin(Plugin plugin) { plugins.emplace_back(std::move(plugin)); }
    void addLogicSystem(SystemType type, System system) {
        if (type == SystemType::INIT)
            logicInits.emplace_back(std::move(system));
        if (type == SystemType::UPDATE)
            logicUpdates.emplace_back(std::move(system));
    }
    void addRenderSystem(SystemType type, System system) {
        if (type == SystemType::INIT)
            renderInits.emplace_back(std::move(system));
        if (type == SystemType::UPDATE)
            renderUpdates.emplace_back(std::move(system));
    }
    template <typename EventType>
    void addEventHandler(Events::Handler &&handler, ThreadType type = ThreadType::logic) {
        if (type == ThreadType::logic)
            world.events.addLogicEventHandler<EventType>(std::move(handler));
        if (type == ThreadType::render)
            world.events.addRenderEventHandler<EventType>(std::move(handler));
    }

private:
    void initPlugins() {
        for (auto &init : plugins)
            init(*this);
        plugins.clear();
    }
    void initRender() {
        frameCount = 0;
        renderTime = 0.0f;
        lastRender = std::chrono::steady_clock::now();
        world.initResources();
        world.initDefaultAssets();
        for (auto &system : renderUpdates)
            system(ecs, world, step);
        for (auto &system : renderInits)
            system(ecs, world, step);
        renderLatch.count_down();
    }
    void initLogic() {
        logicTime = 0.0f;
        lastLogic = std::chrono::steady_clock::now();
        for (auto &system : logicUpdates)
            system(ecs, world, step);
        for (auto &system : logicInits)
            system(ecs, world, step);
        logicLatch.count_down();
    }

    void renderThread() {
        logicLatch.wait();
        initRender();
        while (isRunning) {
            updateTime(lastRender, renderTime);
            world.events.flushRenderSDLEvents(ecs, world);
            for (auto &system : renderUpdates)
                system(ecs, world, step);
            world.window.swapBuffer();
        }
    }
    void logicThread() {
        initLogic();
        renderLatch.wait();
        while (isRunning) {
            updateTime(lastLogic, logicTime);
            while (logicTime >= step && isRunning) {
                world.events.flushLogicSDLEvents(ecs, world);
                rclcpp::spin_some(world.ros);
                for (auto &system : logicUpdates)
                    system(ecs, world, step);
                ecs.swapBuffers();
                logicTime -= step;
            }
        }
    }

    void updateTime(std::chrono::steady_clock::time_point &lastTime,
                    real &time) {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<real> deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        time += deltaTime.count();
    }
};

Engine *createApp();

/*template<typename T, typename... Args>
void addLogicSystem(Args&&... args) {
    logicSystems.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
}*/