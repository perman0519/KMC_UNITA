#pragma once

#include "pch.hpp"

struct Scene {
    std::vector<EntityID> entities;
};

class Scenes {
  private:
    std::unordered_map<std::string, Scene> scenes;
    std::queue<std::string> eventQueue;

    void cleanScene() {}
    void swtichScene(const std::string name) {}

  public:
    Scenes() = default;

    void init() {}

    void addScene(const std::string name, Scene &scene) {
        scenes[name] = scene;
    }

    void loadScene(const char *scanePath) {}

    void queueSceneSwitch(const std::string name) { eventQueue.push(name); }

    void update() {
        while (!eventQueue.empty()) {
            cleanScene();
            swtichScene(eventQueue.front());
        }
    }
};