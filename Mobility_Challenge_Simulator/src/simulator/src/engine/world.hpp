#pragma once

#include "pch.hpp"
#include "world/assets.hpp"
#include "world/render.hpp"
#include "world/events.hpp"
#include "world/source.hpp"
#include "world/window.hpp"
#include "world/rosnode.hpp"

struct Config {
    std::string title;
    ivec2 resolution;
    real fps;
};

class World {
public:
    Config config;
    Assets assets;
    Render render;
    Source source;
    Events events;
    Window window;

    std::shared_ptr<ROSNode> ros;

    ivec2 resolution;

public:
    World() {}

    void init(const char *configFile) {
        parseConfigFile(configFile);
        window.init(config.title, config.resolution);
        source.init();
        events.init();
        ros = std::make_shared<ROSNode>();
    }

    void initResources() {
        window.setContext();
        assets.init();
        render.init(resolution);
        setResolution(config.resolution);
    }
    void initDefaultAssets() {
        addFont("default", "CascadiaMono.ttf");
        addMesh("cube", "cube.obj");
        addMesh("vehicle", "vehicle.obj");

        addMesh("mapGrass1", "map.obj");
        addMesh("mapLaneMarkingYellow1", "map.obj");
        addMesh("mapAsphalt1", "map.obj");
        addMesh("mapLaneMarking1", "map.obj");
        addMesh("mapConcrete1", "map.obj");
        addMesh("mapConcrete2", "map.obj");
        addMesh("mapConcrete3", "map.obj");
        addMesh("mapConcrete4", "map.obj");
        addMesh("mapConcrete5", "map.obj");
        addMesh("mapConcrete6", "map.obj");

        addTexture("sign", "sign.png", vec2(3, 1));
        addTexture("id", "IDs.png", vec2(10, 5));
    }

    void setResolution(ivec2 newResolution) {
        resolution = newResolution;
        render.setResolution(resolution);
    }
    ivec2 getResolution() { return resolution; }

    void addMesh(const std::string &name, const char *path) {
        // assets.loadMesh(name);
        render.createMeshBatch(name);
    }
    void addObject(const std::string &name, real inputVertices[], size_t size) {
        render.addMesh(name, inputVertices, size);
        render.createMeshBatch(name);
    }
    void addTexture(const std::string &name, const char *path, vec2 size) {
        assets.loadTexture(name, path, size);
        render.createSpriteBatch(name);
    }
    void addFont(const std::string &name, const char *path) {
        assets.generateAtlas(name, path);
        render.createTextBatch(name);
    }

private:
    void parseConfigFile(const char *configFile) {
        std::string configPath = PATH_CONFIG + std::string(configFile);
        CSimpleIniA ini;
        ini.SetUnicode();
        SI_Error rc = ini.LoadFile(configPath.c_str());
        if (rc < 0) {
            throw std::runtime_error("Failed to load config file");
        }
        config.title = ini.GetValue("project", "name", "Engine");
        config.resolution.x = std::atoi(ini.GetValue("window", "width", "800"));
        config.resolution.y =
            std::atoi(ini.GetValue("window", "height", "600"));
        config.fps = std::stof(ini.GetValue("window", "fps", "60"));
        resolution = config.resolution;
    }
};