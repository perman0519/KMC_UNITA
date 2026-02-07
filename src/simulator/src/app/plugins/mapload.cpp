#include <json.hpp>
#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"

namespace MapLoadSystem {

    using json = nlohmann::json;
    constexpr double DEG2RAD = glm::pi<double>();
    constexpr double EARTH_RADIUS = 6378137.0f;

    void createLaneEntity(ECS &ecs, const json &pathObj) {
        std::vector<real> xs = pathObj["X"].get<std::vector<real>>();
        std::vector<real> ys = pathObj["Y"].get<std::vector<real>>();
        std::vector<glm::vec3> points;
        points.reserve(xs.size());
        for (size_t i = 0; i < xs.size(); i++) {
            points.emplace_back(xs[i], ys[i], 0.0f);
        }
        EntityID id = ecs.createEntity();
        ecs.add(id, Lane{.points = points});
    }

    void parsePath(ECS &ecs, const json &data) {
        for (const auto &item : data) {
            if (item.is_array()) {
                for (const auto &pathObj : item) {
                    createLaneEntity(ecs, pathObj);
                }
            } else if (item.is_object() && item.contains("X") && item.contains("Y")) {
                createLaneEntity(ecs, item);
            }
        }
    }

    void parseMap(ECS &ecs, const json &data) {
        const float scale = 1.0f / 15.0f;
        for (const auto &item : data) {
            std::vector<vec3> points;
            for (const auto &pt : item["points"]) {
                if (pt.size() >= 2)
                    points.push_back({pt[0].get<float>() * scale, pt[1].get<float>() * scale, 0.0f});
            }
        }
    }

    void init(ECS &ecs, World &world, real dt) {
        std::string fullPath = PATH_ASSETS + std::string("path.json");
        std::ifstream file(fullPath);
        if (!file)
            std::cerr << "Failed to open map file\n";
        json data;
        file >> data;
        parsePath(ecs, data);
    }

}; // namespace MapLoadSystem

class Engine;
void MapLoadPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, MapLoadSystem::init);
};
