#include <json.hpp>
#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"
#include "app/functions/vehicle.hpp"
#include <glm/ext/vector_float3.hpp>

using json = nlohmann::json;

namespace nlohmann {
    template <>
    struct adl_serializer<glm::vec<2, float>> {
        static void to_json(nlohmann::json &j, const glm::vec<2, float> &v) {
            j = nlohmann::json{{"x", v.x}, {"y", v.y}};
        }

        static void from_json(const nlohmann::json &j, glm::vec<2, float> &v) {
            j.at("x").get_to(v.x);
            j.at("y").get_to(v.y);
        }
    };
    template <>
    struct adl_serializer<glm::quat> {
        static void to_json(nlohmann::json &j, const glm::quat &q) {
            j = nlohmann::json{{"w", q.w}, {"x", q.x}, {"y", q.y}, {"z", q.z}};
        }

        static void from_json(const nlohmann::json &j, glm::quat &q) {
            j.at("w").get_to(q.w);
            j.at("x").get_to(q.x);
            j.at("y").get_to(q.y);
            j.at("z").get_to(q.z);
        }
    };
} // namespace nlohmann

namespace WorldSettingSystem {

    void resetCameraInit(ECS &ecs, World &world, Event &baseEvent) {
        auto &camera = world.source.get<Camera>();
        camera.pos = vec3(0.0f, 0.0f, 100.0f);
        camera.yaw = glm::radians(-90.0f);
        camera.pitch = glm::radians(0.0f);
        camera.target = vec3(0.0f);
    }

    void changeWorldState(ECS &ecs, World &world, WorldState::State state) {
        auto &worldState = world.source.get<WorldState>();
        worldState.state = state;

        world.ros->changeState(static_cast<int>(state));

        auto view = ecs.write<Sprite, SpriteType>();
        view.iterate([&](EntityID id, Sprite &sprite, SpriteType &type) {
            if (type.fixed) {
                sprite.frame = state;
                ecs.markChanged<Sprite>(id);
            }
        });
    }

    void init(ECS &ecs, World &world, real dt) {
        world.source.add<WorldState>();
        world.source.add<ConfigState>();
        world.source.add<Camera>();
        world.source.add<CamData>();
        world.events.writeLogicEvent(ecs, world, ClearVehiclesEvent{});

        EntityID id = ecs.createEntity();
        ecs.add(id, State{});
        ecs.add(id, Mesh{.name = "map"});

        std::vector<std::string> names = {
            "mapGrass1",
            "mapLaneMarkingYellow1",
            "mapAsphalt1",
            "mapLaneMarking1",
            "mapConcrete1",
            "mapConcrete2",
            "mapConcrete3",
            "mapConcrete4",
            "mapConcrete5",
            "mapConcrete6"};
        std::vector<vec3> colors = {
            vec3(184, 207, 164) / 255.0f,
            vec3(238, 227, 119) / 255.0f,
            vec3(124, 124, 124) / 255.0f,
            vec3(239, 239, 239) / 255.0f,
            vec3(184, 196, 191) / 255.0f,
            vec3(184, 196, 191) / 255.0f,
            vec3(184, 196, 191) / 255.0f,
            vec3(184, 196, 191) / 255.0f,
            vec3(184, 196, 191) / 255.0f,
            vec3(184, 196, 191) / 255.0f};
        for (size_t i = 0; i < names.size(); ++i) {
            EntityID id = ecs.createEntity();
            ecs.add(id, State{});
            ecs.add(id, Mesh{.name = names[i],
                             .color = colors[i]});
        }
        EntityID sign = ecs.createEntity();
        ecs.add(sign, Sprite{.name = "sign",
                             .frame = WorldState::setup});
        ecs.add(sign, State{.position = vec3(1.667f, 0.0f, 0.02f)});
        ecs.add(sign, SpriteType{.fixed = true});
    }

    void update(ECS &ecs, World &world, real dt) {
        auto &mouse = world.source.get<Mouse>();
        if (mouse.right == MouseState::CLICKED || mouse.right == MouseState::PRESSED) {
            vec2 delta = mouse.current - mouse.last;
            real sensitivity = 0.01f;
            auto &camera = world.source.get<Camera>();
            camera.yaw -= delta.x * sensitivity;
            camera.pitch -= delta.y * sensitivity;
            camera.pitch = glm::clamp(camera.pitch, 0.001f, glm::pi<real>() / 2.0f - 0.001f);
        }
        mouse.last = mouse.current;
    }

    void onPhysics(ECS &ecs, bool on) {
        auto view = ecs.write<Physics, Motion, State>();
        view.iterate([&](EntityID id, Physics &physics, Motion &motion, State &state) {
            motion.speed = state.rotation * vec3(0.1f, 0.0f, 0.0f);
            physics.on = on;
            ecs.markChanged<Physics>(id);
            ecs.markChanged<Motion>(id);
        });
    }

    void resetWorldEvent(ECS &ecs, World &world, Event &baseEvent) {
        changeWorldState(ecs, world, WorldState::setup);
        auto &mouse = world.source.get<Mouse>();
        mouse.editing = true;
        onPhysics(ecs, false);
        auto view = ecs.write<StartingPosition, State, Motion>();
        view.iterate([&](EntityID id, StartingPosition &start, State &state, Motion &motion) {
            state.position = start.position;
            state.previous = start.position;
            VehicleFunctions::findClosestLane(ecs, state.position, state.rotation);
            // state.rotation = start.heading;
            motion.accel = vec3(0.0f);
            motion.speed = state.rotation * vec3(0.1f, 0.0f, 0.0f);
            ecs.markChanged<State>(id);
            ecs.markChanged<Motion>(id);
        });
        std::cout << "[LOG] " << "Reset vehicle positions\n";
    }
    void clearWorldEvent(ECS &ecs, World &world, Event &baseEvent) {
        changeWorldState(ecs, world, WorldState::setup);
        auto &mouse = world.source.get<Mouse>();
        mouse.editing = true;
        onPhysics(ecs, false);
        auto view = ecs.write<Vehicle, TextTag>();
        view.iterate([&](EntityID id, Vehicle &vehicle, TextTag &tag) {
            ecs.removeEntity(tag.id);

            ecs.removeEntity(id);
            auto &av = world.source.get<AVQueue>();
            while (!av.queue.empty())
                av.queue.pop_back();
            auto &hv = world.source.get<HVQueue>();
            while (!hv.queue.empty())
                hv.queue.pop_back();
        });
        world.ros->reset();
        world.ros->changeScene(100);
        std::cout << "[LOG] " << "Cleared all vehicles\n";
    }

    void changeWorldStateEvent(ECS &ecs, World &world, Event &baseEvent) {
        auto &worldState = world.source.get<WorldState>();
        if (worldState.changing == false) {
            return;
        }
        if (worldState.state == WorldState::setup) {
            changeWorldState(ecs, world, WorldState::run);
            worldState.changing = false;
            auto &mouse = world.source.get<Mouse>();
            mouse.editing = false;
            onPhysics(ecs, true);
            return;
        }
        if (worldState.state == WorldState::run) {
            changeWorldState(ecs, world, WorldState::pause);
            worldState.changing = false;
            onPhysics(ecs, false);
            return;
        }
        if (worldState.state == WorldState::pause) {
            changeWorldState(ecs, world, WorldState::run);
            worldState.changing = false;
            onPhysics(ecs, true);
            return;
        }
    }

    void mouseWheelEvent(ECS &ecs, World &world, Event &baseEvent) {
        MouseWheelEvent &event = static_cast<MouseWheelEvent &>(baseEvent);
        if (event.type == SDL_EVENT_MOUSE_WHEEL) {
            auto &mouse = world.source.get<Mouse>();
            if (mouse.selected == 0) {
                auto &camera = world.source.get<Camera>();
                real factor = 0.5f;
                if (event.y > 0)
                    camera.zoom += 1.0f * factor;
                if (event.y < 0)
                    camera.zoom -= 1.0f * factor;
            } else {
                auto &state = ecs.get<State>(mouse.selected);
                auto &start = ecs.get<StartingPosition>(mouse.selected);
                float angle = glm::radians(2.5f) * glm::sign(event.y);
                quat zRotation = quat(cos(angle * 0.5f), 0.0f, 0.0f, sin(angle * 0.5f));
                state.rotation = zRotation * state.rotation;
                start.rotation = state.rotation;

                auto &motion = ecs.get<Motion>(mouse.selected);
                motion.speed = state.rotation * vec3(0.1f, 0.0f, 0.0f);
            }
        }
    }

    void configChangeEvent(ECS &ecs, World &world, Event &baseEvent) {
        ConfigChangeEvent &event = static_cast<ConfigChangeEvent &>(baseEvent);
        auto &config = world.source.get<ConfigState>();
        if (config.save or config.load)
            std::cout << "[LOG] " << "Exiting save / load state\n";
        config.save = event.save;
        config.load = event.load;
        if (config.save)
            std::cout << "[LOG] " << "Save vehicle config #1~5\n";
        if (config.load)
            std::cout << "[LOG] " << "Load vehicle config #1~5\n";
    }

    void saveLoadWorldEvent(ECS &ecs, World &world, Event &baseEvent) {
        SaveLoadWorldEvent &event = static_cast<SaveLoadWorldEvent &>(baseEvent);
        std::string filename = std::string("profile") + std::to_string(event.profile) + std::string(".json ");
        auto &config = world.source.get<ConfigState>();
        if (config.save) {
            if (event.profile < 5) {
                std::cout << "attempting to save to preset profile, aborting\n";
                return;
            }
            VehicleSaveData saveData;
            saveData.eventName = std::string("Profile") + std::to_string(event.profile);

            auto now = std::chrono::system_clock::now();
            std::time_t t_now = std::chrono::system_clock::to_time_t(now);
            std::tm tm_now;
#ifdef _WIN32
            localtime_s(&tm_now, &t_now);
#else
            localtime_r(&t_now, &tm_now);
#endif
            std::ostringstream oss;
            oss << std::put_time(&tm_now, "%Y.%m.%d.%H.%M.%S");

            saveData.timestamp = oss.str();
            auto view = ecs.write<Vehicle, StartingPosition, Mesh>();
            view.iterate([&](EntityID id, Vehicle &vehicle, StartingPosition &start, Mesh &mesh) {
                VehicleData data;
                data.position = vec2(start.position.x, start.position.y);
                data.av = vehicle.AV;
                saveData.vehicles.push_back(data);
            });
            json j = saveData;
            std::ofstream file(filename);
            if (file.is_open()) {
                file << j.dump(4);
                file.close();
                std::cout << "[LOG] " << "Vehicle config saved to Profile #" << event.profile << "\n";
            }
            config.save = false;
        }

        if (config.load) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cout << "[LOG] " << "Could not open Profile #" << event.profile << "\n";
                return;
            }
            json j;
            file >> j;
            file.close();
            VehicleSaveData saveData = j.get<VehicleSaveData>();
            world.events.writeLogicEvent(ecs, world, ClearVehiclesEvent{});
            for (const auto &vehicleData : saveData.vehicles) {
                SpawnVehicleEvent event;
                event.manual = false;
                event.data = vehicleData;
                world.events.writeLogicEvent(ecs, world, event);
            };
            std::cout << "[LOG] " << "Vehicle config loaded from Profile #" << event.profile << "\n";
            if (event.profile < 5)
                world.ros->changeScene(event.profile);
            config.load = false;
        }
    }
}; // namespace WorldSettingSystem

void WorldSettingPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, WorldSettingSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, WorldSettingSystem::update);

    engine.addEventHandler<ResetVehiclesEvent>(WorldSettingSystem::resetWorldEvent);
    engine.addEventHandler<ClearVehiclesEvent>(WorldSettingSystem::clearWorldEvent);

    engine.addEventHandler<MouseWheelEvent>(WorldSettingSystem::mouseWheelEvent);
    engine.addEventHandler<WorldStateEvent>(WorldSettingSystem::changeWorldStateEvent);
    engine.addEventHandler<ClearVehiclesEvent>(WorldSettingSystem::resetCameraInit);

    engine.addEventHandler<ConfigChangeEvent>(WorldSettingSystem::configChangeEvent);
    engine.addEventHandler<SaveLoadWorldEvent>(WorldSettingSystem::saveLoadWorldEvent);
};
