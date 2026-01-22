#include "pch.hpp"
#include "engine/engine.hpp"
#include "engine/defaults.hpp"

namespace CameraSystem {

    void init(ECS &ecs, World &world, real dt) {}

    void cameraMoved(ECS &ecs, World &world, Event &baseEvent) {
        CameraMovedEvent &event = static_cast<CameraMovedEvent &>(baseEvent);
        /*vec2 delta = current - last;
        real sensitivity = 0.005f;
        camera.yaw -= delta.x * sensitivity;
        camera.pitch += delta.y * sensitivity;
        camera.pitch = glm::clamp(camera.pitch, 0.000001f, glm::pi<real>() - 0.01f);
        camera.pos = 100.0f * vec3(sin(camera.pitch) * cos(camera.yaw),
        sin(camera.pitch) * sin(camera.yaw), cos(camera.pitch));

        last = current;
        ecs.markChanged<Camera>(id);*/
    }

    void update(ECS &ecs, World &world, real dt) {
        auto &camera = world.source.get<Camera>();
        auto &data = world.source.get<CamData>();

        float scale = pow(2.0f, -camera.zoom / 10.0f);
        float halfWidth = world.getResolution().x * 0.5f * scale;
        float halfHeight = world.getResolution().y * 0.5f * scale;
        camera.pos =
            camera.target + 100.0f * vec3(sin(camera.pitch) * cos(camera.yaw),
                                          sin(camera.pitch) * sin(camera.yaw),
                                          cos(camera.pitch));

        data.direction = -glm::normalize(camera.pos - camera.target);
        if (data.direction == vec3(0.0f, 0.0f, 1.0f) or
            data.direction == vec3(0.0f, 0.0f, -1.0f))
            data.right = normalize(cross(vec3(0.0f, 1.0f, 1.0f), -data.direction));
        else
            data.right = normalize(cross(vec3(0.0f, 0.0f, 1.0f), -data.direction));
        data.up = normalize(cross(-data.direction, data.right));

        data.projection = glm::ortho(
            -halfWidth, halfWidth, -halfHeight, halfHeight, camera.planeN,
            camera.planeF); // fix half width to have zoom when large window
        data.view = glm::lookAt(camera.pos, camera.target, data.up);

        auto &mouse = world.source.get<Mouse>();
        glm::vec3 screenPos = glm::vec3(
            mouse.current.x, world.getResolution().y - mouse.current.y, 0.0f);
        mouse.origin = glm::unProject(
            screenPos, data.view, data.projection,
            glm::vec4(0, 0, world.getResolution().x, world.getResolution().y));
        mouse.vecDir = data.direction;
    }
} // namespace CameraSystem

class Engine;
void CameraPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, CameraSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, CameraSystem::update);

    engine.addEventHandler<CameraMovedEvent>(CameraSystem::cameraMoved);
};