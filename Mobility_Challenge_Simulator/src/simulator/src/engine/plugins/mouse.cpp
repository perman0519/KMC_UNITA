#include "pch.hpp"
#include "engine/engine.hpp"
#include "engine/defaults.hpp"

namespace MouseSystem {

    bool sphereCheck(const vec3 &point, const real radiusSqrd,
                     const vec3 &linePoint, const vec3 &direction) {
        vec3 v = point - linePoint;
        glm::vec3 crossProduct = glm::cross(v, direction);
        real distanceSqrd = glm::dot(crossProduct, crossProduct);
        if (distanceSqrd > radiusSqrd * glm::dot(direction, direction))
            return false;
        return true;
    }
    bool quadIntersection(const Mouse &mouse, const State &state,
                          const MouseQuad &bound, real &tMin, real &tComp) {
        float t = glm::dot(state.position - mouse.origin, mouse.vecDir);
        if (t < 0 || t > tMin)
            return false;
        tComp = t;
        vec3 hitPoint = mouse.origin + mouse.vecDir * t;
        vec3 local = hitPoint - state.position;
        float x = glm::dot(local, mouse.vecRight);
        float y = glm::dot(local, mouse.vecUp);
        return (std::abs(x) <= bound.half.x && y >= 0 && y <= bound.bound.y);
    }
    bool meshIntersection(const Mouse &mouse, const State &state,
                          const MouseMesh &bound, real &tMin, real &tComp) {
        if (!sphereCheck(state.position, bound.radiusSqrd, mouse.origin,
                         mouse.vecDir))
            return false;

        quat invRot = glm::conjugate(state.rotation);
        vec3 localOrigin = invRot * (mouse.origin - state.position);
        vec3 localDirection = invRot * mouse.vecDir;

        vec3 halfBound = bound.half;
        vec3 invDir = 1.0f / localDirection;
        vec3 t1 = (-halfBound - localOrigin) * invDir;
        vec3 t2 = (halfBound - localOrigin) * invDir;

        vec3 tNear = glm::min(t1, t2);
        vec3 tFar = glm::max(t1, t2);

        real tNearMax = glm::max(glm::max(tNear.x, tNear.y), tNear.z);
        real tFarMin = glm::min(glm::min(tFar.x, tFar.y), tFar.z);

        if (tNearMax > tFarMin || tFarMin < 0)
            return false;
        real t = (tNearMax >= 0) ? tNearMax : tFarMin;
        if (t < 0 || t > tMin)
            return false;
        tComp = t;
        return true;
    }

    void init(ECS &ecs, World &world, real dt) { world.source.add<Mouse>(); };

    void update(ECS &ecs, World &world, real dt) {
        auto &mouse = world.source.get<Mouse>();
        if (!mouse.editing)
            return;
        real tMin = std::numeric_limits<float>::infinity();
        real tComp = std::numeric_limits<float>::infinity();
        EntityID closest;
        bool closestFound = false;

        if (!mouse.selecting && !mouse.detecting) {
            if (mouse.selected != 0) {
                mouse.selected = 0;
                return;
            }
        }
        if (!mouse.selecting)
            return;
        auto viewQuad = ecs.write<State, MouseQuad, Interactive>();
        viewQuad.iterate([&](EntityID id, State &state, MouseQuad &quad,
                             Interactive &interactive) {
            if (!interactive.clickable && !interactive.hoverable &&
                !interactive.draggable)
                return;
            if (!quadIntersection(mouse, state, quad, tMin, tComp))
                return;
            if (tComp < tMin) {
                tMin = tComp;
                closest = id;
                closestFound = true;
            }
        });
        auto viewMesh = ecs.write<State, MouseMesh, Interactive>();
        viewMesh.iterate([&](EntityID id, State &state, MouseMesh &mesh,
                             Interactive &interactive) {
            if (!interactive.clickable && !interactive.hoverable &&
                !interactive.draggable)
                return;
            if (!meshIntersection(mouse, state, mesh, tMin, tComp))
                return;
            if (tComp < tMin) {
                tMin = tComp;
                closest = id;
                closestFound = true;
            }
        });
        if (!closestFound)
            return;

        if (mouse.left == MouseState::CLICKED) {
            auto &interactive = ecs.get<Interactive>(closest);
            if (interactive.clickable) {
                interactive.clicked = true;
                mouse.selected = closest;
                mouse.selecting = true;
                mouse.detecting = false;
            }
        }
    };

    void mouseMotionEvent(ECS &ecs, World &world, Event &baseEvent) {
        MouseMotionEvent &event = static_cast<MouseMotionEvent &>(baseEvent);
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            auto &mouse = world.source.get<Mouse>();
            mouse.last = mouse.current;
            mouse.current = vec2(event.x, event.y);
        }
    }

    void mouseButtonEvent(ECS &ecs, World &world, Event &baseEvent) {
        MouseButtonEvent &event = static_cast<MouseButtonEvent &>(baseEvent);
        if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
            auto &mouse = world.source.get<Mouse>();
            if (event.button == SDL_BUTTON_LEFT) {
                if (mouse.left == MouseState::CLICKED)
                    mouse.left = MouseState::PRESSED;
                if (mouse.left == MouseState::IDLE)
                    mouse.left = MouseState::CLICKED;
                if (mouse.detecting) {
                    mouse.detecting = !mouse.detecting;
                    mouse.selecting = !mouse.selecting;
                }
            }
            if (event.button == SDL_BUTTON_RIGHT) {
                if (mouse.right == MouseState::CLICKED)
                    mouse.right = MouseState::PRESSED;
                if (mouse.right == MouseState::IDLE)
                    mouse.right = MouseState::CLICKED;
            }
            if (event.button == SDL_BUTTON_MIDDLE) {
                if (mouse.scroll == MouseState::CLICKED)
                    mouse.scroll = MouseState::PRESSED;
                if (mouse.scroll == MouseState::IDLE)
                    mouse.scroll = MouseState::CLICKED;
            }
            if (event.button == SDL_BUTTON_X2) {
                if (mouse.sideU == MouseState::CLICKED)
                    mouse.sideU = MouseState::PRESSED;
                if (mouse.sideU == MouseState::IDLE)
                    mouse.sideU = MouseState::CLICKED;
            }
            if (event.button == SDL_BUTTON_X1) {
                if (mouse.sideD == MouseState::CLICKED)
                    mouse.sideD = MouseState::PRESSED;
                if (mouse.sideD == MouseState::IDLE)
                    mouse.sideD = MouseState::CLICKED;
            }
        }
        if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
            auto &mouse = world.source.get<Mouse>();
            if (event.button == SDL_BUTTON_LEFT) {
                mouse.left = MouseState::IDLE;
                mouse.detecting = !mouse.detecting;
            }
            if (event.button == SDL_BUTTON_RIGHT) {
                mouse.right = MouseState::IDLE;
            }
            if (event.button == SDL_BUTTON_MIDDLE) {
                mouse.scroll = MouseState::IDLE;
            }
            if (event.button == SDL_BUTTON_X2) {
                mouse.sideU = MouseState::IDLE;
            }
            if (event.button == SDL_BUTTON_X1) {
                mouse.sideD = MouseState::IDLE;
            }
        }
    };
    void mouseWheelEvent(ECS &ecs, World &world, Event &baseEvent) {
        MouseWheelEvent &event = static_cast<MouseWheelEvent &>(baseEvent);
    }
}; // namespace MouseSystem

void MousePlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, MouseSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, MouseSystem::update);
    engine.addEventHandler<MouseMotionEvent>(MouseSystem::mouseMotionEvent);
    engine.addEventHandler<MouseButtonEvent>(MouseSystem::mouseButtonEvent);
    engine.addEventHandler<MouseWheelEvent>(MouseSystem::mouseWheelEvent);
};