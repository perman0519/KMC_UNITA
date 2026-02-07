#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"

namespace UserSystem {

    void init(ECS &ecs, World &world, real dt) {
    }

    void update(ECS &ecs, World &world, real dt) {
    }

    void keyDownEvent(ECS &ecs, World &world, Event &baseEvent) {
        KeyDownEvent &event = static_cast<KeyDownEvent &>(baseEvent);
        auto &worldState = world.source.get<WorldState>();
        switch (event.key) {
        case SDLK_SPACE: {
            if (worldState.changing == true)
                return;
            worldState.changing = true;
            world.events.writeLogicEvent(ecs, world, WorldStateEvent{});
            break;
        }
        case SDLK_A: { // spawnAV
            if (worldState.state == WorldState::setup) {
                SpawnVehicleEvent spawnEvent;
                spawnEvent.manual = true;
                spawnEvent.AV = true;
                world.events.writeLogicEvent(ecs, world, spawnEvent);
            }
            break;
        }
        // case SDLK_W: { // spawnHV
        //     if (worldState.state == WorldState::setup) {
        //         SpawnVehicleEvent spawnEvent;
        //         spawnEvent.manual = true;
        //         spawnEvent.AV = false;
        //         world.events.writeLogicEvent(ecs, world, spawnEvent);
        //     }
        //     break;
        // }
        case SDLK_D: { // removeAV
            if (worldState.state == WorldState::setup) {
                RemoveVehicleEvent removeEvent;
                removeEvent.AV = true;
                world.events.writeLogicEvent(ecs, world, removeEvent);
            }
            break;
        }
            // case SDLK_P: { // removeHV
            //     if (worldState.state == WorldState::setup) {
            //         RemoveVehicleEvent removeEvent;
            //         removeEvent.AV = false;
            //         world.events.writeLogicEvent(ecs, world, removeEvent);
            //     }
            //     break;
            // }

        case SDLK_R: {
            world.events.writeLogicEvent(ecs, world, ResetVehiclesEvent{});
            break;
        }
        case SDLK_C: {
            world.events.writeLogicEvent(ecs, world, ClearVehiclesEvent{});
            break;
        }
        case SDLK_1: {
            SaveLoadWorldEvent event;
            event.profile = 1;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_2: {
            SaveLoadWorldEvent event;
            event.profile = 2;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_3: {
            SaveLoadWorldEvent event;
            event.profile = 3;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_4: {
            SaveLoadWorldEvent event;
            event.profile = 4;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_5: {
            SaveLoadWorldEvent event;
            event.profile = 5;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_6: {
            SaveLoadWorldEvent event;
            event.profile = 6;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_7: {
            SaveLoadWorldEvent event;
            event.profile = 7;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_8: {
            SaveLoadWorldEvent event;
            event.profile = 8;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_9: {
            SaveLoadWorldEvent event;
            event.profile = 9;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_S: {
            ConfigChangeEvent event;
            event.save = true;
            event.load = false;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_L: {
            ConfigChangeEvent event;
            event.save = false;
            event.load = true;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        case SDLK_ESCAPE: {
            ConfigChangeEvent event;
            event.save = false;
            event.load = false;
            world.events.writeLogicEvent(ecs, world, event);
            break;
        }
        }
    }
}; // namespace UserSystem

class Engine;
void UserPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, UserSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, UserSystem::update);

    engine.addEventHandler<KeyDownEvent>(UserSystem::keyDownEvent);
};
