#include "pch.hpp"
#include "engine/engine.hpp"
#include "engine/defaults.hpp"

namespace PhysicsSystem {

    void updateTime(ECS &ecs, World &world, real dt) {
        auto &time = world.source.get<Time>();
        time.now += dt;

        auto view = ecs.write<Timer>();
        view.iterate([&](EntityID id, Timer &timer) {
            timer.life -= dt;
            ecs.markChanged<Timer>(id);
        });
    };

    void updatePhysics(ECS &ecs, World &world, real dt) {

        auto view = ecs.write<Physics, State, Motion>();
        view.iterate(
            [&](EntityID id, Physics &physics, State &state, Motion &motion) {
                if (physics.on) {
                    motion.speed += motion.accel * dt;
                    state.position += motion.speed * dt;

                    ecs.markChanged<State>(id);
                    ecs.markChanged<Motion>(id);
                }
            });
    };

} // namespace PhysicsSystem

void PhysicsPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::UPDATE, PhysicsSystem::updateTime);
    // engine.addLogicSystem(SystemType::UPDATE, PhysicsSystem::updatePhysics);
};