#include "pch.hpp"
#include "engine/engine.hpp"
#include "engine/defaults.hpp"

namespace RenderSystem {

    void init(ECS &ecs, World &world, real dt) {

    };

    void interpolate(ECS &ecs, World &world, real dt) {
        auto view = ecs.read<State, Motion>();
        view.iterate([&](EntityID id, State &state, Motion &motion) {
            // state.position = state.position + motion.speed * dt;
        });
    }
    void setCameraUniforms(ECS &ecs, World &world, real dt) {
        auto &camera = world.source.get<Camera>();
        auto &data = world.source.get<CamData>();
        world.render.setUniforms(data.position - camera.target, data.projection,
                                 data.view);
    }
    void updateText(ECS &ecs, World &world, real dt) {
        std::unordered_map<std::string, TextBatch> &textBatches =
            world.render.textBatches;
        for (auto &[name, batch] : textBatches)
            batch.data.clear();
        auto view = ecs.read<State, Text>();
        view.iterate([&](EntityID id, State &state, Text &text) {
            float cursorX = state.position.x;
            float cursorY = state.position.y;
            const auto &glyphMap = world.assets.getGlyph(text.name);
            for (char c : text.text) {
                if (c == ' ') {
                    cursorX +=
                        1.0f * text.size; // Define spaceAdvance based on your font
                    continue;
                }
                auto it = glyphMap.find(static_cast<uint32_t>(c));
                if (it == glyphMap.end())
                    continue; // Character not found
                const Glyph &glyph = it->second;
                float screenX = cursorX + glyph.bearingX * text.size;
                float screenY =
                    cursorY + glyph.bearingY * text.size; // Fix Y positioning
                TextData textData;
                textData.color = text.color;
                textData.position = vec2(screenX, screenY);
                textData.size =
                    vec2(glyph.width * text.size, glyph.height * text.size);
                textData.uv = vec4(glyph.atlasLeft, glyph.atlasBottom,
                                   glyph.atlasRight, glyph.atlasTop);
                textBatches[text.name].data.push_back(textData);
                cursorX += glyph.advance * text.size;
            }
        });
        world.render.getPipeline().getTextBuffer().clearBuffer();
        for (auto &[name, batch] : textBatches)
            world.render.render(batch, world.assets.getFontAtlas(batch.name));
    };
    void updateMesh(ECS &ecs, World &world, real dt) {
        std::unordered_map<std::string, MeshBatch> &meshBatches =
            world.render.meshBatches;
        for (auto &[name, batch] : meshBatches)
            batch.data.clear();
        auto view = ecs.read<State, Mesh>();
        view.iterate([&](EntityID id, State &state, Mesh &mesh) {
            MeshData meshData;
            mat4 translation = glm::translate(mat4(1.0f), state.position);
            mat4 rotation = glm::mat4_cast(state.rotation);
            // apply rotation with quats
            meshData.model = translation * rotation;
            meshData.color = mesh.color;
            meshBatches[mesh.name].data.push_back(meshData);
        });
        world.render.getPipeline().getMeshBuffer().clearBuffer();
        for (auto &[name, batch] : meshBatches)
            world.render.render(batch);
    };
    void updateSprite(ECS &ecs, World &world, real dt) {
        std::unordered_map<std::string, SpriteBatch> &spriteBatches =
            world.render.spriteBatches;
        for (auto &[name, batch] : spriteBatches)
            batch.data.clear();
        auto view = ecs.read<State, Sprite>();
        view.iterate([&](EntityID id, State &state, Sprite &sprite) {
            SpriteData spriteData{.position = vec4(state.position, sprite.size),
                                  .uv = vec2(sprite.frame, sprite.cycle)};
            spriteBatches[sprite.name].data.push_back(spriteData);
        });
        world.render.getPipeline().getSpriteBuffer().clearBuffer();
        for (auto &[name, batch] : spriteBatches)
            world.render.render(batch, world.assets.getTexture(batch.name),
                                world.assets.getSheetSize(batch.name));
    };

    void finalRender(ECS &ecs, World &world, real dt) {
        world.render.finalRender();
    }

    void setResolution(ECS &ecs, World &world, Event &baseEvent) {
        WindowResizeEvent &event = static_cast<WindowResizeEvent &>(baseEvent);
        if (event.type == SDL_EVENT_WINDOW_RESIZED) {
            world.setResolution(ivec2(event.resolution.x, event.resolution.y));
        }
    }

} // namespace RenderSystem

void RenderPlugin(Engine &engine) {
    engine.addRenderSystem(SystemType::INIT, RenderSystem::init);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::interpolate);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::setCameraUniforms);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::updateText);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::updateMesh);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::updateSprite);
    engine.addRenderSystem(SystemType::UPDATE, RenderSystem::finalRender);

    engine.addEventHandler<WindowResizeEvent>(RenderSystem::setResolution, ThreadType::render);
};
