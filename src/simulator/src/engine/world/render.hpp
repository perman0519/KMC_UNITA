#pragma once

#include "pch.hpp"
#include "shader.hpp"
#include "pipeline.hpp"

// add color to MeshData and change relevant shaders
struct TextData {
    vec4 uv;
    vec3 color;
    vec2 position;
    vec2 size;
};
struct TextBatch {
    std::string name;
    GLuint instance;
    std::vector<TextData> data;
};
struct MeshData {
    mat4 model;
    vec3 color;
};
struct MeshBatch {
    std::string name;
    GLuint instance;
    std::vector<MeshData> data;
};
struct SpriteData {
    vec4 position;
    vec2 uv;
};
struct SpriteBatch {
    std::string name;
    GLuint instance;
    std::vector<SpriteData> data;
};

class Render {
public:
    std::unordered_map<std::string, TextBatch> textBatches;
    std::unordered_map<std::string, MeshBatch> meshBatches;
    std::unordered_map<std::string, SpriteBatch> spriteBatches;

private:
    ivec2 resolution;

protected:
    std::unique_ptr<Pipeline> pipeline;

    vec3 direction;
    mat4 view, projection, textProjection;

    Shader textShader;
    Shader meshShader;
    Shader spriteShader;

    GLuint textVAO, textVBO;
    GLuint spriteVAO, spriteVBO;

    std::unordered_map<std::string, GLuint> meshVAO;
    std::unordered_map<std::string, GLuint> meshVBO;
    std::unordered_map<std::string, GLsizei> vertices;

public:
    Render() = default;

    void init(ivec2 newResolution) {
        pipeline = std::make_unique<Pipeline>(newResolution);
        setResolution(newResolution);
        initTextRenderer();
        initMeshRenderer();
        initSpriteRenderer();
        initMap();
    }
    void customPipeline(Pipeline customPipeline) {
        pipeline = std::make_unique<Pipeline>(std::move(customPipeline));
    }

    void setResolution(ivec2 newResolution) {
        resolution = newResolution;
        textProjection = glm::ortho(-resolution.x / 2.0f, resolution.x / 2.0f,
                                    -resolution.y / 2.0f, resolution.y / 2.0f);
        pipeline->setResolution(newResolution);
    }
    void setUniforms(const vec3 directionVec, const mat4 &projectionMat,
                     const mat4 &viewMat) {
        direction = directionVec;
        projection = projectionMat;
        view = viewMat;
    }
    Pipeline &getPipeline() { return *pipeline; }
    Shader &getTextShader() { return textShader; }
    Shader &getMeshShader() { return meshShader; }
    Shader &getSpriteShader() { return spriteShader; }

    void addMesh(const std::string &object, real inputVertices[], size_t size) {
        GLuint vao, vbo;
        vertices.insert({object, static_cast<GLsizei>(size / sizeof(real) / 6)});
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, inputVertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real), (void *)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real), (void *)(3 * sizeof(real)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        meshVAO.insert({object, vao});
        meshVBO.insert({object, vbo});
    }

    void render(const TextBatch &batch, GLuint fontAtlas,
                const mat4 &lightMatrix = mat4(1.0f)) {
        textShader.activate();
        textShader.setMat4("projection", textProjection);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fontAtlas);
        textShader.setInt("atlas", 0);
        glBindVertexArray(textVAO);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, batch.data.size() * sizeof(TextData),
                     batch.data.data(), GL_DYNAMIC_DRAW);
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4,
                              static_cast<GLsizei>(batch.data.size()));
        glBindVertexArray(0);
    }
    void render(const MeshBatch &batch, const mat4 &lightMatrix = mat4(1.0f)) {
        meshShader.activate();
        meshShader.setMat4("projection", projection);
        meshShader.setMat4("view", view);
        meshShader.setMat4("lightMatrix", lightMatrix);
        glBindVertexArray(meshVAO[batch.name]);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, batch.data.size() * sizeof(MeshData),
                     batch.data.data(), GL_DYNAMIC_DRAW);
        glDrawArraysInstanced(GL_TRIANGLES, 0, vertices[batch.name],
                              static_cast<GLsizei>(batch.data.size()));
        glBindVertexArray(0);
    }
    void render(const SpriteBatch &batch, GLuint texture, vec2 sheetSize,
                const mat4 &lightMatrix = mat4(1.0f)) {
        spriteShader.activate();
        spriteShader.setMat4("projection", projection);
        spriteShader.setMat4("view", view);
        spriteShader.setVec3("direction", direction);
        spriteShader.setVec2("spriteSheetGrid", sheetSize);
        spriteShader.setMat4("lightMatrix", lightMatrix);
        glBindVertexArray(spriteVAO);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);
        spriteShader.setInt("tSprite", 0);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, batch.data.size() * sizeof(SpriteData),
                     batch.data.data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(SpriteData),
                              (void *)offsetof(SpriteData, position));
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(SpriteData),
                              (void *)offsetof(SpriteData, uv));
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4,
                              static_cast<GLsizei>(batch.data.size()));
        glBindVertexArray(0);
    }
    void finalRender() { pipeline->composite(); }

    void createTextBatch(const std::string &name) {
        TextBatch batch;
        batch.name = name;
        glGenBuffers(1, &batch.instance);
        glBindVertexArray(textVAO);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(TextData), (void *)offsetof(TextData, uv));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(TextData), (void *)offsetof(TextData, color));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(TextData), (void *)offsetof(TextData, position));
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(TextData), (void *)offsetof(TextData, size));
        glVertexAttribDivisor(1, 1);
        glVertexAttribDivisor(2, 1);
        glVertexAttribDivisor(3, 1);
        glVertexAttribDivisor(4, 1);
        glBindVertexArray(0);
        textBatches[name] = batch;
    }
    void createMeshBatch(const std::string &name) {
        MeshBatch batch;
        batch.name = name;
        glGenBuffers(1, &batch.instance);
        glBindVertexArray(meshVAO[name]);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(MeshData), (void *)0);
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MeshData), (void *)(sizeof(vec4)));
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(MeshData), (void *)(2 * sizeof(vec4)));
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(MeshData), (void *)(3 * sizeof(vec4)));
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(MeshData), (void *)(4 * sizeof(vec4)));
        glVertexAttribDivisor(2, 1);
        glVertexAttribDivisor(3, 1);
        glVertexAttribDivisor(4, 1);
        glVertexAttribDivisor(5, 1);
        glVertexAttribDivisor(6, 1);
        glBindVertexArray(0);
        meshBatches[name] = batch;
    }
    void createSpriteBatch(const std::string &name) {
        SpriteBatch batch;
        batch.name = name;
        glGenBuffers(1, &batch.instance);
        glBindVertexArray(spriteVAO);
        glBindBuffer(GL_ARRAY_BUFFER, batch.instance);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(SpriteData),
                              (void *)offsetof(SpriteData, position));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(SpriteData),
                              (void *)offsetof(SpriteData, uv));
        glVertexAttribDivisor(2, 1);
        glVertexAttribDivisor(3, 1);
        glBindVertexArray(0);
        spriteBatches[name] = batch;
    }

private:
    void initTextRenderer() {
        textShader = Shader("2Dtext.vert", "2Dtext.frag");
        float vertices[] = {
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            1.0f,
            1.0f,
        };
        glGenVertexArrays(1, &textVAO);
        glGenBuffers(1, &textVBO);
        glBindVertexArray(textVAO);
        glBindBuffer(GL_ARRAY_BUFFER, textVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices,
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float),
                              (void *)0);
        glBindVertexArray(0);
    }
    void initMeshRenderer() {
        meshShader = Shader("3Dobject.vert", "3Dobject.frag");

        float unit = 0.5f;
        float cube[] = {
            -unit, -unit, -unit, 0.0f, 0.0f, -1.0f, unit, -unit, -unit,
            0.0f, 0.0f, -1.0f, unit, unit, -unit, 0.0f, 0.0f, -1.0f,
            unit, unit, -unit, 0.0f, 0.0f, -1.0f, -unit, unit, -unit,
            0.0f, 0.0f, -1.0f, -unit, -unit, -unit, 0.0f, 0.0f, -1.0f,

            -unit, -unit, unit, 0.0f, 0.0f, 1.0f, unit, -unit, unit,
            0.0f, 0.0f, 1.0f, unit, unit, unit, 0.0f, 0.0f, 1.0f,
            unit, unit, unit, 0.0f, 0.0f, 1.0f, -unit, unit, unit,
            0.0f, 0.0f, 1.0f, -unit, -unit, unit, 0.0f, 0.0f, 1.0f,

            -unit, unit, unit, -1.0f, 0.0f, 0.0f, -unit, unit, -unit,
            -1.0f, 0.0f, 0.0f, -unit, -unit, -unit, -1.0f, 0.0f, 0.0f,
            -unit, -unit, -unit, -1.0f, 0.0f, 0.0f, -unit, -unit, unit,
            -1.0f, 0.0f, 0.0f, -unit, unit, unit, -1.0f, 0.0f, 0.0f,

            unit, unit, unit, 1.0f, 0.0f, 0.0f, unit, -unit, -unit,
            1.0f, 0.0f, 0.0f, unit, unit, -unit, 1.0f, 0.0f, 0.0f,
            unit, -unit, -unit, 1.0f, 0.0f, 0.0f, unit, unit, unit,
            1.0f, 0.0f, 0.0f, unit, -unit, unit, 1.0f, 0.0f, 0.0f,

            -unit, -unit, -unit, 0.0f, -1.0f, 0.0f, unit, -unit, -unit,
            0.0f, -1.0f, 0.0f, unit, -unit, unit, 0.0f, -1.0f, 0.0f,
            unit, -unit, unit, 0.0f, -1.0f, 0.0f, -unit, -unit, unit,
            0.0f, -1.0f, 0.0f, -unit, -unit, -unit, 0.0f, -1.0f, 0.0f,

            -unit, unit, -unit, 0.0f, 1.0f, 0.0f, unit, unit, -unit,
            0.0f, 1.0f, 0.0f, unit, unit, unit, 0.0f, 1.0f, 0.0f,
            unit, unit, unit, 0.0f, 1.0f, 0.0f, -unit, unit, unit,
            0.0f, 1.0f, 0.0f, -unit, unit, -unit, 0.0f, 1.0f, 0.0f};
        addMesh("cube", cube, sizeof(cube));

        std::string configPath = PATH_CONFIG + std::string("config.ini");
        CSimpleIniA ini;
        ini.SetUnicode();
        SI_Error rc = ini.LoadFile(configPath.c_str());
        if (rc < 0) {
            throw std::runtime_error("Failed to load config file");
        }
        float lengthF = std::stof(ini.GetValue("vehicle", "lengthF", "0.17"));
        float lengthR = std::stof(ini.GetValue("vehicle", "lengthR", "0.16"));
        float lengthW = std::stof(ini.GetValue("vehicle", "lengthW", "0.075"));
        float lengthE = lengthW / 2.0f;

        float lenFF = lengthF;
        float lenFE = lengthF - lengthE;
        float lenRE = lengthR;
        float lenWF = lengthW;
        float lenWE = lengthW - lengthE;
        float lenH = 0.12f;
        float lenB = 0.0f;
        float vehicle[] = {
            // Bottom Face
            -lenRE, -lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            lenFE, -lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            lenFE, lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            lenFE, lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            -lenRE, lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            -lenRE, -lenWF, -lenB, 0.0f, 0.0f, -1.0f,

            lenFE, -lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            lenFF, -lenWE, -lenB, 0.0f, 0.0f, -1.0f,
            lenFF, lenWE, -lenB, 0.0f, 0.0f, -1.0f,
            lenFF, lenWE, -lenB, 0.0f, 0.0f, -1.0f,
            lenFE, lenWF, -lenB, 0.0f, 0.0f, -1.0f,
            lenFE, -lenWF, -lenB, 0.0f, 0.0f, -1.0f,

            // Top Face
            -lenRE, -lenWF, lenH, 0.0f, 0.0f, 1.0f,
            lenFE, -lenWF, lenH, 0.0f, 0.0f, 1.0f,
            lenFE, lenWF, lenH, 0.0f, 0.0f, 1.0f,
            lenFE, lenWF, lenH, 0.0f, 0.0f, 1.0f,
            -lenRE, lenWF, lenH, 0.0f, 0.0f, 1.0f,
            -lenRE, -lenWF, lenH, 0.0f, 0.0f, 1.0f,

            lenFE, -lenWF, lenH, 0.0f, 0.0f, 1.0f,
            lenFF, -lenWE, lenH, 0.0f, 0.0f, 1.0f,
            lenFF, lenWE, lenH, 0.0f, 0.0f, 1.0f,
            lenFF, lenWE, lenH, 0.0f, 0.0f, 1.0f,
            lenFE, lenWF, lenH, 0.0f, 0.0f, 1.0f,
            lenFE, -lenWF, lenH, 0.0f, 0.0f, 1.0f,

            // Rear Face
            -lenRE, lenWF, lenH, -1.0f, 0.0f, 0.0f,
            -lenRE, lenWF, -lenB, -1.0f, 0.0f, 0.0f,
            -lenRE, -lenWF, -lenB, -1.0f, 0.0f, 0.0f,
            -lenRE, -lenWF, -lenB, -1.0f, 0.0f, 0.0f,
            -lenRE, -lenWF, lenH, -1.0f, 0.0f, 0.0f,
            -lenRE, lenWF, lenH, -1.0f, 0.0f, 0.0f,

            // Front Face
            lenFF, lenWE, lenH, 1.0f, 0.0f, 0.0f,
            lenFF, -lenWE, -lenB, 1.0f, 0.0f, 0.0f,
            lenFF, lenWE, -lenB, 1.0f, 0.0f, 0.0f,
            lenFF, -lenWE, -lenB, 1.0f, 0.0f, 0.0f,
            lenFF, lenWE, lenH, 1.0f, 0.0f, 0.0f,
            lenFF, -lenWE, lenH, 1.0f, 0.0f, 0.0f,

            // Right Face
            lenFE, -lenWF, -lenB, -1.0f, -1.0f, 0.0f,
            lenFF, -lenWE, -lenB, -1.0f, -1.0f, 0.0f,
            lenFF, -lenWE, lenH, -1.0f, -1.0f, 0.0f,
            lenFF, -lenWE, lenH, -1.0f, -1.0f, 0.0f,
            lenFE, -lenWF, lenH, -1.0f, -1.0f, 0.0f,
            lenFE, -lenWF, -lenB, -1.0f, -1.0f, 0.0f,

            -lenRE, -lenWF, -lenB, 0.0f, -1.0f, 0.0f,
            lenFE, -lenWF, -lenB, 0.0f, -1.0f, 0.0f,
            lenFE, -lenWF, lenH, 0.0f, -1.0f, 0.0f,
            lenFE, -lenWF, lenH, 0.0f, -1.0f, 0.0f,
            -lenRE, -lenWF, lenH, 0.0f, -1.0f, 0.0f,
            -lenRE, -lenWF, -lenB, 0.0f, -1.0f, 0.0f,

            // Left Face
            lenFE, lenWF, -lenB, 1.0f, 1.0f, 0.0f,
            lenFF, lenWE, -lenB, 1.0f, 1.0f, 0.0f,
            lenFF, lenWE, lenH, 1.0f, 1.0f, 0.0f,
            lenFF, lenWE, lenH, 1.0f, 1.0f, 0.0f,
            lenFE, lenWF, lenH, 1.0f, 1.0f, 0.0f,
            lenFE, lenWF, -lenB, 1.0f, 1.0f, 0.0f,

            -lenRE, lenWF, -lenB, 0.0f, 1.0f, 0.0f,
            lenFE, lenWF, -lenB, 0.0f, 1.0f, 0.0f,
            lenFE, lenWF, lenH, 0.0f, 1.0f, 0.0f,
            lenFE, lenWF, lenH, 0.0f, 1.0f, 0.0f,
            -lenRE, lenWF, lenH, 0.0f, 1.0f, 0.0f,
            -lenRE, lenWF, -lenB, 0.0f, 1.0f, 0.0f};
        addMesh("vehicle", vehicle, sizeof(vehicle));
    }
    void initSpriteRenderer() {
        spriteShader = Shader("2Dsprite.vert", "2Dsprite.frag");
        float point[] = {
            0.0f,
            0.0f,
            0.0f,
            -0.5f,
            0.5f,
            0.0f,
            0.0f,
            0.0f,
            -0.5f,
            -0.5f,
            0.0f,
            0.0f,
            0.0f,
            0.5f,
            -0.5f,
            0.0f,
            0.0f,
            0.0f,
            0.5f,
            0.5f,
        };
        glGenVertexArrays(1, &spriteVAO);
        glGenBuffers(1, &spriteVBO);
        glBindVertexArray(spriteVAO);
        glBindBuffer(GL_ARRAY_BUFFER, spriteVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(point), point, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(real),
                              (void *)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(real),
                              (void *)(3 * sizeof(real)));
        glBindVertexArray(0);
    }
    void initMap() {
        struct Object {
            std::vector<unsigned int> vI, uI, nI;
        };
        std::vector<glm::vec3> temp_vertices;
        std::vector<glm::vec2> temp_uvs;
        std::vector<glm::vec3> temp_normals;

        std::unordered_map<std::string, Object> objects;

        std::vector<real> map;

        std::string fullPath = PATH_ASSETS + std::string("map.obj");
        std::ifstream file(fullPath);
        if (!file.is_open()) {
            std::cerr << "Impossible to open the file!" << std::endl;
        }
        std::string line;
        std::string currentMaterial = "default";
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string lineHeader;
            ss >> lineHeader;

            if (lineHeader == "v") {
                glm::vec3 vertex;
                ss >> vertex.x >> vertex.y >> vertex.z;
                temp_vertices.push_back(vertex / 15.0f);
            } else if (lineHeader == "vt") {
                glm::vec2 uv;
                ss >> uv.x >> uv.y;
                temp_uvs.push_back(uv);
            } else if (lineHeader == "vn") {
                glm::vec3 normal;
                ss >> normal.x >> normal.y >> normal.z;
                temp_normals.push_back(normal);
            } else if (lineHeader == "usemtl") {
                // Switch to new material
                ss >> currentMaterial;
            } else if (lineHeader == "f") {
                std::string v[3];
                ss >> v[0] >> v[1] >> v[2];

                for (int i = 0; i < 3; ++i) {
                    std::istringstream fs(v[i]);
                    std::string vi, uvi, ni;

                    std::getline(fs, vi, '/');
                    std::getline(fs, uvi, '/');
                    std::getline(fs, ni, '/');

                    objects[currentMaterial].vI.push_back(std::stoi(vi));
                    objects[currentMaterial].uI.push_back(std::stoi(uvi));
                    objects[currentMaterial].nI.push_back(std::stoi(ni));
                }
            }
        }
        // For each vertex of each triangle
        for (const auto &[materialName, object] : objects) {
            std::vector<real> meshData;
            // For each vertex of each triangle in this material group
            for (unsigned int i = 0; i < object.vI.size(); ++i) {
                unsigned int vIdx = object.vI[i];
                unsigned int nIdx = object.nI[i];

                glm::vec3 vertex = temp_vertices[vIdx - 1];
                glm::vec3 normal = temp_normals[nIdx - 1];

                meshData.push_back(vertex.x);
                meshData.push_back(vertex.y);
                meshData.push_back(vertex.z);
                meshData.push_back(normal.x);
                meshData.push_back(normal.y);
                meshData.push_back(normal.z);
            }
            std::string meshName = "map" + materialName;
            addMesh(meshName, meshData.data(), meshData.size() * sizeof(real));
        }
    }
};
