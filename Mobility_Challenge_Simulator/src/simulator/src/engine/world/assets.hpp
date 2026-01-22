#pragma once

#include <stb_image.h>
#include <simpleIni.h>
#include <json.hpp>
#include "pch.hpp"

using json = nlohmann::json;

struct Glyph {
    real atlasLeft, atlasBottom, atlasRight, atlasTop;
    ;
    real bearingX, bearingY;
    real width, height;
    real advance;
};
using GlyphMap = std::unordered_map<uint32_t, Glyph>;
struct MeshObject {
    GLuint vao;
    GLuint vbo;
    GLsizei vertices;
};

class Assets {
private:
    std::unordered_map<std::string, GLuint> textures;
    std::unordered_map<std::string, vec2> spriteSheetGrid;

    std::unordered_map<std::string, GLuint> fontAtlas;
    std::unordered_map<std::string, GlyphMap> glyphMap;

    std::unordered_map<std::string, MeshObject> meshObjects;

public:
    Assets() = default;
    void init() {}

    void loadMesh(const std::string &name, real inputVertices[], size_t size) {
        MeshObject object;
        object.vertices = static_cast<GLsizei>(size / sizeof(real) / 6);
        glGenVertexArrays(1, &object.vao);
        glGenBuffers(1, &object.vbo);
        glBindVertexArray(object.vao);
        glBindBuffer(GL_ARRAY_BUFFER, object.vbo);
        glBufferData(GL_ARRAY_BUFFER, size, inputVertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real),
                              (void *)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(real),
                              (void *)(3 * sizeof(real)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        meshObjects.insert({name, object});
    }

    void loadTexture(const std::string &name, const char *path, vec2 sheetSize, bool smoothing = false) {
        int width, height, nrChannels;
        std::string fullPath = PATH_ASSETS + std::string(path);
        stbi_set_flip_vertically_on_load(true);
        unsigned char *data =
            stbi_load(fullPath.c_str(), &width, &height, &nrChannels, 0);
        if (!data)
            throw std::runtime_error(std::string("Failed to load texture: ") +
                                     path);
        GLenum format = GL_RGBA;
        if (nrChannels < 4)
            throw std::runtime_error(
                std::string("Too few channels than RGBA ") + path);

        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
                     GL_UNSIGNED_BYTE, data);
        if (smoothing) {
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        }
        stbi_image_free(data);
        textures[name] = textureID;
        spriteSheetGrid[name] = sheetSize;
    }
    GLuint getTexture(const std::string &name) {
        if (textures.find(name) != textures.end()) {
            return textures[name];
        }
        return 0;
    }
    vec2 getSheetSize(const std::string &name) {
        if (spriteSheetGrid.find(name) != spriteSheetGrid.end()) {
            return spriteSheetGrid[name];
        }
        return vec2(1.0f);
    }

    void generateAtlas(const std::string &name, const char *filePath) {
        std::string fullPath = PATH_FONT + std::string(filePath);
        std::string fileName(filePath);
        std::string fontName(filePath);
        size_t dotPos = fileName.rfind('.');
        if (dotPos != std::string::npos) {
            fileName = fileName.substr(0, dotPos) + ".png";
            fontName = fontName.substr(0, dotPos);
        } else
            fileName += ".png";
        std::string jsonName = fontName + ".json";
        loadAtlas(name, fileName.c_str());
        loadGlyphs(name, jsonName.c_str());
    }
    void loadAtlas(const std::string &name, const char *path) {
        int width, height, nrChannels;
        std::string fullPath = PATH_FONT + std::string(path);
        stbi_set_flip_vertically_on_load(true);
        unsigned char *data =
            stbi_load(fullPath.c_str(), &width, &height, &nrChannels, 0);
        if (!data)
            throw std::runtime_error(std::string("Failed to load texture: ") + path);
        GLenum format = GL_RGBA;
        if (nrChannels < 4)
            throw std::runtime_error(
                std::string("Too few channels than RGBA ") + path);

        GLuint textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format,
                     GL_UNSIGNED_BYTE, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        stbi_image_free(data);
        fontAtlas[name] = textureID;
    }
    void loadGlyphs(const std::string &name, const char *path) {
        std::string fullPath = PATH_FONT + std::string(path);
        std::ifstream file(fullPath);
        json glyphsJson;
        file >> glyphsJson;
        file.close();
        for (auto &[key, value] : glyphsJson.items()) {
            uint32_t codepoint = static_cast<uint32_t>(std::stoul(key));
            Glyph &glyph = glyphMap[name][codepoint];
            glyph.atlasLeft = value["atlasLeft"].get<real>();
            glyph.atlasBottom = value["atlasBottom"].get<real>();
            glyph.atlasRight = value["atlasRight"].get<real>();
            glyph.atlasTop = value["atlasTop"].get<real>();
            glyph.bearingX = value["bearingX"].get<real>();
            glyph.bearingY = value["bearingY"].get<real>();
            glyph.width = value["width"].get<real>();
            glyph.height = value["height"].get<real>();
            glyph.advance = value["advance"].get<real>();
        }
    }
    GLuint getFontAtlas(const std::string &name) {
        if (fontAtlas.find(name) != fontAtlas.end()) {
            return fontAtlas[name];
        }
        return 0;
    }
    GlyphMap &getGlyph(const std::string &name) { return glyphMap[name]; }
};
