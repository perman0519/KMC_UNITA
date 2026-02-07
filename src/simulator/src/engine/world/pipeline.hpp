#pragma once

#include "pch.hpp"
#include "shader.hpp"

class Framebuffer {
  private:
    GLuint buffer;
    GLuint tColor;
    GLuint tDepth;
    std::vector<GLuint> tOthers;
    std::vector<GLenum> draws;
    ivec2 screenSize;
    int numTextures;
    bool depthOnly;

    void generateBuffer() {
        glGenFramebuffers(1, &buffer);
        glBindFramebuffer(GL_FRAMEBUFFER, buffer);

        if (!depthOnly) {
            glGenTextures(1, &tDepth);
            glBindTexture(GL_TEXTURE_2D, tDepth);

            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, screenSize.x,
                         screenSize.y, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
                         nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                   GL_TEXTURE_2D, tDepth, 0);

            glGenTextures(1, &tColor);
            glBindTexture(GL_TEXTURE_2D, tColor);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, screenSize.x,
                         screenSize.y, 0, GL_RGBA, GL_FLOAT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                   GL_TEXTURE_2D, tColor, 0);
            draws.push_back(GL_COLOR_ATTACHMENT0);

            for (size_t t = 0; t < numTextures; ++t) {
                GLuint tOther;
                glGenTextures(1, &tOther);
                glBindTexture(GL_TEXTURE_2D, tOther);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, screenSize.x,
                             screenSize.y, 0, GL_RGBA, GL_FLOAT, nullptr);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                                GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                                GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                                GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                                GL_CLAMP_TO_EDGE);
                glFramebufferTexture2D(GL_FRAMEBUFFER,
                                       GL_COLOR_ATTACHMENT1 +
                                           static_cast<GLenum>(t),
                                       GL_TEXTURE_2D, tOther, 0);
                draws.push_back(GL_COLOR_ATTACHMENT1 + static_cast<GLenum>(t));
                tOthers.push_back(tOther);
            }
            glDrawBuffers(static_cast<GLsizei>(draws.size()), draws.data());

        } else {
            glGenTextures(1, &tDepth);
            glBindTexture(GL_TEXTURE_2D, tDepth);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, screenSize.x,
                         screenSize.y, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
                         nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                            GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                            GL_CLAMP_TO_BORDER);
            float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
            glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR,
                             borderColor);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                   GL_TEXTURE_2D, tDepth, 0);
            glDrawBuffer(GL_NONE);
            glReadBuffer(GL_NONE);
        }
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error("Framebuffer not complete!");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    void removeBuffer() {
        for (auto &tOther : tOthers) {
            glDeleteTextures(1, &tOther);
        }
        if (tColor)
            glDeleteTextures(1, &tColor);
        if (tDepth)
            glDeleteTextures(1, &tDepth);
        if (buffer)
            glDeleteFramebuffers(1, &buffer);
    }
    void bindBuffer() {
        glViewport(0, 0, screenSize.x, screenSize.y);
        glBindFramebuffer(GL_FRAMEBUFFER, buffer);
    }

  public:
    Framebuffer(ivec2 screenSize, int textures = 0) : screenSize(screenSize) {
        depthOnly = false;
        numTextures = textures;
        generateBuffer();
    }
    Framebuffer(ivec2 screenSize, bool depth) : screenSize(screenSize) {
        depthOnly = depth;
        generateBuffer();
    }
    ~Framebuffer() { removeBuffer(); }

    void unbindBuffer() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

    void resetBuffer(ivec2 newScreenSize) {
        screenSize = newScreenSize;
        removeBuffer();
        generateBuffer();
    }

    void clearBuffer(const vec4 &color = vec4(0.1f, 0.1f, 0.1f, 0.0f)) {
        bindBuffer();
        glClearColor(color.r, color.g, color.b, color.a);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    GLuint getColorTexture() const { return tColor; }
    GLuint getDepthTexture() const { return tDepth; }
    GLuint getOtherTexture(int index) const { return tOthers[index]; }
};

class Pipeline {
  private:
    Framebuffer textBuffer;
    Framebuffer meshBuffer;
    Framebuffer spriteBuffer;
    Framebuffer antiAliasingBuffer;

    GLuint quadVAO, quadVBO;
    std::unordered_map<std::string, Shader> shaders;
    std::unordered_map<std::string, Framebuffer> buffers;
    int width, height;
    ivec2 resolution;

    vec4 clearColor = vec4(0.2f, 0.2f, 0.2f, 1.0f);

  public:
    Pipeline(ivec2 resolution)
        : textBuffer(resolution), meshBuffer(resolution, 1),
          spriteBuffer(resolution, 1), antiAliasingBuffer(resolution, 1) {
        setResolution(resolution);
        // addShader("final", Shader("post.vert", "post.frag"));
        addShader("composite", Shader("post.vert", "fComposite.frag"));
        // addShader("antialiasing", Shader("post.vert", "fAntialiasing.frag"));
        float quadVertices[] = {-1.0f, 1.0f, 0.0f, 1.0f, -1.0f, -1.0f,
                                0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f,

                                -1.0f, 1.0f, 0.0f, 1.0f, 1.0f, -1.0f,
                                1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices,
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
        glBindVertexArray(0);
    }

    void setResolution(ivec2 newResolution) {
        resolution = newResolution;
        textBuffer.resetBuffer(resolution);
        meshBuffer.resetBuffer(resolution);
        spriteBuffer.resetBuffer(resolution);
        for (auto &[name, buffer] : buffers) {
            buffer.resetBuffer(resolution);
        }
    }

    Framebuffer &getTextBuffer() { return textBuffer; }
    Framebuffer &getMeshBuffer() { return meshBuffer; }
    Framebuffer &getSpriteBuffer() { return spriteBuffer; }
    /*Framebuffer& getBuffer(const std::string& name) {
        return buffers[name];
    }*/
    void addShader(const std::string &name, const Shader &shader) {
        shaders.insert({name, shader});
    }
    Shader &getShader(const std::string &name) { return shaders[name]; }

    void composite() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, resolution.x, resolution.y);
        glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glDisable(GL_DEPTH_TEST);

        shaders["composite"].activate();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textBuffer.getColorTexture());
        shaders["composite"].setInt("tTextColor", 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, meshBuffer.getColorTexture());
        shaders["composite"].setInt("tMeshColor", 1);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, meshBuffer.getDepthTexture());
        shaders["composite"].setInt("tMeshDepth", 2);
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, spriteBuffer.getColorTexture());
        shaders["composite"].setInt("tSpriteColor", 3);
        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, spriteBuffer.getDepthTexture());
        shaders["composite"].setInt("tSpriteDepth", 4);

        renderQuad();
        glEnable(GL_DEPTH_TEST);

        // glBindFramebuffer(GL_FRAMEBUFFER, 0);
        // glViewport(0, 0, resolution.x, resolution.y);
        // glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // shaders["antialiasing"].activate();
        // glActiveTexture(GL_TEXTURE0);
        // glBindTexture(GL_TEXTURE_2D, antiAliasingBuffer.getColorTexture());
        // shaders["antialiasing"].setInt("Color", 0);
        // shaders["antialiasing"].setVec2("InvResolution", 2 * resolution);

        // renderQuad();
    }

    void renderQuad() const {
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);
    }

    void processState() {
        /*	glBindFramebuffer(GL_FRAMEBUFFER, buffer);
            glClear(GL_COLOR_BUFFER_BIT);
            shader.activate();
            shader.setVec4("resolution", resolution);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex.color);
            shader.setInt("tDiffuse", 0);
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, tex.depth);
            shader.setInt("tDepth", 1);
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, tex.normal);
            shader.setInt("tNormal", 2);
            glBindVertexArray(frameVAO);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glBindVertexArray(0);*/
    }

    void applyBloom(Framebuffer &source, Framebuffer &destination) {
        // Extract bright areas
        // Apply gaussian blur
        // Composite back
        // Implementation details...
    }
};