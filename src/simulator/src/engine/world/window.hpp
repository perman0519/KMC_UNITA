#pragma once

#include "pch.hpp"

class Window {
  private:
    SDL_Window *window = nullptr;
    SDL_GLContext glContext = nullptr;

    std::string title;
    ivec2 resolution;

  public:
    Window() = default;

    void init(const std::string &title, ivec2 resolution) {
        SDL_Init(SDL_INIT_VIDEO);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                            SDL_GL_CONTEXT_PROFILE_CORE);
        window = SDL_CreateWindow(title.c_str(), resolution.x, resolution.y,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        SDL_GL_SetSwapInterval(1);
    }

    void setContext() {
        glContext = SDL_GL_CreateContext(window);
        SDL_GL_MakeCurrent(window, glContext);
        gladLoadGL(SDL_GL_GetProcAddress);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    void swapBuffer() { SDL_GL_SwapWindow(window); }

    void shutdown() {
        if (glContext) {
            SDL_GL_DestroyContext(glContext);
            glContext = nullptr;
        }
        if (window) {
            SDL_DestroyWindow(window);
            window = nullptr;
        }
        SDL_Quit();
    }

    /*void Engine::renderFPS() {
        ++frameCount;
        if (renderTime >= 1.0f) {
            std::string fps = "Engine " + std::to_string(frameCount /
    renderTime); SDL_SetWindowTitle(window->getWindow(), fps.c_str());
            renderTime -= 1.0f;
            frameCount = 0;
        }
    }*/
};
