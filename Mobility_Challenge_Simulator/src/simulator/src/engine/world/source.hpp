#pragma once

#include "pch.hpp"

class Source {
  private:
    std::unordered_map<size_t, std::unique_ptr<Resource>> resources;
    std::unordered_map<size_t, std::unique_ptr<Resource>> *readSource = nullptr;
    std::unordered_map<size_t, std::unique_ptr<Resource>> *writeSource =
        nullptr;

  public:
    Source() = default;

    void init() {
        // readSource = &resources[0];
        // writeSource = &resources[1];
    }

    template <typename T> void add() {
        static_assert(std::is_base_of_v<Resource, T>,
                      "T must inherit from Resource");
        size_t type = getStructHash<T>();
        resources[type] = std::make_unique<T>();
    }

    template <typename T> T &get() {
        static_assert(std::is_base_of_v<Resource, T>,
                      "T must inherit from Resource");
        size_t type = getStructHash<T>();
        auto it = resources.find(type);
        if (it == resources.end()) {
            add<T>();
            return static_cast<T &>(*resources[type]);
        }
        return static_cast<T &>(*it->second);
    }

    void swapBuffer() { std::swap(readSource, writeSource); }
};