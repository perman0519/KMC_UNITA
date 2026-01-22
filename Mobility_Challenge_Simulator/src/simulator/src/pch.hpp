#pragma once
#ifdef _WIN32
#define NOMINMAX
#endif
#include <glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

#include <any>
#include <map>
#include <queue>
#include <deque>
#include <memory>
#include <array>
#include <tuple>
#include <bitset>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <cmath>
#include <limits>
#include <chrono>
#include <random>
#include <cstdint>
#include <typeindex>
#include <algorithm>
#include <functional>

#include <mutex>
#include <thread>
#include <shared_mutex>
#include <condition_variable>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <SDL3/SDL.h>

using real = float;
using vec2 = glm::vec2;
using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3 = glm::mat3;
using mat4 = glm::mat4;
using quat = glm::quat;
using ivec2 = glm::ivec2;

using EntityID = uint64_t;
constexpr EntityID GLOBAL_ENTITY = 0;
static constexpr EntityID NULL_ENTITY = std::numeric_limits<EntityID>::max();
constexpr size_t MAX_ENTITIES = NULL_ENTITY;
constexpr size_t MAX_COMPONENTS = 64;

template <typename T>
constexpr size_t getStructHash() {
#if defined(_MSC_VER)
    constexpr const char *name = __FUNCSIG__;
#elif defined(__clang__) || defined(__GNUC__)
    constexpr const char *name = __PRETTY_FUNCTION__;
#else
#error "Unsupported compiler"
#endif

    // FNV-1a 64-bit hash
    size_t hash = 14695981039346656037ULL;
    for (const char *p = name; *p; ++p) {
        hash ^= static_cast<size_t>(*p);
        hash *= 1099511628211ULL;
    }
    return hash;
}

#ifdef DEBUG
#define RESOURCE_PATH(path) "../../" path
#else
#define RESOURCE_PATH(path) path
#endif

#ifdef USE_ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>

inline std::string getResourceBasePath() {
    try {
        return ament_index_cpp::get_package_share_directory("simulator") + "/resource/";
    } catch (...) {
        // Fallback for development builds
        return "resource/";
    }
}

#define RESOURCE_PATH(x) (getResourceBasePath() + std::string(x)).c_str()

const std::string PATH_CONFIG = getResourceBasePath();
const std::string PATH_FONT = getResourceBasePath() + "font/";
const std::string PATH_SHADER = getResourceBasePath() + "shader/";
const std::string PATH_ASSETS = getResourceBasePath() + "assets/";

#else
// Non-ROS2 builds use relative paths
constexpr const char *PATH_CONFIG = "resource/";
constexpr const char *PATH_FONT = "resource/font/";
constexpr const char *PATH_SHADER = "resource/shader/";
constexpr const char *PATH_ASSETS = "resource/assets/";
#endif

using EventType = Uint32;
struct Event {
    virtual ~Event() = default;
};

struct Resource {
    virtual ~Resource() = default;
};

// world setting

const vec3 PLATFORM_AV = vec3(-1.0f, 1.0f, 0.05f);
const vec3 PLATFORM_HV = vec3(-1.0f, -1.0f, 0.05f);
const vec3 PLATFORM_OFFSET = vec3(0.0f, 0.0f, 0.05f);
