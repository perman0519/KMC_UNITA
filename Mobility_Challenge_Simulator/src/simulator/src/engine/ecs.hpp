#pragma once

#include "pch.hpp"

class ISparseSet {
public:
    virtual ~ISparseSet() = default;
    virtual void clear() = 0;
    virtual void remove(EntityID id) = 0;
    virtual size_t size() = 0;
    virtual bool contains(EntityID id) = 0;
    virtual std::vector<EntityID> getEntityList() = 0;
};

template <typename T>
class SparseSet : public ISparseSet {
private:
    using Sparse = std::vector<size_t>;
    std::vector<Sparse> pages;
    std::vector<T> dense;
    std::vector<EntityID> denseToEntity;
    const size_t MAX_SPARSE_SIZE = 1000;
    static constexpr size_t empty = std::numeric_limits<size_t>::max();

    void setDenseIndex(EntityID id, size_t index) {
        size_t page = id / MAX_SPARSE_SIZE;
        size_t sparseIndex = id % MAX_SPARSE_SIZE;
        if (page >= pages.size())
            pages.resize(page + 1);
        Sparse &sparse = pages[page];
        if (sparseIndex >= sparse.size())
            sparse.resize(sparseIndex + 1, empty);
        sparse[sparseIndex] = index;
    }

    size_t getDenseIndex(EntityID id) {
        size_t page = id / MAX_SPARSE_SIZE;
        size_t sparseIndex = id % MAX_SPARSE_SIZE;
        if (page < pages.size()) {
            Sparse &sparse = pages[page];
            if (sparseIndex < sparse.size())
                return sparse[sparseIndex];
        }
        return empty;
    }

public:
    SparseSet() { dense.reserve(MAX_SPARSE_SIZE); }

    void set(EntityID id, T obj) {
        size_t index = getDenseIndex(id);
        if (index == empty) {
            setDenseIndex(id, dense.size());
            dense.push_back(obj);
            denseToEntity.push_back(id);
            return;
        }
        dense[index] = obj;
        denseToEntity[index] = id;
    }

    T &get(EntityID id) {
        size_t index = getDenseIndex(id);
        return dense[index];
    }

    void remove(EntityID id) override {
        size_t deletedIndex = getDenseIndex(id);
        if (dense.empty() || deletedIndex == empty)
            return;
        setDenseIndex(denseToEntity.back(), deletedIndex);
        setDenseIndex(id, empty);
        std::swap(dense.back(), dense[deletedIndex]);
        std::swap(denseToEntity.back(), denseToEntity[deletedIndex]);
        dense.pop_back();
        denseToEntity.pop_back();
    }

    size_t size() override { return dense.size(); }

    std::vector<EntityID> getEntityList() override { return denseToEntity; }

    bool contains(EntityID id) override { return getDenseIndex(id) != empty; }

    void clear() override {
        dense.clear();
        pages.clear();
        denseToEntity.clear();
    }
};

class IComponentBuffer {
public:
    virtual ~IComponentBuffer() = default;
    virtual void remove(EntityID) = 0;
    virtual void swapBuffer() noexcept = 0;
    virtual void unlockRead() const = 0;
};

template <typename T>
class ComponentBuffer : public IComponentBuffer {
private:
    SparseSet<T> buffer[2];
    SparseSet<T> *read = nullptr;
    SparseSet<T> *write = nullptr;
    mutable std::shared_mutex mutex;

    std::unordered_set<EntityID> changed;

public:
    ComponentBuffer() {
        read = &buffer[0];
        write = &buffer[1];
    }

    void markChanged(EntityID id) { changed.insert(id); }

    SparseSet<T> *writeBuffer() { return write; }

    SparseSet<T> *readBuffer() const {
        mutex.lock_shared();
        return read;
    }

    void unlockRead() const override { mutex.unlock_shared(); }

    void swapBuffer() noexcept override {
        std::unique_lock lock(mutex);
        for (EntityID id : changed) {
            if (write->contains(id)) {
                T &value = write->get(id);
                if (!read->contains(id)) {
                    read->set(id, value);
                } else {
                    read->get(id) = value;
                }
            } else {
                read->remove(id);
            }
        }
        changed.clear();
        std::swap(read, write);
    }

    void remove(EntityID id) override {
        write->remove(id);
        markChanged(id);
    }
};

template <class... Types>
struct type_list {
    using type_tuple = std::tuple<Types...>;
    template <size_t Index>
    using get = std::tuple_element_t<Index, type_tuple>;
    static constexpr size_t size = sizeof...(Types);
};

template <typename... Components>
class View {
private:
    using componentTypes = type_list<Components...>;
    std::array<ISparseSet *, sizeof...(Components)> viewPools;
    ISparseSet *smallest = nullptr;
    std::array<const IComponentBuffer *, sizeof...(Components)> lockedBuffers;

    bool allContain(EntityID id) {
        return std::all_of(
            viewPools.begin(), viewPools.end(),
            [id](ISparseSet *pool) { return pool->contains(id); });
    }

    template <size_t Index>
    auto getPool() {
        using componentType = typename componentTypes::template get<Index>;
        return static_cast<SparseSet<componentType> *>(viewPools[Index]);
    }

    template <size_t... Indices>
    auto makeTuple(EntityID id, std::index_sequence<Indices...>) {
        return std::make_tuple((std::ref(getPool<Indices>()->get(id)))...);
    }

    void init() {
        auto smallestPool =
            std::min_element(viewPools.begin(), viewPools.end(),
                             [](ISparseSet *poolA, ISparseSet *poolB) {
                                 return poolA->size() < poolB->size();
                             });
        smallest = *smallestPool;
    }

public:
    View(std::array<ISparseSet *, sizeof...(Components)> pools)
        : viewPools{pools}, lockedBuffers{} {
        init();
    }

    View(std::array<std::pair<ISparseSet *, const IComponentBuffer *>,
                    sizeof...(Components)>
             pairs) {
        for (size_t i = 0; i < sizeof...(Components); ++i) {
            viewPools[i] = pairs[i].first;
            lockedBuffers[i] = pairs[i].second;
        }
        init();
    }

    template <typename Func>
    void iterate(Func &&func) {
        std::vector<EntityID> dirtyEntities;
        auto inds = std::make_index_sequence<sizeof...(Components)>{};
        for (EntityID id : smallest->getEntityList()) {
            if (std::all_of(
                    viewPools.begin(), viewPools.end(),
                    [id](ISparseSet *pool) { return pool->contains(id); })) {
                std::apply(func, std::tuple_cat(std::make_tuple(id),
                                                makeTuple(id, inds)));
            }
        }
        for (auto *buffer : lockedBuffers) {
            if (buffer)
                buffer->unlockRead();
        }
    }

    std::vector<EntityID> getEntities() {
        std::vector<EntityID> result;
        for (EntityID id : smallest->getEntityList()) {
            if (allContain(id))
                result.push_back(id);
        }
        return result;
    }
};

class ECS {
private:
    using ComponentMask = std::bitset<MAX_COMPONENTS>;
    std::vector<std::unique_ptr<IComponentBuffer>> componentBuffers;
    std::unordered_map<std::type_index, size_t> componentBit;
    std::vector<EntityID> availableEntities;
    SparseSet<ComponentMask> entityMasks;
    EntityID maxEntityID = 0;
    static constexpr size_t empty = std::numeric_limits<size_t>::max();

    template <typename... Components>
    friend class View;

    template <typename T>
    void registerComponent() {
        addComponent<T>();
        componentBuffers.push_back(std::make_unique<ComponentBuffer<T>>());
    }

    template <typename T>
    void addComponent() {
        std::type_index type = std::type_index(typeid(T));
        componentBit[type] = componentBuffers.size();
    }

    template <typename Component>
    void setComponentBit(ComponentMask &mask, bool val) {
        size_t bitPos = getComponentBit<Component>();
        mask[bitPos] = val;
    }

    template <typename T>
    size_t getComponentBit() {
        std::type_index type = std::type_index(typeid(T));
        auto it = componentBit.find(type);
        if (it == componentBit.end())
            return empty;
        return it->second;
    }

    template <typename T>
    SparseSet<T> &getReadBuffer() {
        ISparseSet *genericPtr = getReadBufferPtr<T>();
        SparseSet<T> *buffer = static_cast<SparseSet<T> *>(genericPtr);
        return *buffer;
    }

    template <typename T>
    SparseSet<T> &getWriteBuffer() {
        ISparseSet *genericPtr = getWriteBufferPtr<T>();
        SparseSet<T> *buffer = static_cast<SparseSet<T> *>(genericPtr);
        return *buffer;
    }

    template <typename T>
    std::pair<ISparseSet *, const ComponentBuffer<T> *> getReadBufferPtr() {
        size_t bitPos = getComponentBit<T>();
        if (bitPos == empty) {
            registerComponent<T>();
            bitPos = getComponentBit<T>();
        }
        auto *buffer =
            static_cast<ComponentBuffer<T> *>(componentBuffers[bitPos].get());
        auto *sparse = buffer->readBuffer();
        return {sparse, buffer};
    }

    template <typename T>
    ISparseSet *getWriteBufferPtr() {
        size_t bitPos = getComponentBit<T>();
        if (bitPos == empty) {
            registerComponent<T>();
            bitPos = getComponentBit<T>();
        }
        IComponentBuffer *genericPtr = componentBuffers[bitPos].get();
        ComponentBuffer<T> *buffer =
            static_cast<ComponentBuffer<T> *>(genericPtr);
        return buffer->writeBuffer();
    }

    template <typename Component>
    ComponentMask::reference getComponentBit(ComponentMask &mask) {
        size_t bitPos = getComponentBit<Component>();
        return mask[bitPos];
    }

    ComponentMask &getEntityMask(EntityID id) {
        ComponentMask &mask = entityMasks.get(id);
        return mask;
    }

public:
    ECS() = default;

    void init() { reset(); }

    void reset() {
        entityMasks.clear();
        componentBit.clear();
        componentBuffers.clear();
        availableEntities.clear();
        maxEntityID = 0;
    }

    EntityID createEntity() {
        EntityID id = empty;
        if (availableEntities.size() == 0) {
            id = maxEntityID++;
        } else {
            id = availableEntities.back();
            availableEntities.pop_back();
        }
        entityMasks.set(id, {});
        return id;
    }

    void removeEntity(EntityID &id) {
        ComponentMask &mask = getEntityMask(id);
        for (int bit = 0; bit < MAX_COMPONENTS; bit++)
            if (mask[bit] == 1) {
                componentBuffers[bit]->remove(id);
            }
        entityMasks.remove(id);
        availableEntities.push_back(id);
        id = NULL_ENTITY;
    }

    template <typename T>
    void add(EntityID id, T &&component = {}) {
        size_t bitPos = getComponentBit<T>();
        if (bitPos == empty) {
            registerComponent<T>();
            bitPos = getComponentBit<T>();
        }
        auto *componentBuffer =
            static_cast<ComponentBuffer<T> *>(componentBuffers[bitPos].get());
        componentBuffer->markChanged(id);

        SparseSet<T> &buffer = getWriteBuffer<T>();
        ComponentMask &mask = getEntityMask(id);
        setComponentBit<T>(mask, 1);
        buffer.set(id, std::move(component));
    }

    template <typename T>
    T &get(EntityID id) {
        SparseSet<T> &buffer = getWriteBuffer<T>();
        T &component = buffer.get(id);
        return component;
    }

    template <typename T>
    void remove(EntityID id) {
        size_t bitPos = getComponentBit<T>();
        auto *componentBuffer =
            static_cast<ComponentBuffer<T> *>(componentBuffers[bitPos].get());
        componentBuffer->markChanged(id);

        SparseSet<T> &buffer = getWriteBuffer<T>();
        ComponentMask &mask = getEntityMask(id);
        setComponentBit<T>(mask, 0);
        buffer.remove(id);
    }

    template <typename T>
    void markChanged(EntityID id) {
        size_t bitPos = getComponentBit<T>();
        auto *componentBuffer =
            static_cast<ComponentBuffer<T> *>(componentBuffers[bitPos].get());
        componentBuffer->markChanged(id);
    }

    template <typename... Components>
    View<Components...> write() {
        return {{getWriteBufferPtr<Components>()...}};
    }

    template <typename... Components>
    View<Components...> read() {
        std::array<std::pair<ISparseSet *, const IComponentBuffer *>,
                   sizeof...(Components)>
            pairs = {getReadBufferPtr<Components>()...};
        return View<Components...>(pairs);
    }

    size_t getEntityCount() { return entityMasks.size(); }

    size_t getPoolCount() { return componentBuffers.size(); }

    void swapBuffers() {
        for (auto &buffer : componentBuffers) {
            buffer->swapBuffer();
        }
    }
};
