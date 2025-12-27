// ECS-style storage: generational entity ids and one sparse set per component
// type. Iteration over a view walks the smallest participating set.
#pragma once
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

namespace ifg {

using Entity = std::uint32_t;  // 20-bit index | 12-bit generation
constexpr Entity kNullEntity = 0xFFFFFFFFu;
inline std::uint32_t entity_index(Entity e) { return e & 0xFFFFFu; }
inline std::uint32_t entity_gen(Entity e) { return e >> 20; }

class SparseSetBase {
public:
    virtual ~SparseSetBase() = default;
    virtual void remove(Entity e) = 0;
    bool contains(Entity e) const {
        std::uint32_t i = entity_index(e);
        return i < sparse_.size() && sparse_[i] != kAbsent && dense_[sparse_[i]] == e;
    }
    std::size_t size() const { return dense_.size(); }
    const std::vector<Entity>& entities() const { return dense_; }

protected:
    static constexpr std::uint32_t kAbsent = 0xFFFFFFFFu;
    std::vector<std::uint32_t> sparse_;  // index -> dense slot
    std::vector<Entity> dense_;
};

template <class T>
class SparseSet final : public SparseSetBase {
public:
    template <class... A>
    T& emplace(Entity e, A&&... a) {
        assert(!contains(e));
        std::uint32_t i = entity_index(e);
        if (i >= sparse_.size()) sparse_.resize(i + 1, kAbsent);
        sparse_[i] = static_cast<std::uint32_t>(dense_.size());
        dense_.push_back(e);
        data_.push_back(T{std::forward<A>(a)...});
        return data_.back();
    }
    void remove(Entity e) override {
        if (!contains(e)) return;
        std::uint32_t slot = sparse_[entity_index(e)];
        std::uint32_t last = static_cast<std::uint32_t>(dense_.size() - 1);
        if (slot != last) {  // swap-and-pop keeps the arrays packed
            dense_[slot] = dense_[last];
            data_[slot] = std::move(data_[last]);
            sparse_[entity_index(dense_[slot])] = slot;
        }
        dense_.pop_back();
        data_.pop_back();
        sparse_[entity_index(e)] = kAbsent;
    }
    T& get(Entity e) {
        assert(contains(e));
        return data_[sparse_[entity_index(e)]];
    }
    T* try_get(Entity e) { return contains(e) ? &data_[sparse_[entity_index(e)]] : nullptr; }
    std::vector<T>& data() { return data_; }

private:
    std::vector<T> data_;
};

class Registry {
public:
    Entity create() {
        if (!free_.empty()) {
            std::uint32_t i = free_.back();
            free_.pop_back();
            dead_[i] = 0;
            return (gens_[i] << 20) | i;
        }
        std::uint32_t i = static_cast<std::uint32_t>(gens_.size());
        assert(i < (1u << 20));
        gens_.push_back(0);
        dead_.push_back(0);
        return i;
    }
    bool alive(Entity e) const {
        std::uint32_t i = entity_index(e);
        return i < gens_.size() && gens_[i] == entity_gen(e) && !dead_[i];
    }
    void destroy(Entity e) {
        if (!alive(e)) return;
        for (auto& p : pools_)
            if (p) p->remove(e);
        std::uint32_t i = entity_index(e);
        gens_[i] = (gens_[i] + 1) & 0xFFFu;
        dead_[i] = 1;
        free_.push_back(i);
    }
    std::size_t alive_count() const { return gens_.size() - free_.size(); }

    template <class T, class... A>
    T& emplace(Entity e, A&&... a) { return pool<T>().emplace(e, std::forward<A>(a)...); }
    template <class T>
    void remove(Entity e) { pool<T>().remove(e); }
    template <class T>
    bool has(Entity e) { return pool<T>().contains(e); }
    template <class T>
    T& get(Entity e) { return pool<T>().get(e); }
    template <class T>
    T* try_get(Entity e) { return pool<T>().try_get(e); }

    // Calls fn(entity, A&, B&...) for every entity that has all components.
    template <class First, class... Rest, class Fn>
    void each(Fn&& fn) {
        SparseSetBase* sets[] = {&pool<First>(), &pool<Rest>()...};
        SparseSetBase* smallest = sets[0];
        for (auto* s : sets)
            if (s->size() < smallest->size()) smallest = s;
        // copy: fn may add/remove components of other types
        std::vector<Entity> ents = smallest->entities();
        for (Entity e : ents)
            if (pool<First>().contains(e) && (pool<Rest>().contains(e) && ...))
                fn(e, pool<First>().get(e), pool<Rest>().get(e)...);
    }

    template <class T>
    SparseSet<T>& pool() {
        std::size_t id = type_id<T>();
        if (id >= pools_.size()) pools_.resize(id + 1);
        if (!pools_[id]) pools_[id] = std::make_unique<SparseSet<T>>();
        return static_cast<SparseSet<T>&>(*pools_[id]);
    }

private:
    static std::size_t next_type_id() {
        static std::size_t counter = 0;
        return counter++;
    }
    template <class T>
    static std::size_t type_id() {
        static const std::size_t id = next_type_id();
        return id;
    }
    std::vector<std::uint32_t> gens_;
    std::vector<std::uint32_t> free_;
    std::vector<std::uint8_t> dead_;
    std::vector<std::unique_ptr<SparseSetBase>> pools_;
};

}  // namespace ifg
