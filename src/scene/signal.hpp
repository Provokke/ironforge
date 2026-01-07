// Minimal typed signal/slot. Emission is safe against connect/disconnect
// from inside a slot: slots connected during emit are not called this round,
// slots disconnected during emit are skipped.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace ifg {

template <class... Args>
class Signal {
public:
    using Slot = std::function<void(Args...)>;
    using Id = std::uint64_t;

    Id connect(Slot s) {
        Id id = ++next_;
        slots_.push_back(std::make_shared<Entry>(Entry{id, std::move(s), true}));
        return id;
    }
    bool disconnect(Id id) {
        for (std::size_t i = 0; i < slots_.size(); ++i)
            if (slots_[i]->id == id) {
                slots_[i]->alive = false;
                slots_.erase(slots_.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        return false;
    }
    void emit(Args... args) const {
        auto snapshot = slots_;
        for (auto& e : snapshot)
            if (e->alive) e->fn(args...);
    }
    std::size_t size() const { return slots_.size(); }
    void clear() {
        for (auto& e : slots_) e->alive = false;
        slots_.clear();
    }

private:
    struct Entry { Id id; Slot fn; bool alive; };
    std::vector<std::shared_ptr<Entry>> slots_;
    Id next_ = 0;
};

}  // namespace ifg
