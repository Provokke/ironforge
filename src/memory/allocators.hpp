// IronForge memory: pool, frame (linear) and free-list allocators plus an
// allocation tracker. None of these are thread-safe; use one per thread.
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ifg {

inline std::size_t align_up(std::size_t v, std::size_t a) { return (v + a - 1) & ~(a - 1); }
inline bool is_pow2(std::size_t v) { return v && !(v & (v - 1)); }

class Allocator {
public:
    virtual ~Allocator() = default;
    virtual void* allocate(std::size_t size, std::size_t align = alignof(std::max_align_t)) = 0;
    virtual void deallocate(void* p) = 0;
};

// Fixed-size blocks, O(1) alloc/free through an intrusive free list.
class PoolAllocator final : public Allocator {
public:
    PoolAllocator(std::size_t block_size, std::size_t block_count, std::size_t align = alignof(std::max_align_t));
    ~PoolAllocator() override;
    PoolAllocator(const PoolAllocator&) = delete;
    PoolAllocator& operator=(const PoolAllocator&) = delete;

    void* allocate(std::size_t size, std::size_t align = alignof(std::max_align_t)) override;
    void deallocate(void* p) override;
    bool owns(const void* p) const;
    std::size_t block_size() const { return block_size_; }
    std::size_t capacity() const { return count_; }
    std::size_t used() const { return used_; }

private:
    struct Node { Node* next; };
    std::byte* mem_ = nullptr;
    std::size_t block_size_, count_, align_, used_ = 0;
    Node* free_ = nullptr;
};

// Bump-pointer allocator reset once per frame. Supports markers for scoped rewinds.
class FrameAllocator final : public Allocator {
public:
    explicit FrameAllocator(std::size_t capacity);
    ~FrameAllocator() override;
    FrameAllocator(const FrameAllocator&) = delete;
    FrameAllocator& operator=(const FrameAllocator&) = delete;

    void* allocate(std::size_t size, std::size_t align = alignof(std::max_align_t)) override;
    void deallocate(void*) override {}  // freed wholesale by reset()
    template <class T, class... A>
    T* make(A&&... a) {
        void* p = allocate(sizeof(T), alignof(T));
        return p ? new (p) T(static_cast<A&&>(a)...) : nullptr;
    }
    std::size_t marker() const { return offset_; }
    void rewind(std::size_t m) { offset_ = m; }
    void reset() { offset_ = 0; }
    std::size_t used() const { return offset_; }
    std::size_t capacity() const { return cap_; }
    std::size_t peak() const { return peak_; }

private:
    std::byte* mem_;
    std::size_t cap_, offset_ = 0, peak_ = 0;
};

// General-purpose allocator over one arena: address-ordered free list,
// first-fit or best-fit, splitting and immediate coalescing.
class FreeListAllocator final : public Allocator {
public:
    enum class Policy { FirstFit, BestFit };
    struct Stats {
        std::size_t capacity, used_bytes, free_bytes, largest_free, free_blocks, live_allocations;
        double fragmentation;  // 1 - largest_free / free_bytes (0 = one contiguous hole)
    };
    explicit FreeListAllocator(std::size_t capacity, Policy policy = Policy::FirstFit);
    ~FreeListAllocator() override;
    FreeListAllocator(const FreeListAllocator&) = delete;
    FreeListAllocator& operator=(const FreeListAllocator&) = delete;

    void* allocate(std::size_t size, std::size_t align = alignof(std::max_align_t)) override;
    void deallocate(void* p) override;
    Stats stats() const;
    bool validate() const;  // internal consistency check used by tests

private:
    struct FreeNode { std::size_t size; FreeNode* next; };
    struct Header { std::size_t block_size; std::size_t padding; };
    static constexpr std::size_t kGranule = 16;
    std::byte* mem_;
    std::size_t cap_;
    Policy policy_;
    FreeNode* head_ = nullptr;
    std::size_t used_ = 0, live_ = 0;
};

// Records every allocation passing through it; reports leaks and peaks.
class TrackingAllocator final : public Allocator {
public:
    explicit TrackingAllocator(Allocator& inner) : inner_(inner) {}
    void* allocate(std::size_t size, std::size_t align = alignof(std::max_align_t)) override;
    void deallocate(void* p) override;
    void* allocate_tagged(std::size_t size, const char* tag, std::size_t align = alignof(std::max_align_t));
    std::size_t live_count() const { return live_.size(); }
    std::size_t live_bytes() const { return live_bytes_; }
    std::size_t peak_bytes() const { return peak_bytes_; }
    std::size_t total_allocations() const { return total_; }
    std::size_t double_frees() const { return bad_frees_; }
    std::string leak_report() const;

private:
    struct Rec { std::size_t size; const char* tag; };
    Allocator& inner_;
    std::unordered_map<void*, Rec> live_;
    std::size_t live_bytes_ = 0, peak_bytes_ = 0, total_ = 0, bad_frees_ = 0;
};

}  // namespace ifg
