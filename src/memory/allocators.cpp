#include "memory/allocators.hpp"

#include <cassert>
#include <cstdlib>
#include <new>
#include <sstream>

namespace ifg {

static std::byte* aligned_block(std::size_t size, std::size_t align) {
    size = align_up(size ? size : 1, align);
    void* p = std::aligned_alloc(align, size);
    if (!p) throw std::bad_alloc();
    return static_cast<std::byte*>(p);
}

// ---------------------------------------------------------------- Pool
PoolAllocator::PoolAllocator(std::size_t block_size, std::size_t block_count, std::size_t align)
    : block_size_(align_up(std::max(block_size, sizeof(Node)), align)), count_(block_count), align_(align) {
    assert(is_pow2(align));
    mem_ = aligned_block(block_size_ * count_, align_);
    // thread the free list in address order
    for (std::size_t i = count_; i-- > 0;) {
        Node* n = reinterpret_cast<Node*>(mem_ + i * block_size_);
        n->next = free_;
        free_ = n;
    }
}
PoolAllocator::~PoolAllocator() { std::free(mem_); }

void* PoolAllocator::allocate(std::size_t size, std::size_t align) {
    if (size > block_size_ || align > align_ || !free_) return nullptr;
    Node* n = free_;
    free_ = n->next;
    ++used_;
    return n;
}

void PoolAllocator::deallocate(void* p) {
    if (!p) return;
    assert(owns(p));
    Node* n = static_cast<Node*>(p);
    n->next = free_;
    free_ = n;
    --used_;
}

bool PoolAllocator::owns(const void* p) const {
    auto b = static_cast<const std::byte*>(p);
    if (b < mem_ || b >= mem_ + block_size_ * count_) return false;
    return static_cast<std::size_t>(b - mem_) % block_size_ == 0;
}

// ---------------------------------------------------------------- Frame
FrameAllocator::FrameAllocator(std::size_t capacity) : mem_(aligned_block(capacity, 64)), cap_(capacity) {}
FrameAllocator::~FrameAllocator() { std::free(mem_); }

void* FrameAllocator::allocate(std::size_t size, std::size_t align) {
    assert(is_pow2(align));
    auto base = reinterpret_cast<std::uintptr_t>(mem_);
    std::size_t start = align_up(base + offset_, align) - base;
    if (start + size > cap_) return nullptr;
    offset_ = start + size;
    peak_ = std::max(peak_, offset_);
    return mem_ + start;
}

// ---------------------------------------------------------------- FreeList
FreeListAllocator::FreeListAllocator(std::size_t capacity, Policy policy)
    : mem_(aligned_block(align_up(capacity, kGranule), kGranule)), cap_(align_up(capacity, kGranule)), policy_(policy) {
    head_ = reinterpret_cast<FreeNode*>(mem_);
    head_->size = cap_;
    head_->next = nullptr;
}
FreeListAllocator::~FreeListAllocator() { std::free(mem_); }

void* FreeListAllocator::allocate(std::size_t size, std::size_t align) {
    if (size == 0) size = 1;
    if (align < alignof(Header)) align = alignof(Header);
    assert(is_pow2(align));
    FreeNode *best = nullptr, *best_prev = nullptr;
    std::size_t best_need = 0, best_pad = 0;
    FreeNode* prev = nullptr;
    for (FreeNode* n = head_; n; prev = n, n = n->next) {
        auto addr = reinterpret_cast<std::uintptr_t>(n);
        std::size_t user = align_up(addr + sizeof(Header), align);
        std::size_t pad = user - addr;
        std::size_t need = align_up(pad + size, kGranule);
        if (n->size < need) continue;
        if (!best || (policy_ == Policy::BestFit && n->size < best->size)) {
            best = n;
            best_prev = prev;
            best_need = need;
            best_pad = pad;
            if (policy_ == Policy::FirstFit) break;
        }
    }
    if (!best) return nullptr;
    std::size_t block = best->size;
    FreeNode* next = best->next;
    if (block - best_need >= sizeof(FreeNode) + kGranule) {
        // split: remainder becomes a new free node
        auto rem = reinterpret_cast<FreeNode*>(reinterpret_cast<std::byte*>(best) + best_need);
        rem->size = block - best_need;
        rem->next = next;
        next = rem;
        block = best_need;
    }
    if (best_prev) best_prev->next = next; else head_ = next;
    std::byte* user = reinterpret_cast<std::byte*>(best) + best_pad;
    Header* h = reinterpret_cast<Header*>(user - sizeof(Header));
    h->block_size = block;
    h->padding = best_pad;
    used_ += block;
    ++live_;
    return user;
}

void FreeListAllocator::deallocate(void* p) {
    if (!p) return;
    std::byte* user = static_cast<std::byte*>(p);
    assert(user > mem_ && user < mem_ + cap_);
    Header* h = reinterpret_cast<Header*>(user - sizeof(Header));
    std::size_t block = h->block_size;
    auto node = reinterpret_cast<FreeNode*>(user - h->padding);
    node->size = block;
    used_ -= block;
    --live_;
    // insert in address order
    FreeNode* prev = nullptr;
    FreeNode* cur = head_;
    while (cur && cur < node) { prev = cur; cur = cur->next; }
    node->next = cur;
    if (prev) prev->next = node; else head_ = node;
    // coalesce with next
    if (cur && reinterpret_cast<std::byte*>(node) + node->size == reinterpret_cast<std::byte*>(cur)) {
        node->size += cur->size;
        node->next = cur->next;
    }
    // coalesce with prev
    if (prev && reinterpret_cast<std::byte*>(prev) + prev->size == reinterpret_cast<std::byte*>(node)) {
        prev->size += node->size;
        prev->next = node->next;
    }
}

FreeListAllocator::Stats FreeListAllocator::stats() const {
    Stats s{cap_, used_, 0, 0, 0, live_, 0.0};
    for (FreeNode* n = head_; n; n = n->next) {
        s.free_bytes += n->size;
        s.largest_free = std::max(s.largest_free, n->size);
        ++s.free_blocks;
    }
    s.fragmentation = s.free_bytes ? 1.0 - double(s.largest_free) / double(s.free_bytes) : 0.0;
    return s;
}

bool FreeListAllocator::validate() const {
    std::size_t free_total = 0;
    for (FreeNode* n = head_; n; n = n->next) {
        auto b = reinterpret_cast<std::byte*>(n);
        if (b < mem_ || b + n->size > mem_ + cap_) return false;
        if (n->size % kGranule) return false;
        if (n->next) {
            // ordered and never adjacent (adjacent holes must have been coalesced)
            if (reinterpret_cast<std::byte*>(n->next) <= b + n->size - 1) return false;
            if (b + n->size == reinterpret_cast<std::byte*>(n->next)) return false;
        }
        free_total += n->size;
    }
    return free_total + used_ == cap_;
}

// ---------------------------------------------------------------- Tracking
void* TrackingAllocator::allocate(std::size_t size, std::size_t align) { return allocate_tagged(size, "untagged", align); }

void* TrackingAllocator::allocate_tagged(std::size_t size, const char* tag, std::size_t align) {
    void* p = inner_.allocate(size, align);
    if (!p) return nullptr;
    live_[p] = {size, tag};
    live_bytes_ += size;
    peak_bytes_ = std::max(peak_bytes_, live_bytes_);
    ++total_;
    return p;
}

void TrackingAllocator::deallocate(void* p) {
    if (!p) return;
    auto it = live_.find(p);
    if (it == live_.end()) {  // double free or foreign pointer: refuse to forward it
        ++bad_frees_;
        return;
    }
    live_bytes_ -= it->second.size;
    live_.erase(it);
    inner_.deallocate(p);
}

std::string TrackingAllocator::leak_report() const {
    std::ostringstream os;
    os << live_.size() << " live allocation(s), " << live_bytes_ << " bytes\n";
    std::unordered_map<std::string, std::pair<std::size_t, std::size_t>> by_tag;
    for (auto& [p, r] : live_) {
        auto& e = by_tag[r.tag];
        e.first++;
        e.second += r.size;
    }
    for (auto& [tag, e] : by_tag) os << "  " << tag << ": " << e.first << " allocs, " << e.second << " bytes\n";
    return os.str();
}

}  // namespace ifg
