#include <cstring>
#include <random>
#include <set>

#include "doctest/doctest.h"
#include "memory/allocators.hpp"

using namespace ifg;

TEST_CASE("pool allocator: exhaust, reuse, alignment, ownership") {
    PoolAllocator pool(24, 100, 16);
    CHECK(pool.block_size() == 32);
    std::vector<void*> ptrs;
    for (int i = 0; i < 100; ++i) {
        void* p = pool.allocate(24, 16);
        REQUIRE(p);
        CHECK(reinterpret_cast<std::uintptr_t>(p) % 16 == 0);
        CHECK(pool.owns(p));
        std::memset(p, i, 24);
        ptrs.push_back(p);
    }
    CHECK(pool.allocate(24) == nullptr);  // exhausted
    CHECK(pool.used() == 100);
    CHECK(std::set<void*>(ptrs.begin(), ptrs.end()).size() == 100);
    for (int i = 0; i < 100; ++i) CHECK(static_cast<unsigned char*>(ptrs[i])[23] == static_cast<unsigned char>(i));
    CHECK(pool.allocate(64) == nullptr);  // too large
    int local = 0;
    CHECK_FALSE(pool.owns(&local));
    CHECK_FALSE(pool.owns(static_cast<char*>(ptrs[0]) + 1));  // interior pointer
    pool.deallocate(ptrs[50]);
    CHECK(pool.allocate(8) == ptrs[50]);  // LIFO reuse
    for (void* p : ptrs) pool.deallocate(p);
    CHECK(pool.used() == 0);
}

TEST_CASE("frame allocator: alignment, markers, reset") {
    FrameAllocator fa(1024);
    void* a = fa.allocate(3, 1);
    void* b = fa.allocate(8, 8);
    CHECK(reinterpret_cast<std::uintptr_t>(b) % 8 == 0);
    CHECK(b > a);
    auto m = fa.marker();
    void* c = fa.allocate(100, 64);
    CHECK(reinterpret_cast<std::uintptr_t>(c) % 64 == 0);
    fa.rewind(m);
    CHECK(fa.allocate(100, 64) == c);  // rewinding reuses the same memory
    struct P { int x, y; };
    P* p = fa.make<P>(P{3, 4});
    CHECK(p->y == 4);
    CHECK(fa.allocate(4096) == nullptr);  // over capacity
    std::size_t peak = fa.peak();
    fa.reset();
    CHECK(fa.used() == 0);
    CHECK(fa.peak() == peak);
    CHECK(fa.allocate(3, 1) == a);
}

TEST_CASE("free-list allocator: coalescing returns to one block") {
    for (auto policy : {FreeListAllocator::Policy::FirstFit, FreeListAllocator::Policy::BestFit}) {
        FreeListAllocator fl(64 * 1024, policy);
        std::vector<void*> v;
        for (int i = 0;; ++i) {  // fill the arena completely so no big tail hole remains
            void* p = fl.allocate(100 + i % 50);
            if (!p) break;
            CHECK(reinterpret_cast<std::uintptr_t>(p) % alignof(std::max_align_t) == 0);
            v.push_back(p);
        }
        CHECK(fl.validate());
        // free every other block -> fragmented
        for (std::size_t i = 0; i < v.size(); i += 2) fl.deallocate(v[i]);
        auto s = fl.stats();
        CHECK(fl.validate());
        CHECK(s.free_blocks > 100);
        CHECK(s.fragmentation > 0.9);  // many small holes, no large one
        CHECK(s.live_allocations == v.size() / 2);
        CHECK(fl.allocate(1000) == nullptr);  // plenty of free bytes but none contiguous
        CHECK(s.free_bytes > 10000);
        for (std::size_t i = 1; i < v.size(); i += 2) fl.deallocate(v[i]);
        s = fl.stats();
        CHECK(fl.validate());
        CHECK(s.free_blocks == 1);
        CHECK(s.fragmentation == 0.0);
        CHECK(s.free_bytes == s.capacity);
        CHECK(s.used_bytes == 0);
    }
}

TEST_CASE("free-list allocator: randomized stress with content checks") {
    std::mt19937 rng(42);
    FreeListAllocator fl(1 << 20);
    struct Live { unsigned char* p; std::size_t n; unsigned char tag; };
    std::vector<Live> live;
    for (int it = 0; it < 20000; ++it) {
        if (live.empty() || rng() % 3 != 0) {
            std::size_t n = 1 + rng() % 2000;
            std::size_t align = std::size_t(1) << (rng() % 7);  // 1..64
            auto* p = static_cast<unsigned char*>(fl.allocate(n, align));
            if (!p) continue;
            CHECK(reinterpret_cast<std::uintptr_t>(p) % align == 0);
            auto tag = static_cast<unsigned char>(rng());
            std::memset(p, tag, n);
            live.push_back({p, n, tag});
        } else {
            std::size_t k = rng() % live.size();
            Live l = live[k];
            for (std::size_t j = 0; j < l.n; ++j)
                if (l.p[j] != l.tag) FAIL("memory corrupted");
            fl.deallocate(l.p);
            live[k] = live.back();
            live.pop_back();
        }
        if (it % 1000 == 0) REQUIRE(fl.validate());
    }
    for (auto& l : live) fl.deallocate(l.p);
    CHECK(fl.validate());
    CHECK(fl.stats().free_blocks == 1);
}

TEST_CASE("free-list allocator: best fit picks the tighter hole") {
    FreeListAllocator fl(8192, FreeListAllocator::Policy::BestFit);
    void* a = fl.allocate(1000);
    void* sep1 = fl.allocate(16);
    void* b = fl.allocate(200);
    void* sep2 = fl.allocate(16);
    fl.deallocate(a);  // big hole first
    fl.deallocate(b);  // small hole second
    void* c = fl.allocate(150);
    CHECK(c == b);  // best fit chooses the 200-byte hole
    (void)sep1;
    (void)sep2;
    FreeListAllocator ff(8192, FreeListAllocator::Policy::FirstFit);
    void* a2 = ff.allocate(1000);
    ff.allocate(16);
    void* b2 = ff.allocate(200);
    ff.allocate(16);
    ff.deallocate(a2);
    ff.deallocate(b2);
    CHECK(ff.allocate(150) == a2);  // first fit chooses the first hole
}

TEST_CASE("free-list allocator: out of memory returns null") {
    FreeListAllocator fl(1024);
    CHECK(fl.allocate(4096) == nullptr);
    void* p = fl.allocate(900);
    CHECK(p);
    CHECK(fl.allocate(200) == nullptr);
    fl.deallocate(p);
    CHECK(fl.allocate(900) == p);
}

TEST_CASE("tracking allocator reports leaks, peaks and double frees") {
    FreeListAllocator fl(64 * 1024);
    TrackingAllocator tr(fl);
    void* a = tr.allocate_tagged(100, "mesh");
    void* b = tr.allocate_tagged(50, "mesh");
    void* c = tr.allocate_tagged(10, "audio");
    CHECK(tr.live_count() == 3);
    CHECK(tr.live_bytes() == 160);
    tr.deallocate(b);
    CHECK(tr.peak_bytes() == 160);
    CHECK(tr.live_bytes() == 110);
    tr.deallocate(b);  // double free is caught and not forwarded
    CHECK(tr.double_frees() == 1);
    std::string rep = tr.leak_report();
    CHECK(rep.find("2 live allocation(s), 110 bytes") != std::string::npos);
    CHECK(rep.find("mesh: 1 allocs, 100 bytes") != std::string::npos);
    CHECK(rep.find("audio: 1 allocs, 10 bytes") != std::string::npos);
    tr.deallocate(a);
    tr.deallocate(c);
    CHECK(tr.live_count() == 0);
    CHECK(tr.total_allocations() == 3);
    CHECK(fl.validate());
}
