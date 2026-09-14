#include <atomic>
#include <numeric>

#include "doctest/doctest.h"
#include "jobs/job_system.hpp"

using namespace ifg;

TEST_CASE("job system runs every job exactly once") {
    for (unsigned threads : {1u, 2u, 4u, 8u}) {
        JobSystem js(threads);
        constexpr int N = 20000;
        std::vector<std::atomic<int>> hits(N);
        WaitGroup wg;
        for (int i = 0; i < N; ++i) js.submit([&hits, i] { hits[i].fetch_add(1); }, &wg);
        js.wait(wg);
        int bad = 0;
        for (auto& h : hits) bad += h.load() != 1;
        CHECK(bad == 0);
    }
}

TEST_CASE("parallel_for covers the range without overlap") {
    JobSystem js(4);
    std::vector<int> v(100003, 0);
    js.parallel_for(0, v.size(), 1000, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) v[i] += 1;
    });
    CHECK(std::all_of(v.begin(), v.end(), [](int x) { return x == 1; }));
    js.parallel_for(5, 5, 10, [&](std::size_t, std::size_t) { FAIL("empty range must not run"); });
}

TEST_CASE("nested parallelism does not deadlock (waiters help)") {
    JobSystem js(3);
    std::atomic<long> sum{0};
    js.parallel_for(0, 64, 1, [&](std::size_t ob, std::size_t) {
        js.parallel_for(0, 64, 4, [&](std::size_t b, std::size_t e) {
            for (std::size_t i = b; i < e; ++i) sum.fetch_add(static_cast<long>(ob * 64 + i));
        });
    });
    long n = 64 * 64;
    CHECK(sum.load() == n * (n - 1) / 2);
}

TEST_CASE("work stealing happens when one worker spawns everything") {
    JobSystem js(4);
    std::atomic<int> done{0};
    WaitGroup outer;
    js.submit([&] {
        // runs on a worker: children land on that worker's deque and must be stolen
        WaitGroup inner;
        for (int i = 0; i < 2000; ++i)
            js.submit([&] {
                volatile double x = 0;
                for (int k = 0; k < 2000; ++k) x = x + k;
                done.fetch_add(1);
            }, &inner);
        js.wait(inner);
    }, &outer);
    // do not help from this thread: the outer job must run on a worker
    while (!outer.finished()) std::this_thread::yield();
    CHECK(done.load() == 2000);
    MESSAGE("steals: " << js.steal_count());
    CHECK(js.steal_count() > 0);
}

TEST_CASE("stress: many waves of recursive jobs") {
    JobSystem js(4);
    for (int wave = 0; wave < 50; ++wave) {
        std::atomic<int> count{0};
        WaitGroup wg;
        std::function<void(int)> spawn = [&](int depth) {
            count.fetch_add(1);
            if (depth == 0) return;
            for (int i = 0; i < 3; ++i) js.submit([&spawn, depth] { spawn(depth - 1); }, &wg);
        };
        js.submit([&] { spawn(5); }, &wg);
        js.wait(wg);
        CHECK(count.load() == 1 + 3 + 9 + 27 + 81 + 243);
    }
}

TEST_CASE("destructor drains pending jobs") {
    std::atomic<int> n{0};
    {
        JobSystem js(2);
        for (int i = 0; i < 1000; ++i) js.submit([&] { n.fetch_add(1); });
    }
    CHECK(n.load() == 1000);
}
