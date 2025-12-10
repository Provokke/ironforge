// Work-stealing thread pool. Each worker owns a deque: it pushes/pops at the
// back (LIFO, cache friendly) while idle workers steal from the front of a
// victim's deque (FIFO, takes the oldest/biggest work). Jobs submitted from
// non-worker threads go to a shared injection queue. Waiting on a WaitGroup
// helps execute jobs, so nested parallelism cannot deadlock.
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ifg {

class WaitGroup {
public:
    void add(int n = 1) { count_.fetch_add(n, std::memory_order_relaxed); }
    void done() { count_.fetch_sub(1, std::memory_order_acq_rel); }
    bool finished() const { return count_.load(std::memory_order_acquire) == 0; }

private:
    std::atomic<int> count_{0};
};

class JobSystem {
public:
    using Job = std::function<void()>;
    explicit JobSystem(unsigned threads = 0);  // 0 = hardware_concurrency
    ~JobSystem();
    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    void submit(Job job, WaitGroup* wg = nullptr);
    void wait(WaitGroup& wg);  // helps run jobs until wg reaches zero
    // Splits [begin, end) into chunks of `grain` and runs fn(chunk_begin, chunk_end).
    void parallel_for(std::size_t begin, std::size_t end, std::size_t grain,
                      const std::function<void(std::size_t, std::size_t)>& fn);
    unsigned thread_count() const { return static_cast<unsigned>(nworkers_); }
    std::uint64_t steal_count() const { return steals_.load(); }
    std::uint64_t executed_count() const { return executed_.load(); }

private:
    struct Task { Job fn; WaitGroup* wg; };
    struct Queue {
        std::mutex m;
        std::deque<Task> q;
    };
    bool try_run_one(int self);
    bool pop_task(int self, Task& out);
    void worker_main(int index);
    void run(Task& t);

    std::vector<std::unique_ptr<Queue>> queues_;  // [0..n) workers, [n] injection
    int nworkers_ = 0;  // fixed before any worker starts (workers_ grows while they run)
    std::vector<std::thread> workers_;
    std::mutex sleep_m_;
    std::condition_variable sleep_cv_;
    std::atomic<std::int64_t> pending_{0};
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> steals_{0}, executed_{0};
};

}  // namespace ifg
