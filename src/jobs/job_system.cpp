#include "jobs/job_system.hpp"

namespace ifg {

namespace {
thread_local const JobSystem* tl_owner = nullptr;
thread_local int tl_index = -1;
}  // namespace

JobSystem::JobSystem(unsigned threads) {
    if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
    nworkers_ = static_cast<int>(threads);
    for (unsigned i = 0; i <= threads; ++i) queues_.push_back(std::make_unique<Queue>());
    workers_.reserve(threads);
    for (unsigned i = 0; i < threads; ++i) workers_.emplace_back([this, i] { worker_main(static_cast<int>(i)); });
}

JobSystem::~JobSystem() {
    {
        std::lock_guard<std::mutex> lk(sleep_m_);
        stop_.store(true);
    }
    sleep_cv_.notify_all();
    for (auto& t : workers_) t.join();
}

void JobSystem::submit(Job job, WaitGroup* wg) {
    if (wg) wg->add();
    int target = (tl_owner == this) ? tl_index : nworkers_;
    {
        std::lock_guard<std::mutex> lk(queues_[target]->m);
        queues_[target]->q.push_back(Task{std::move(job), wg});
    }
    pending_.fetch_add(1, std::memory_order_release);
    { std::lock_guard<std::mutex> lk(sleep_m_); }
    sleep_cv_.notify_one();
}

bool JobSystem::pop_task(int self, Task& out) {
    const int n = nworkers_;
    // 1. own deque, LIFO
    if (self >= 0) {
        Queue& q = *queues_[self];
        std::lock_guard<std::mutex> lk(q.m);
        if (!q.q.empty()) {
            out = std::move(q.q.back());
            q.q.pop_back();
            return true;
        }
    }
    // 2. injection queue, FIFO
    {
        Queue& q = *queues_[n];
        std::lock_guard<std::mutex> lk(q.m);
        if (!q.q.empty()) {
            out = std::move(q.q.front());
            q.q.pop_front();
            return true;
        }
    }
    // 3. steal from the front of other workers' deques
    int start = self >= 0 ? self + 1 : 0;
    for (int k = 0; k < n; ++k) {
        int v = (start + k) % n;
        if (v == self) continue;
        Queue& q = *queues_[v];
        std::lock_guard<std::mutex> lk(q.m);
        if (!q.q.empty()) {
            out = std::move(q.q.front());
            q.q.pop_front();
            steals_.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
    }
    return false;
}

void JobSystem::run(Task& t) {
    pending_.fetch_sub(1, std::memory_order_acq_rel);
    t.fn();
    executed_.fetch_add(1, std::memory_order_relaxed);
    if (t.wg) t.wg->done();
}

bool JobSystem::try_run_one(int self) {
    Task t;
    if (!pop_task(self, t)) return false;
    run(t);
    return true;
}

void JobSystem::worker_main(int index) {
    tl_owner = this;
    tl_index = index;
    for (;;) {
        if (try_run_one(index)) continue;
        std::unique_lock<std::mutex> lk(sleep_m_);
        sleep_cv_.wait(lk, [&] { return stop_.load() || pending_.load(std::memory_order_acquire) > 0; });
        if (stop_.load() && pending_.load() <= 0) return;
    }
}

void JobSystem::wait(WaitGroup& wg) {
    int self = (tl_owner == this) ? tl_index : -1;
    while (!wg.finished()) {
        if (!try_run_one(self)) std::this_thread::yield();
    }
}

void JobSystem::parallel_for(std::size_t begin, std::size_t end, std::size_t grain,
                             const std::function<void(std::size_t, std::size_t)>& fn) {
    if (begin >= end) return;
    if (grain == 0) grain = 1;
    WaitGroup wg;
    for (std::size_t b = begin; b < end; b += grain) {
        std::size_t e = std::min(end, b + grain);
        submit([&fn, b, e] { fn(b, e); }, &wg);
    }
    wait(wg);
}

}  // namespace ifg
