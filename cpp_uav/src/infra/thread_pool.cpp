#include "uav/infra/thread_pool.hpp"

namespace uav::infra {

ThreadPool::ThreadPool(int num_threads) {
    for (int i = 0; i < num_threads; ++i) {
        workers_.emplace_back(&ThreadPool::workerLoop, this);
    }
}

ThreadPool::~ThreadPool() {
    stop_ = true;
    queue_cv_.notify_all();
    for (auto& w : workers_) {
        if (w.joinable()) w.join();
    }
}

void ThreadPool::workerLoop() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) return;
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        active_count_++;
        try { task(); } catch (...) {}
        active_count_--;
        idle_cv_.notify_all();
    }
}

void ThreadPool::waitAll() {
    std::unique_lock lock(queue_mutex_);
    idle_cv_.wait(lock, [this] {
        return tasks_.empty() && active_count_ == 0;
    });
}

ThreadPool::Stats ThreadPool::getStats() const {
    std::lock_guard lock(queue_mutex_);
    return {active_count_.load(), static_cast<int>(tasks_.size())};
}

} // namespace uav::infra
