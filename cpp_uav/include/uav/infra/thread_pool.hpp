#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace uav::infra {

class ThreadPool {
public:
    explicit ThreadPool(int num_threads = 4);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    template <typename F>
    auto submit(F&& task) -> std::future<decltype(task())> {
        using ReturnType = decltype(task());
        auto packaged = std::make_shared<std::packaged_task<ReturnType()>>(
            std::forward<F>(task));
        auto future = packaged->get_future();
        {
            std::lock_guard lock(queue_mutex_);
            tasks_.emplace([packaged] { (*packaged)(); });
        }
        queue_cv_.notify_one();
        return future;
    }

    void waitAll();

    struct Stats {
        int active_threads;
        int queued_tasks;
    };
    Stats getStats() const;

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> stop_{false};
    std::atomic<int> active_count_{0};
    std::condition_variable idle_cv_;

    void workerLoop();
};

} // namespace uav::infra
