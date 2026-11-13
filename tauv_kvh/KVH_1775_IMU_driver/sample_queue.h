#ifndef KVH_SAMPLE_QUEUE_H
#define KVH_SAMPLE_QUEUE_H

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <vector>

// Preallocated handoff: the producer never waits for the consumer to make space.
// On overload, discard the oldest sample to bound publication latency.
template <typename T>
class SampleQueue {
 public:
    explicit SampleQueue(size_t capacity) : storage_(capacity) {
        if (!capacity) throw std::invalid_argument("sample queue capacity must be positive");
    }
    bool push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return false;
        bool dropped = size_ == storage_.size();
        if (dropped) { head_ = (head_ + 1) % storage_.size(); --size_; }
        storage_[(head_ + size_) % storage_.size()] = value;
        ++size_;
        ready_.notify_one();
        return dropped;
    }
    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return closed_ || size_ > 0; });
        if (closed_) return false;
        value = storage_[head_];
        head_ = (head_ + 1) % storage_.size();
        --size_;
        return true;
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        ready_.notify_all();
    }
 private:
    std::vector<T> storage_;
    size_t head_ = 0, size_ = 0;
    bool closed_ = false;
    std::mutex mutex_;
    std::condition_variable ready_;
};
#endif
