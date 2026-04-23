#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include <string.h>   // for memset

#include <cstdint>
#include <iterator>

template <typename T, uint64_t N>
class CircularBuffer {
    struct Iterator {
        // From:
        // https://www.internalpointers.com/post/writing-custom-iterators-modern-cpp
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = T;
        using pointer           = T*;
        using reference         = T&;

        Iterator(pointer ptr, pointer buffer_start, pointer buffer_end)
            : m_ptr(ptr), buffer_start_(buffer_start), buffer_end_(buffer_end) {
        }
        reference operator*() const {
            return *m_ptr;
        }
        pointer operator->() {
            return m_ptr;
        }
        Iterator& operator++() {   // Prefix increment
            if (m_ptr == buffer_end_) {
                m_ptr = buffer_start_;
            } else {
                m_ptr++;
            }
            return *this;
        }
        Iterator operator++(int) {   // Postfix increment
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }
        friend bool operator==(const Iterator& a, const Iterator& b) {
            return a.m_ptr == b.m_ptr;
        };
        friend bool operator!=(const Iterator& a, const Iterator& b) {
            return a.m_ptr != b.m_ptr;
        };

       private:
        pointer m_ptr, buffer_start_, buffer_end_;
    };

   public:
    CircularBuffer() = default;

    Iterator begin() {
        return Iterator(&(buffer_[tail_idx_]), &(buffer_[0]), &(buffer_[N]));
    }

    Iterator end() {
        return Iterator(&(buffer_[head_idx_]), &(buffer_[0]), &(buffer_[N]));
    }

    /**
     * \brief Add an element to the buffer
     *
     * \param newElement New element to add
     */
    void add(T newElement) {
        // Add the new element
        buffer_[head_idx_] = newElement;

        mod(++head_idx_);   // Advance the head index

        // Check if full and advance the tail index if so
        if (full()) {
            mod(++tail_idx_);
        } else {
            size_++;
        }
    }

    /**
     * \brief Return the oldest element in the buffer
     * NOTE: this does not remove the element from the buffer, and will return
     * garbage if the buffer is empty
     */
    T get() const {
        return buffer_[tail_idx_];
    }

    /**
     * \brief Remove the oldest element from the buffer
     * and advance the tail index
     */
    void remove() {
        if (empty()) {
            return;
        }
        size_--;
        mod(++tail_idx_);   // Increment tail index
    }

    void reset() {
        memset(&buffer_, 0, sizeof(buffer_));
        head_idx_ = 0;
        tail_idx_ = 0;
        size_     = 0;
    }

    bool full() const {
        return size_ == N;
    }

    bool empty() const {
        return size_ == 0;
    }

    int head() const {
        return head_idx_;
    }

    int tail() const {
        return tail_idx_;
    }

    uint64_t size() const {
        return size_;
    }

    uint64_t capacity() const {
        return N;
    }

    /**
     * \brief Perform modulo operation
     *
     * \param idx Index to modulate by buffer capacity, passed by reference
     */
    static void mod(uint64_t& idx) {
        idx = idx % (N + 1);
    }

   private:
    T buffer_[N + 1];       // 1 extra element so begin and end iterators are
                            // different when buffer is full
    uint64_t head_idx_{};   // array idx where next element will be inserted
    uint64_t tail_idx_{};   // array idx where next element will be "deleted"
    uint64_t size_{};
};

#endif /* _CIRCULAR_BUFFER_H_ */
