#ifndef __CIRCULAR_BUFFER_HPP__
#define __CIRCULAR_BUFFER_HPP__

#include <stdint.h>
#include <string.h>

namespace esphome::ld245x {

/**
 * @brief A template-based circular buffer for efficient data buffering
 *
 * This circular buffer provides O(1) read and write operations with
 * wrap-around support. It's designed for single-producer-single-consumer
 * scenarios with volatile indices for thread-safety.
 *
 * @tparam T The type of elements stored in the buffer
 * @tparam SIZE The fixed size of the buffer (must be > 0)
 */
template<typename T, size_t SIZE>
class CircularBuffer {
public:
    CircularBuffer() : head(0), tail(0), count(0) {
        static_assert(SIZE > 0 && SIZE <= 32768, "Buffer size must be 1-32768");
    }

    /**
     * @brief Write a single element to the buffer
     * @param item The item to write
     * @return true if successful, false if buffer is full
     */
    bool write(const T& item) {
        if (isFull()) return false;
        buffer[head] = item;
        head = (head + 1) % SIZE;
        count++;
        return true;
    }

    /**
     * @brief Write multiple elements to the buffer (bulk operation)
     * @param items Pointer to array of items to write
     * @param len Number of items to write
     * @return Number of items actually written
     */
    size_t write(const T* items, size_t len) {
        if (!items) return 0;
        size_t written = 0;
        while (written < len && !isFull()) {
            buffer[head] = items[written];
            head = (head + 1) % SIZE;
            count++;
            written++;
        }
        return written;
    }

    /**
     * @brief Read a single element from the buffer
     * @param item Reference to store the read item
     * @return true if successful, false if buffer is empty
     */
    bool read(T& item) {
        if (isEmpty()) return false;
        item = buffer[tail];
        tail = (tail + 1) % SIZE;
        count--;
        return true;
    }

    /**
     * @brief Read multiple elements from the buffer (bulk operation)
     * @param items Pointer to array to store read items
     * @param len Maximum number of items to read
     * @return Number of items actually read
     */
    size_t read(T* items, size_t len) {
        if (!items) return 0;
        size_t readCount = 0;
        while (readCount < len && !isEmpty()) {
            items[readCount] = buffer[tail];
            tail = (tail + 1) % SIZE;
            count--;
            readCount++;
        }
        return readCount;
    }

    /**
     * @brief Peek at an element without removing it
     * @param item Reference to store the peeked item
     * @param offset Offset from tail (0 = next item to be read)
     * @return true if successful, false if offset is beyond available data
     */
    bool peek(T& item, size_t offset = 0) const {
        size_t c = count;  // Read volatile once
        if (offset >= c) return false;
        size_t t = tail;  // Read volatile once
        size_t pos = (t + offset) % SIZE;
        item = buffer[pos];
        return true;
    }

    /**
     * @brief Find a pattern in the buffer
     * @param pattern Pointer to pattern to search for
     * @param patternLen Length of the pattern
     * @return Offset from tail where pattern starts, or -1 if not found
     */
    int find(const T* pattern, size_t patternLen) const {
        size_t c = count;  // Read volatile once
        if (!pattern || patternLen > c || patternLen == 0) return -1;

        size_t searchLen = c - patternLen + 1;
        for (size_t i = 0; i < searchLen; i++) {
            bool match = true;
            for (size_t j = 0; j < patternLen; j++) {
                T val = T();
                peek(val, i + j);
                if (val != pattern[j]) {
                    match = false;
                    break;
                }
            }
            if (match) return static_cast<int>(i);
        }
        return -1;
    }

    /**
     * @brief Discard bytes from the head of the buffer
     * @param len Number of bytes to discard
     */
    void discard(size_t len) {
        if (len >= count) {
            clear();
            return;
        }
        tail = (tail + len) % SIZE;
        count -= len;
    }

    /**
     * @brief Clear all data from the buffer
     */
    void clear() {
        head = tail = count = 0;
    }

    // Query methods
    size_t available() const {
        size_t c = count;  // Read volatile once
        return c;
    }
    size_t capacity() const { return SIZE; }
    size_t freeSpace() const {
        size_t c = count;  // Read volatile once
        return SIZE - c;
    }
    bool isEmpty() const {
        size_t c = count;  // Read volatile once
        return c == 0;
    }
    bool isFull() const {
        size_t c = count;  // Read volatile once
        return c >= SIZE;
    }

    /**
     * @brief Statistics structure for debugging and monitoring
     */
    struct Stats {
        uint32_t overflowCount;
        uint32_t underflowCount;
        size_t maxUsage;
    };

    /**
     * @brief Update statistics based on current buffer state
     */
    void updateStats() {
        size_t c = count;  // Read volatile once
        if (c > stats.maxUsage) stats.maxUsage = c;
        if (isFull()) stats.overflowCount++;
    }

    const Stats& getStats() const { return stats; }
    void resetStats() { stats = {0, 0, 0}; }

private:
    T buffer[SIZE];
    volatile size_t head;
    volatile size_t tail;
    volatile size_t count;
    Stats stats = {0, 0, 0};
};

// Type aliases for common use cases
using ByteCircularBuffer = CircularBuffer<uint8_t, 2048>;
using SmallByteCircularBuffer = CircularBuffer<uint8_t, 512>;

} // namespace esphome::ld245x

#endif // __CIRCULAR_BUFFER_HPP__
