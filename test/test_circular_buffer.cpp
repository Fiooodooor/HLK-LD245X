/**
 * Basic unit tests for CircularBuffer
 *
 * To run these tests, compile with a test framework like Unity or ArduinoUnit
 */

#include "../src/CircularBuffer.hpp"
#include <assert.h>
#include <stdio.h>

using namespace esphome::ld245x;

// Test helper macros
#define TEST_ASSERT(condition, message) \
    if (!(condition)) { \
        printf("FAIL: %s\n", message); \
        return false; \
    }

#define TEST_ASSERT_EQUAL(expected, actual, message) \
    if ((expected) != (actual)) { \
        printf("FAIL: %s (expected: %d, actual: %d)\n", message, (int)expected, (int)actual); \
        return false; \
    }

bool test_circular_buffer_write_read() {
    CircularBuffer<uint8_t, 16> buf;

    TEST_ASSERT(buf.write(42), "Should be able to write to empty buffer");
    TEST_ASSERT_EQUAL(1, buf.available(), "Buffer should have 1 item");

    uint8_t val;
    TEST_ASSERT(buf.read(val), "Should be able to read from buffer");
    TEST_ASSERT_EQUAL(42, val, "Read value should match written value");
    TEST_ASSERT_EQUAL(0, buf.available(), "Buffer should be empty after read");

    printf("PASS: test_circular_buffer_write_read\n");
    return true;
}

bool test_circular_buffer_wrap() {
    CircularBuffer<uint8_t, 4> buf;

    // Fill buffer
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT(buf.write(i), "Should be able to fill buffer");
    }
    TEST_ASSERT(buf.isFull(), "Buffer should be full");
    TEST_ASSERT(!buf.write(100), "Should not be able to write to full buffer");

    // Read two items
    uint8_t val;
    buf.read(val);
    buf.read(val);
    TEST_ASSERT_EQUAL(2, buf.available(), "Buffer should have 2 items");

    // Write two more (wraps around)
    TEST_ASSERT(buf.write(10), "Should be able to write after reading");
    TEST_ASSERT(buf.write(11), "Should be able to write with wraparound");
    TEST_ASSERT(buf.isFull(), "Buffer should be full again");

    printf("PASS: test_circular_buffer_wrap\n");
    return true;
}

bool test_circular_buffer_find_pattern() {
    CircularBuffer<uint8_t, 32> buf;

    uint8_t data[] = {0x01, 0x02, 0xAA, 0xFF, 0x03, 0x04};
    buf.write(data, 6);

    uint8_t pattern[] = {0xAA, 0xFF};
    int pos = buf.find(pattern, 2);

    TEST_ASSERT_EQUAL(2, pos, "Pattern should be found at position 2");

    uint8_t not_found_pattern[] = {0xBB, 0xCC};
    pos = buf.find(not_found_pattern, 2);
    TEST_ASSERT_EQUAL(-1, pos, "Non-existent pattern should return -1");

    printf("PASS: test_circular_buffer_find_pattern\n");
    return true;
}

bool test_circular_buffer_bulk_operations() {
    CircularBuffer<uint8_t, 16> buf;

    uint8_t write_data[] = {1, 2, 3, 4, 5};
    size_t written = buf.write(write_data, 5);
    TEST_ASSERT_EQUAL(5, written, "Should write all 5 bytes");
    TEST_ASSERT_EQUAL(5, buf.available(), "Buffer should have 5 bytes");

    uint8_t read_data[5];
    size_t read = buf.read(read_data, 5);
    TEST_ASSERT_EQUAL(5, read, "Should read all 5 bytes");

    for (int i = 0; i < 5; i++) {
        TEST_ASSERT_EQUAL(write_data[i], read_data[i], "Read data should match written data");
    }

    printf("PASS: test_circular_buffer_bulk_operations\n");
    return true;
}

bool test_circular_buffer_discard() {
    CircularBuffer<uint8_t, 16> buf;

    uint8_t data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    buf.write(data, 8);
    TEST_ASSERT_EQUAL(8, buf.available(), "Buffer should have 8 bytes");

    buf.discard(3);
    TEST_ASSERT_EQUAL(5, buf.available(), "Buffer should have 5 bytes after discard");

    uint8_t val;
    buf.read(val);
    TEST_ASSERT_EQUAL(4, val, "First byte after discard should be 4");

    printf("PASS: test_circular_buffer_discard\n");
    return true;
}

int main() {
    printf("Running CircularBuffer tests...\n\n");

    int passed = 0;
    int total = 0;

    total++; if (test_circular_buffer_write_read()) passed++;
    total++; if (test_circular_buffer_wrap()) passed++;
    total++; if (test_circular_buffer_find_pattern()) passed++;
    total++; if (test_circular_buffer_bulk_operations()) passed++;
    total++; if (test_circular_buffer_discard()) passed++;

    printf("\n%d/%d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
