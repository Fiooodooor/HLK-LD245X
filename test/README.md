# HLK-LD245X Tests

This directory contains unit tests for the HLK-LD245X library.

## Running Tests

### Desktop Testing (Native C++)

The circular buffer tests can be compiled and run on a desktop system:

```bash
cd test
g++ -std=c++17 test_circular_buffer.cpp -o test_circular_buffer
./test_circular_buffer
```

### Arduino Testing

For Arduino-specific tests, you can use frameworks like:
- [ArduinoUnit](https://github.com/mmurdoch/arduinounit)
- [Unity](https://github.com/ThrowTheSwitch/Unity)
- [AUnit](https://github.com/bxparks/AUnit)

## Test Coverage

- `test_circular_buffer.cpp` - Tests for the CircularBuffer template class
  - Write/read operations
  - Wraparound behavior
  - Pattern finding
  - Bulk operations
  - Discard functionality

## Future Tests

Planned test coverage:
- RadarTarget encoding/decoding
- Frame parsing with various data patterns
- Math optimization accuracy
- Performance benchmarks
- Edge cases and error handling
