// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

template<typename T, int N>
class RingBuffer {
  public:
    RingBuffer() : head(0), tail(0), count(0) {}

    T* push(const T& value) {
        if (count == N) return nullptr; // Full
        buffer[head] = value;
        T* ptr = &buffer[head];
        head = (head + 1) % N;
        ++count;
        return ptr;
    }

    bool pop(T& value) {
        if (count == 0) return false; // Empty
        value = buffer[tail];
        tail = (tail + 1) % N;
        --count;
        return true;
    }

    bool pop() {
        if (count == 0) return false; // Empty
        tail = (tail + 1) % N;
        --count;
        return true;
    }

    T* peek() {
      if (count == 0) return nullptr;
      return &(buffer[tail]);
    }

    T* get(int i) {
        if (count == 0) return nullptr;
        return &(buffer[(tail + i) % N]);
    }

    bool empty() const { return count == 0; }
    bool full() const { return count == N; }
    int size() const { return count; }
    int free_item_count() const { return N-count; }

  private:
    T buffer[N];
    volatile int head;
    volatile int tail;
    volatile int count;
};
