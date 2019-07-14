#pragma once

#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <atomic>

namespace {
int cmpxchg(std::atomic<int>* var, int old_value, int new_value) {
  int* var_after_update = &old_value;
  std::atomic_compare_exchange_strong(var, var_after_update, new_value);
  return *var_after_update;
}
}  // namespace

// Implemented based on Futexes Are Tricky, Ulrich Drepper
class Mutex {
 public:
  Mutex() : value_(0) {
  }
  bool lock(const struct timespec* timeout = 0) {
    int value_after_cmpxchg = cmpxchg(&value_, 0, 1);
    if (value_after_cmpxchg != 0) {
      do {
        if (value_after_cmpxchg == 2 || cmpxchg(&value_, 1, 2) != 0) {
          int result = syscall(SYS_futex, (int*)&value_, FUTEX_WAIT, 2, timeout, 0, 0);
          if (result != 0) {
            return false;
          }
          return true;
        }
      } while ((value_after_cmpxchg = cmpxchg(&value_, 0, 2)) != 0);
    }
    return true;
  }

  void unlock() {
    if (value_.fetch_sub(1) != 1) {
      value_.store(0);
      syscall(SYS_futex, (int*)&value_, FUTEX_WAKE, 1, 0, 0, 0);
    }
  }

 private:
  // 0: unlocked
  // 1: locked
  // 3: locked, with waiters
  std::atomic<int> value_;
};
