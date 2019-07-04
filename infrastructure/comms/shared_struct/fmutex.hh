#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <linux/futex.h>
#include <pthread.h>
#include <sstream>
#include <sys/resource.h>
#include <sys/shm.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

namespace {
  int cmpxchg(std::atomic<int>* atom, int expected, int desired) {
    int* ep = &expected;
    std::atomic_compare_exchange_strong(atom, ep, desired);
    return *ep;
  }
}

enum class FutexValue {
  UNLOCKED,
  LOCKED,
  LOCKED_WITH_WAITERS
};

// Implemented based on Futexes Are Tricky, Ulrich Drepper
class Mutex {
public:
  Mutex() : value_(0) {}

  bool lock(const struct timespec *timeout = 0) {
    int value_after_cmpxchg = cmpxchg(&value_, 0, 1);
    // If the lock was previously unlocked, there's nothing else for us to do.
    // Otherwise, we'll probably have to wait.
    if (value_after_cmpxchg != 0) {
      do {
        // If the mutex is locked, we signal that we're waiting by setting the
        // atom to 2. A shortcut checks is it's 2 already and avoids the atomic
        // operation in this case.
        if (value_after_cmpxchg == 2 || cmpxchg(&value_, 1, 2) != 0) {
          // Here we have to actually sleep, because the mutex is actually
          // locked. Note that it's not necessary to loop around this syscall;
          // a spurious wakeup will do no harm since we only exit the do...while
          // loop when value_ is indeed 0.
          int result = syscall(SYS_futex, (int*)&value_, FUTEX_WAIT, 2, timeout, 0, 0);
          if (result != 0) {
            return false;
          }
          return true;
        }
        // We're here when either:
        // (a) the mutex was in fact unlocked (by an intervening thread).
        // (b) we slept waiting for the atom and were awoken.
        //
        // So we try to lock the atom again. We set teh state to 2 because we
        // can't be certain there's no other thread at this exact point. So we
        // prefer to err on the safe side.
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
  // 0 means unlocked
  // 1 means locked, no waiters
  // 2 means locked, there are waiters in lock()
  std::atomic<int> value_;
};
