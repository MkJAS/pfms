Week 6 Prework Questions
=========================


Attempt the following question with supporting material in canvas.

Ex01 - Simple Threading
-----------------------------------------

Create threads on two functions `incrementNum` and `printNum`, passing `num` and `numMutex` to both of them.

Attempt following
* Do not secure `num` by a mutex, what occurs?
* Insert a delay in each thread `std::this_thread::sleep_for (std::chrono::milliseconds(100));` is there a change?
* Use a [mutex](https://en.cppreference.com/w/cpp/thread/mutex) and the lock and unlock, is there a change?
* Use a [scoped lock](https://en.cppreference.com/w/cpp/thread/scoped_lock) or [lock guard](https://en.cppreference.com/w/cpp/thread/lock_guard) instead?

