Week 6 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the tutorial session.


Ex02 - Critical Sections
-----------------------------------------

Consider a case where we want to process a stream of incoming data as fast as possible using multithreading.
To showcase the requirement of having to synchronise access, create a program which:

* Fills a queue with one million non-zero numbers which sum to zero

And thereafter:

* Spawns two threads in parallel, which each repeatedly pop the front of the queue and add it to their subtotal until the the queue is empty
* Combines the two subtotals to compute the sum of all numbers from the queue

Hint: _To ensure synchronisation use std::mutex and std::lock_guard_

**Learning outcomes** (1) passing same function to two threads (2) we can not guarantee when threads run (3) providing a bit of time between locking/unlocking mutex in same thread (be it a sleep or some heavy computing) will not `starve` the other thread

Ex03 - Multi-Thread Access Data 
-------------------------

Three threads should access same data container, a struct that contains:
* string name
* vector of doubles

Thread 1: adds random number (from uniform distribution 0-100) to the vector of doubles 
Thread 2: removes numbers less than 20 and greater than 80
Thread 3: Keeps size of vector to max 20 elements, removes oldest element

Questions:
* How best to protect data?
* What should a efficient implementation do?

**Learning outcomes** (1) instead of passing a single variable we can pass a struct that contains more data to a thread and embed the mutex (2) the way threads will run is unpredictable, if we need synchronisation we need to use something other than mutex

Ex04 - Multi-Thread Access Data 
-------------------------

Building upon previous example, process all the data and functions in a Class.

Questions:
* What changes when it comes to starting the threads that needs to run a function in a class? 

**Learning outcomes** we show how to implement a thread safe class, where functions of object can be called and internally it guarantees the variables are thread safe. 
