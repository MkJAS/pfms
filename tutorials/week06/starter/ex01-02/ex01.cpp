#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

void incrementNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

void printNum(int &num, std::mutex &numMutex) {
    while (true) {
    //Use mutex 
    }
}

int main ()
{
    int num = 0;
    // We will use this mutex to synchonise access to num
    std::mutex numMutex;

    // Create the threads, how do we create the thread? What are the parameters
    //std::thread inc_thread();
    //std::thread print_thread();

    // Wait for the threads to finish (they wont)
    //inc_thread.join();
    //print_thread.join();

    return 0;
}



