#include <iostream>
#include "container_ops.h" // header file that contains the functions

#include <chrono>
#include <random>



int main() {

    //we select a seed for the random generator, so it is truly random (never the same seed)
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a normal distribution with means zero and standard deviation 10
    std::normal_distribution<> distribution(0,10.0);

    std::deque<double> container;
    // generate some random numbers for a deque
    for (unsigned int i=0; i<3; i++) {
        container.push_back(distribution(generator));
    }

    //An example usage of the function your developing in TASK1
    populateContainer(container, 3, 3.0);
    std::cout<<"size of container: "<<container.size()<<std::endl;
    std::cout<<"The deque is: ";
    for (auto i:container)
    {
        std::cout<<'\t'<<i;
    }
    

    //An example usage of the function your developing in TASK2
    std::cout<<"\n"<<std::endl;
    bubbleSortContainer(container);
    for (int i=0;i<container.size();i++)
    {
        std::cout<<container[i]<<" ";
    }

	return 0;
} 
