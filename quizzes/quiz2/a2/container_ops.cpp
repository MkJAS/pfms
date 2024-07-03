#include "container_ops.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers


//!TODO TASK 1:
//! Implement function that accepts a container and modifies it by adding user specified numbers of elements
//! To front of container. The actual element is also supplied by user (for instance num_values =4 element =-1.5  means
//! four elements of -1.5 are added to begining of deque)
void populateContainer(std::deque<double>& container, unsigned int num_values, double element)
{
    for (unsigned int i=0;i<num_values;i++)
    {
        container.push_front(element);
    }
}

//!TODO TASK 2: Implement function that accept the chosen container and rearranges elements by bubble sort operation
//! An example on arrays with C++ code is here https://www.programiz.com/dsa/bubble-sort
void bubbleSortContainer( std::deque<double>& container)
{
    for (int step = 0; step < container.size(); ++step) 
    {      
        for (auto it = container.begin(); it != container.end(); ++it) 
        {
            if (it != container.end()-1)
            {
                if (*it > *(it + 1)) 
                {
                    auto temp = *(it+1);
                    container.erase(it+1);
                    container.push_front(temp);
                    
                }
            }
        }
    }
}


