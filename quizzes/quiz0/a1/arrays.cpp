#include "arrays.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers
#include <math.h>

// function to populate array with random numbers
void populateWithRandomNumbers(double x[], unsigned int& array_size, unsigned int num_elements) {

    //we select a seed for the random generator, so it is truly random (never the same seed)
    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a normal distribution with means zero and standard deviation 10
    std::normal_distribution<> distribution(0,10.0);
    // generate the required amount of random numbers
    for (unsigned int i=array_size; i<array_size+num_elements; i++) {
        x[i] = distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}


//1) TASK: Create a function that assigns elements of array x to a vector named `vec`
void assignArrayToVector(double x[] ,unsigned int arraySize ,std::vector<double> &myVec )
{
    for (int i=0; i<arraySize;i++)
        {
            myVec.push_back(x[i]);
        }
}



//2) TASK: Create a function that accepts and vector and removes elements of vector greater than the limit (value)
void removeNumbersLargerThan(std::vector<double> &myVec, double limit)
{
    for (std::vector<double>::iterator it = myVec.begin() ; it != myVec.end(); ++it)
    {
        if (*it > limit)
        {
            myVec.erase( it );
        }
    }
}


//3) TASK: Create a function that computes the means and standard deviation and returns the Stats straucture with these elements
Stats computeMeanAndStdDev(std::vector<double> myVec)
{
    Stats stats;

    double sum = 0;
    double Size = myVec.size();
    //int n;
    for (auto& n : myVec)
    {
        sum += n;
    }
    double Mean = sum/Size;
    stats.mean = Mean;

    double diff = 0;
    double var_nume = 0;
    for (auto& n : myVec)
    {
        diff = (n - Mean)*(n - Mean);
        var_nume += diff;
    }
    double Std_dev = sqrt(var_nume/Size);
    stats.std_dev = Std_dev;

    return stats;
}

//4) TASK: Create a function that retruns a vector containing elements of an initial vector that are less than a value
std::vector<double> returnVecWithNumbersSmallerThan(std::vector<double> myVec, double limit)
{
    std::vector<double> smallVec;
    for (std::vector<double>::iterator it = myVec.begin() ; it != myVec.end(); ++it)
    {
        if (*it < limit)
        {
            myVec.erase( it );
        }
    }
    for (int i=0; i<myVec.size(); i++)
    {
        smallVec.push_back(myVec[i]);
    }
        
    return smallVec;
    //BONUS - Can you use a lambda function instead of looping?

}


