// BONUS Exercise
// Includes std::cout and friends so we can output to console
#include <iostream>
<<<<<<< HEAD
#include <string>
using namespace std;
=======
>>>>>>> origin/subject

// Every executable needs a main function which returns an int
int main () {

    // Create an string array (char[]) x to value “41012”
<<<<<<< HEAD
    string x[] = {"4","1","0","1","2"};
    // Can we typecast to integer each value (and [print] what the value of each integer is?) (int)(x[i])
    int num[5] = {};
    num[0] = stoi(x[0]);
    num[1] = stoi(x[1]);
    num[2] = stoi(x[2]);
    num[3] = stoi(x[3]);
    num[4] = stoi(x[4]);

    cout<<""<<num[0];
    cout<<""<<num[1];
    cout<<""<<num[2];
    cout<<""<<num[3];
    cout<<""<<num[4];
    
    for (int i = 0; i < 5; i++) 
    {
        cout <<"\n"<< num[i];
    }

    // Add up all the elements (as numbers)?
    int SUM = num[0]+num[1]+num[2]+num[3]+num[4];
    // Can we count number of elements less than 2
    // Use a for loop
    int num_2 = 0;
    for (int i = 0; i<5; i++)
    {
        if(num[i] < 2)   
            num_2++;
    }
    cout<<"\nAmount of numbers less than 2:"<<num_2;
    // Use a while loop
=======

    // Can we typecast to integer each value (and [print] what the value of each integer is?) (int)(x[i])

    // Add up all the elements (as numbers)?

    // Can we count number of elements less than 2
    // Use a for loop

    // Use a while loop

>>>>>>> origin/subject
    // ADVANCED: Use a for range loop
    // Note must have the following line to the CMakeList.txt to enable C++11 features
    // set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

    // Main function should return an integer
    return 0;
}
