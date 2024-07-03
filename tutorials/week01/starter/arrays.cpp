// Includes std::cout and friends so we can output to console
#include <iostream>
<<<<<<< HEAD
using namespace std;

=======
>>>>>>> origin/subject

// Create a macro (#define) to represent the array size
#define ARRAY_SIZE 10

// Every executable needs a main function which returns an int
int main () {
    // Create an array x of doubles with 10 elements
<<<<<<< HEAD
    double x [ARRAY_SIZE] = {};  
    // Populate the elements of array on creating of array, each element [i] has value i (HINT: INITIALISER LIST)
    double y[] = {0,1,2,3,4,5,6,7,8,9}; 
    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (HINT: USE MACRO)
    for (int i=0;i<10;i++)
    {
        //cout<<"\n"<<i;
        x[i] = i;
    }
    cout<<"\n";
    // Can you use a pointer and loop to initialise elements of array
     double z [ARRAY_SIZE] = {};
     double* p;
     for (int i=0;i<10;i++)
    {
        p = &x[i];
        //cout<<"\n"<<i;
        z[i] = *p;
    }
    // Main function should return an integer
    for (int i = ARRAY_SIZE - 1; i >= 0; i--) 
    {
        cout <<"\n"<< x[i];
        cout<<"\n"<<z[i];
    }
    
=======
    // Populate the elements of array on creating of array, each element [i] has value i (HINT: INITIALISER LIST)

    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (HINT: USE MACRO)

    // Can you use a pointer and loop to initialise elements of array

    // Main function should return an integer
>>>>>>> origin/subject
    return 0;
}
