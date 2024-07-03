// Includes std::cout and friends so we can output to console
#include <iostream>

// Ex01. 
//* Create a function that accepts a double value as a parameter and
//* Returns a bool value if the double is greater than zero and the square value instead of initial passed value.
<<<<<<< HEAD
bool Ex01(double& a)
{
    bool greater = a > 0;
    a = a*a;
return greater;    
}
// Ex02
//* Create an additional function that accepts a double value as a parameter and
//* Returns bool value if the double is greater than zero, the square value of the initially passed value, 
//  and the passed value incremented by one
bool Ex02(double& a, double& b)
{
    bool greater = a > 0;
    a = a*a;
    b++;
return greater;    
}

=======

// Ex02
//* Create an additional function that accepts a double value as a parameter and
//* Returns bool value if the double is greater than zero, the square value of the initially passed value, and the passed value incremented by one
>>>>>>> origin/subject


// Every executable needs a main function which returns an int
int main () {

	//Call the functions, obtain values and print to screen (using std::cout) 


    return 0;
}




