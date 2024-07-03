#include "sample.h"
#include <iostream>

int main () {


//  Create an object `sample` of `Sample` class
    int x; 
    //std::cout << "Type a number: ";
    //std::cin >> x; 
    x = 67;
    Sample sample(x);
    //sample.setvalue(5);


//  Display to standard output the value of parameter `value_` from the `sample` object.
    std::cout<<"\nThe sample number is: "<<sample.readvalue()<<std::endl;
    return 0;
}
