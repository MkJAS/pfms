// We need to include the declaration of our new circle class in order to use it.
#include "circle.h"
#include <iostream>
#include <random>   
#include <chrono>   

double CirclesArea(std::vector<Circle> CircVec)
{
    double total_area = 0;
    // int n;
    // double area = CircVec[0].getarea();

    for (std::vector<Circle>::iterator it = CircVec.begin() ; it != CircVec.end(); ++it)
    {
        total_area += (*it).getarea();
    }
   return total_area;
}

int main () 
{
    Circle circle1(1.0);
    Circle circle2(2.0);
    Circle circle3(5.0);

    std::cout<<"Parameters of circles;"<<std::endl;
    std::cout<<"Area: "<<circle1.getarea()<<" "<<circle2.getarea()<<" "<<circle3.getarea()<<std::endl;
    std::cout<<"Perimeter: "<<circle1.getperimeter()<<" "<<circle2.getperimeter()<<" "<<circle3.getperimeter()<<std::endl;

    int x = 0; 
    std::cout << "How many circles to create?: ";
    std::cin >> x;
    
    std::vector<Circle> vecCirc;

    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::uniform_real_distribution<double> distribution(1.0,10.0);

    for (int i = 0;i<x;i++)
    {
      double rnd_radius = distribution(generator);
      std::cout<<"Radius of circle "<<i+1<<": "<<rnd_radius<<std::endl;
      vecCirc.push_back(Circle(rnd_radius));   
    
    }
    double totalarea = CirclesArea(vecCirc);
    std::cout<<"\nTotal area: "<<totalarea<<std::endl;


    return 0;
}

