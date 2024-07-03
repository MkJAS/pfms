#include <iostream>
#include <vector>
#include <stdlib.h>
#include <random>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    vector<Shape*> shapes;
    
    shapes.push_back(new Rectangle(2,2));
    shapes.push_back(new Circle(3));
    shapes.push_back(new Triangle(4,5));

    for (auto it : shapes)
    {
        std::cout<<(*it).getArea()<<std::endl;
    }
    double num_circ;
    double num_rect;
    double num_tri;
    double max_length;

    std::cout<<"Enter number of Circles: "<<std::endl;
    std::cin>>num_circ;
    std::cout<<"Enter number of Rectangles: "<<std::endl;
    std::cin>>num_rect;
    std::cout<<"Enter number of Triangles: "<<std::endl;
    std::cin>>num_tri;
    std::cout<<"Enter a max length: "<<std::endl;
    std::cin>>max_length;

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, max_length); // define the range
    shapes.clear();
    //create shapes
    for (int i =0; i<num_circ;i++)
    {
        shapes.push_back(new Circle(distr(gen)));
    }
    for (int i =0; i<num_rect;i++)
    {
        shapes.push_back(new Rectangle(distr(gen),distr(gen)));
    }
    for (int i =0; i<num_tri;i++)
    {
        shapes.push_back(new Triangle(distr(gen),distr(gen)));
    }
    //set off set for all shapes
    for (auto it:shapes)
    {
        (*it).offsetCentre(distr(gen)/2,distr(gen)/2);
    }
    double x = 0, y = 0;

    std::cout<<"Select an x and y coordinate: "<<std::endl;

    std::cin>>x>>y;
    for (int i =0;i<shapes.size();i++)
    {
        std::cout<<i+1<<":"<<shapes[i]->getDescription()<<std::endl;
        if(shapes[i]->checkIntercept(x,y))
        {
            std::cout<<"INTERCEPTED!"<<std::endl;
        }
    }
    

    // triangle.offsetCentre(4,9);
    // std::cout<<std::boolalpha<<triangle.checkIntercept(50,8)<<std::endl;

}
