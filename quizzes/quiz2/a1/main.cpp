#include <iostream>
#include <vector>
#include <chrono>

#include "circle.h"
#include "rectangle.h"
#include "triangle.h"
#include "line.h"

#include "analysis.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;


int main () {

    vector<Shape*> shapes;

    //! Rectangle with sides of 2.0
    shapes.push_back(new Rectangle(2.0));
    //! Rectangle with sides of 3.0
    shapes.push_back(new Rectangle(3.0));
    //! Rectangle with sides of 4.0
    shapes.push_back(new Rectangle(4.0));

    // Created two points that should only intersect the last Rectangle noted above
    geometry_msgs::Point pt1{1.7, 5.0},pt2{1.7, -5.0};

    // Created line from the two poits
    Line l(pt1,pt2);

    //! Create an object of class Analysis and
    //! Pass the shapes and line to analysis using the approprate member functions
    Analysis analysis(shapes,l);

    //Let's call method to see if they intersect
    std::vector<bool> intersects = analysis.intersectsLine();
  

    for(unsigned int i = 0; i < intersects.size(); i++) {

        std::cout << "Area " << i + 1 << " : " << shapes.at(i)->getArea() << std::endl;
        if(intersects.at(i)) {
            std::cout << "Shape " << i << " Intersects" << std::endl;
        }
    }

    //Let's delete pointers
    for (auto s : shapes) {
        delete s;
    }


}


