#include <random>
#include <iostream>
#include <limits>

#include "shapeprocessing.h" // Q) Why do we only need to include this header?

int main () {

  vector<Shape*> shape;



  //    * Allow the user to specify number of circles and rectangles
  double num_circ = 0;
  double num_rect = 0;

  std::cout<<"Enter number of Circles: "<<std::endl;
  std::cin>>num_circ;
  std::cout<<"Enter number of Rectangles: "<<std::endl;
  std::cin>>num_rect;

  //    * Create the shapes with random lengths to be capped to `max_length` - which is a const in    [shape_processing.h]./starter/library_test/shape_processing.h).
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr(0, shapeprocessing::max_length); // define the range

  for (int i =0; i<num_circ;i++)
    {
        shape.push_back(new Circle(distr(gen)));
    }
  for (int i =0; i<num_rect;i++)
    {
        shape.push_back(new Rectangle(distr(gen),distr(gen)));
    }

  ShapeProcessing processedshapes(shape);

  //Allows user to enter a location x,y within (-max_size, max_size). `max_size` is a const in [shape_processing.h]./starter/library_test/shape_processing.h) until all the shapes have been intersected


    //! What is this? Why is this a good practice?
    //! How has this been wrapped, look up namespace.
  std::cout <<"Enter a x and y value between -"<<shapeprocessing::max_size <<" and "<<shapeprocessing::max_size<<std::endl;
  double x = 0,y = 0;
  std::cin>>x>>y;
  
  while(shapeprocessing::max_size/abs(x)<1 || shapeprocessing::max_size/abs(y)<1)
  {
    std::cin.clear();
    std::cout << "Invalid entry! Please enter a valid value: ";
    std::cin>>x>>y;
  }  
  std::cout<<"You entered:x="<<x<<" and y="<<y<<std::endl;

  
    //! Additional questions
    //! Is shape and the shape_ in ShapeProcessing the same?
    //! If not, how do we change the code for this to occur?

    return 0;
}
