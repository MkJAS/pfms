Quiz 2
======

In both quizzes DO NOT CHANGE ANY OF THE EXISTING function declarations. You can add other private functions, but your code will only be tested against the currently defined public functions accessible.

Part A
------

**PREAMBLE**

This coding challenges bears some similarity to Assignment 1, it examines whether a line and a shape intercets each other and the fact we use a base class to abstract the shapes and uses enums and properties inside the object to determine it's inner behaviour. To solve this challenge for polygons we have provided code that checks wether two lines intercet, and this therefore generalises to a line and a polygon (which is made from line segments). 

The code examines lines and consistency of line orientations, refer to [line segment intercect](https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/). The implementation from this website has been provided in [analysis.cpp](./a2/analysis.cpp) . Your task is to use this code on two polygonal shapes `Triangle` and `Rectangle` and we use a vector of pointers to the base class `Shape` ( a `std::vector<Shape*>`)

**TASK DESCRIPTIONS**

1. We need to ensure that each shape has a type `type_` which can be one of the enums `shape::Type` and a number of `sides_`. Sides are continuous (a side has no change of cuvature or direction). For example, a hexagon has 6 sides. Initialise the member variables such that on creation of the object of the class  `Circle` , `Rectangle` , `Triangle` these member variables are initialised. 

   HINT: Have a look at the constructor of the base class (in terms of what parameter you can initialise via the base class Shape constructor, and what parameters you will need to initialise via assignment to the member variable.

   NOTE: There is one suite of tests (ShapeTest) with 3 seperate tests provided (one for each shape). This entire suite is only for this task and is 0.5 of marks

2. TASK: Implement the `std::vector<int> detectShapes()` function that returns a vector of 3 elements, each element a count of the number of shapes in `shapes_` which are of type `Circle`, `Triangle` and `Rectangle` respectively (ie element 0 of vector is count of Circles, 2 of triangles and 3 of rectangles).

   HINT: Use the fact that you have `shape::Type` and  `sides_` for this task. It shows that all the required data to decide this is already available in the object. DO NOT USE string comparisons!

3. TASK: Use your developed code in TASK 2 to determine shape types and re-use it for `std::vector<bool> Analysis::intersectsLine()` to determine the type of shape and call the respective function specific to the shape. 

   Now you need to implement the function `bool intersectsRectangle(Shape* shape);` which is specific to `Rectangles`. Have a look at the resources for line segment interaction, the functions are already provided and in particular `bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2);` function that you will use. 

   HINT: To determine if the rectangle intercets the line, look at each side of the rectangle and the line respectively. Determine the coordinates of corners, make the lines from these (refer to [Line](./a1/line.h)  and the constructor as well as `getPoints(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2)` function.

   NOTE: `bool intersectsRectangle(Shape* shape)` is a private function. For testing we would simply provide `Rectangles` as shapes to the constuctor of `Analysis` and then call `intersectsLine`. Refer to an example of using the functions in [example main](./a1/main.cpp) 

4. TASK: Now you need to implement the function `bool intersectsTriangles(Shape* shape);` which is specific to `Triangles`. 

   HINT: To determine if the triangles intercets the line, look at each side of the triangle and the line respectively. Determine the coordinates of corners, make the lines from these (refer to [Line](./a1/line.h)  and the constructor as well as `getPoints(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2)` function.


Part B
------

1) TASK: Implement function `void populateContainer(std::deque<double>& container, unsigned int num_values, double element)` that accepts a container and modifies it by adding user specified numbers of elements to the front of container. The actual element is also supplied by user (for instance num_values =4 element =-1.5) ; this would result in four elements of -1.5 are added to begining of deque)  [container_ops.cpp](./a/container_ops.cpp)
2) TASK: Implement function t`void bubbleSortContainer( std::deque<double>& container)` that accepts a deque container and rearranges elements by bubble sort operation An example of C++ code for arrays is here https://www.programiz.com/dsa/bubble-sort 

