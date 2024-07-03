Week 4 Prework Questions
=========================
Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material please raise them in the next tutorial session.

This buildis upon solutions in week 03 (ex03) and uses [shapes](./starter/shape)

Polymorphism - Ex01
------------------

* Modify the base class `Shape` file [shape.h](./a/shape.h) such that functions `getArea()` and `checkIntercept()` is defined in `Shape`, and the child classes are required to implement this function. 
(Polymorphism and the concept of virtual).

* Create a Square, Circle and Traingle, and store them in a vector of `Shape` pointers
* Create a function that loops through shapes and display their area.

Polymorphism - Ex02
------------------

Building upon your previous solution in the main

* Allow the user to specify number of circles, triangles and rectangles and `max_length`.
* Create the shapes with random lengths to be capped to `max_length` and `setCentre` for each shape to a random values capped at `max_length/2`
* Implement the checkIntercept funtion for [Isosceles_triangle](https://en.wikipedia.org/wiki/Isosceles_triangle) in `triangle.cpp`
* Allow the user to specify a point to be used for intercept checking `x` and `y`
* Write a function that check if  the shapes intersect the point

Unit Testing - Intro
------------------

Review the Unit Testing material from Canvas from [IBM](https://developer.ibm.com/articles/au-googletestingframework/), even IBM uses Google Tests :grin:
* Write a test for Zero And Negative Numbers

