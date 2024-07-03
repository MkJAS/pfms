Quiz 1
======

Part A1
------

1) TASK
The `Person` class is missing a special member function. This function needs to enable crteating an object of `Person` class with the `name` , `age` and `vacinated` initialised with values supplied by user of the class. You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as well as implement this function in [implementation file of Person class](./a1/person.cpp).
2) TASK
Implement the method `vacinate` in `Person` class. This function retruns a `bool` indicating if a person can be given a vaccine. The person can be given a vacine, if they are not already vaccinated.  When a person is given a vacine, their `vacinated` status should change.


3) TASK
Implement [oldestPerson](./a1/processing.h) function that returns the `Person` who is the oldest person(s) of the `crowd`. Create a `crowd` using a STL vector container of people, populate it with 5 people. To test you can use details of the people are in the [main](./a1/main.cpp) or improvise.

4) TASK
Implement [eligibleForVacine](./a1/processing.h) function that returns the `people` from the `crowd` that need to be vaccinated. Criteria is that they are older then the specified `ageCuttoff` and have not been previously vaccinated.

Part A2
------

1) TASK
Modify the file rectangle [rectangle](./a2/rectangle.h) so it inherits from the base class of shape [shape](./a2/shape.h). Correct the missing access specifiers of base class [shape](./b2/shape.h) so that the immplementaion of the [`Rectangle` constructor](./a2/rectangle.cpp) can still access the required member varaible of `Shape` but have it restricted to users of a `Rectangle` object.

2) TASK
The `Rectangle` class already has a special member function. Thanks to *Polymorphism* we can have functions with same name but different number of parameters. Can you add to the `Rectangle` class another function that enables the `Rectangle` to on creation have `width` , `height` and `description` initialised with values supplied by user of the class. You will need to add the declaration of this member function in the
[header of Rectangle class](./a2/rectangle.h) as well as implement this function in [implementation file of Rectangle class](./a2/rectangle.cpp).

