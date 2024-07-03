#include "person.h"

//    1) TASK The `Person` class is missing a special member function. This function needs to enable crteating an object of
//    `Person` class with the `name` , `age` and `vacinated` initialised with values supplied by user of the class.
//    You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as
//    well as implement this function in the [implementation file of Person class](./a1/person.cpp).
Person::Person(std::string name,unsigned int age, bool vacinated)
{
  name_ = name;
  age_ = age;
  vacinated_ = vacinated;
}

std::string Person::getName(void) {
  return name_;
}

unsigned int Person::getAge(void) {
  return age_;
}


bool Person::getVacinated(void) {
  return vacinated_;
}

//2) TASK
//Implement the method `vacinate` in `Person` class.
//This function retruns a `bool` indicating if a person can be given a vaccine.
//The person can be given a vacine, if they are not already vaccinated.
//
//When a person is given a vacine, their `vacinated` status should change.
bool Person::vacinate(void){
  
  bool isvacsed = getVacinated();
  if (isvacsed)
  {
    vacinated_ = false;
  }

  return vacinated_;
  

}
