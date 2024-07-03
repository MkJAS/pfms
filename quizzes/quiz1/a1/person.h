#ifndef PERSON_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define PERSON_H

#include <string>
#include <iostream>

class Person {
public:
    //    1) TASK The `Person` class is missing a special member function. 
    //    This function needs to enable crteating an object of
    //    `Person` class with the `name` , `age` and `vacinated` 
    //    initialised with values supplied by user of the class.
    //    You will need to add the declaration of this member function in the
    //    [header of Person class](./a1/person.h) as
    //    well as implement this function in the [implementation file of Person class](./a1/person.cpp).
  
  Person(std::string name,unsigned int age, bool vacinated);
  
  /**
   * @brief Function that obtains name
   * @return name of person
   */
  std::string getName(void);

  /**
   * @brief Function that retruns the age
   * @return age in years
   */
  unsigned int getAge(void);

  /**
   * @brief Function that retruns vacinated status
   * @return vacinated status
   */
  bool getVacinated(void);

  /**
   * @brief Increments one age (birthday)
   */
  void birthday(void);

  /**
   * @brief Function that vacinates a person if they have not previously been vacinated
   * @return indicates if a person should be vacinated (true is they have not already been vacinated)
   */
  bool vacinate();



  

private:
  std::string name_; //!< name of person
  unsigned int age_; //!< age in years
  bool vacinated_;  //!< vacinated status (true - has been given vacine, false - not vacinated)
};


#endif // PERSON_H
