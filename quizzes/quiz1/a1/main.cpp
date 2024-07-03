#include <iostream> // Includes std::cout and friends so we can output to console
#include "person.h"
#include "processing.h"

int main (void) {


    //! TASK
    //! 3) Create a vector of Person's (crowd)
    //!
    //! --  With below 5 elements --
    //! Alice, 32 year old, vacinated
    //! Bob, 62 year old male, unvacinated
    //! Carol, 72 year old female, vacinated
    //! John, 82 year old male, unvacinated
    //! Karen, 42 year old female, vacinated

    Person Alice("Alice",32,false);
    Person Bob("Bob",62,true);
    Person Carol("Carol",72,false);
    Person John("John",82,true);
    Person Karen("Karen",42,false);


    std::vector<Person> crowd;
    crowd.push_back(Alice);
    crowd.push_back(Bob);
    crowd.push_back(Carol);
    crowd.push_back(John);
    crowd.push_back(Karen);

    std::vector<Person> oldest = oldestPerson(crowd);
    //std::cout<<oldest[0].getName()<<std::endl;

    //! 4) TASK
    //!
    //! specify and age cutoff and call
    unsigned int ageCutoff = 50;
    std::vector<Person> eligible = eligibleForVaccine(crowd, ageCutoff);

    //std::cout<<eligible[0].getName()<<std::endl;
    //std::cout<<eligible[1].getName()<<std::endl;
    

  return 0;
}
