#include "processing.h"
// You can add additional headers needed here, such as
#include <limits>

//3) TASK Implement a function that returns the `Person` who is the oldest member of the `crowd`.
std::vector<Person> oldestPerson(std::vector<Person> crowd)
{
    int oldest_index = 0;
    for (int i = 0; i < crowd.size();i++)
    {
        if (crowd[oldest_index].getAge()<crowd[i].getAge())
        {
            oldest_index = i;
        }
    }
    std::vector<Person> oldest = {crowd[oldest_index]};
return oldest;
    
}


//4) TASK Implement a function that returns the `people` from the `crowd` that need to be vaccinated.
//Criteria is that they are older then the specified `ageCuttoff` and have not been previously vaccinated.
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff)
{
    std::vector<Person> iselig;

    for (int i=0;i<crowd.size();i++)
    {
        if (crowd[i].getAge()>ageCutoff && crowd[i].getVacinated())
        {
            iselig.push_back(crowd[i]);
        }
    }
    return iselig;

}
