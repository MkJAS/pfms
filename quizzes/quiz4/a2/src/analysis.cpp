#include "analysis.h"

#include <iostream> // Only here for showing the code is working

namespace analysis {

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
unsigned int countCharacters(std::string sentence)
{
    unsigned int count = 0;
    std::string s = sentence;
    count = s.size();
    return count;
}

//! @todo
//! TASK 5 - Refer to README.md and the Header file for full description
int getNumber(std::string sentence){
    unsigned int num = 0;

    std::string s = sentence;
    for ( unsigned int i =0;i < s.size(); i++ )
    { 
        if(isdigit(s[i]))
        {
            num = s[i];
            break;
    
        }
    }

    return num;

}

}
