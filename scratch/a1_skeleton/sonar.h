#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"

class Sonar: public Ranger
{
public:
  //Default constructor should set all sensor attributes to a default value
  Sonar();
  void getSetParams(void);
};

#endif // SONAR_H
