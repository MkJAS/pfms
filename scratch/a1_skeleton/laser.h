#ifndef LASER_H
#define LASER_H

#include "ranger.h"

class Laser: public Ranger
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Laser();
  void getSetParams(void);
};

#endif // LASER_H
