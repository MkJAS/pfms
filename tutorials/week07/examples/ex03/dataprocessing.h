#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <vector>
#include "radar.h"

class DataProcessing
{
public:
  DataProcessing(std::shared_ptr<Radar> radar);
  void setRadars(std::vector<Radar*> radars);
  void findClosestReading();
  std::shared_ptr<Radar> radar_;
  std::shared_ptr<Fuck> fuck_;
private:
  std::vector<Radar*> radars_;
  

};

#endif // DATAPROCESSING_H
