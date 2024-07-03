#include "analysis.h"

#include <iostream> // Only here for showing the code is working

#include <chrono>


Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{

}

//! @todo
//! TASK 1 and 2 - Same implementation, just being called twice Refer to README.md and the Header file for full description
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed)
{
  
  double diffs = 0;
  //
  for(int i=0;i<samples;i++)
  {
    auto start = std::chrono::steady_clock::now();
    //auto start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::vector<double> data = radarPtr_->getData();
    //auto end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - start;
    diffs = diffs + diff.count();
  }

  //
  

  scanningSpeed = diffs/samples;

  
  return;
}
