#include <iostream>
#include <thread>
#include <vector>

#include "radar.h"
#include "dataprocessing.h"
#include "fuck.h"

int main (void){

  //! We create a vector of pointers to Radar
  std::vector<Radar*> radars;
  //! Push pack 3 radars
  // radars.push_back(new Radar);
  // radars.push_back(new Radar);
  // radars.push_back(new Radar);

  // //! Start thraed of each radar
  // for (auto radar : radars){
  //   radar->start();
  // }
  std::shared_ptr<Fuck> fuckPtr(new Fuck);
  std::shared_ptr<Radar> radarPtr(new Radar(fuckPtr));
  //! Created a pointer to data processing
  std::shared_ptr<DataProcessing> dataProcessingPtr(new DataProcessing(radarPtr));
  //! Create thread
   std::thread processing_thread(&DataProcessing::findClosestReading,dataProcessingPtr);

   std::this_thread::sleep_for (std::chrono::milliseconds(1000));
   //! Pass the radars
   dataProcessingPtr->setRadars(radars);

    std::cout<<"Check: "<<radarPtr->check<<std::endl;
    dataProcessingPtr->radar_->check = 20;
    std::cout<<"Check: "<<radarPtr->check<<std::endl;

    std::cout<<"Fuck: "<<fuckPtr->getx()<<std::endl;
    dataProcessingPtr->fuck_->setx(32);
    std::cout<<"Fuck: "<<fuckPtr->getx()<<std::endl;






  //! Join thread when we have finished
  processing_thread.join();
//! Call destructors for each radar
  for (auto radar : radars){
    delete radar;
  }


  return 0;
}

