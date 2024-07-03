

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <vector>

#include "analysis.h"

void printOrderStats(std::vector<Car*> cars,std::vector<unsigned int> order){
    for (unsigned int i=0;i<cars.size();i++){
         std::cout << cars.at(i)->getMake() << " " << cars.at(i)->getModel() <<
                      " odo:" << cars.at(i)->getOdometry() <<
                      " position:" << order.at(i) << std::endl;
    }
}

int main (void) {


    std::vector<Car*> cars;

    //! @todo
    //! TASK 1
    //! Create 3 cars with follwing specifications
    
    // Mercedes - C180
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg
    Car car1("Mercedes","C180",1.45,1.77,143,0.29,1200);
    // Bugatti - Veyron
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg
    Car car2("Bugatti","Veyron",1.19,2,1200,0.35,2200);
    // Toyota - Yaris_WRsC
    // height = 1.19 m, width = 1.87 m, power P = 420 HP, drag coefficient = 0.30, weight = 1190 kg
    Car car3("Toyota","Yarris_WRsC",1.19,1.87,420,0.3,1190);

    cars.push_back(&car1);
    cars.push_back(&car2);
    cars.push_back(&car3);
    

    std::shared_ptr<DisplayRace> raceDisplay(new DisplayRace(cars));

    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars,raceDisplay));

    // The below is just a demo race, to show how to accelerate and decelerate the cars.
    // You have to keep accelerating to go fast, and decelerate to slow down
    //analysisPtr->demoRace();

    // We call TASK 1
    {
        std::vector<unsigned int> order = analysisPtr->sortByOdometry();
        printOrderStats(cars,order);
        std::cout<<"\n"<<std::endl;
    }

    // We call TASK 2
    {
        std::vector<unsigned int> order = analysisPtr->dragRace(1000.0);
        printOrderStats(cars,order);
        std::cout<<"\n"<<std::endl;
    }

    //We call TASK 3
    analysisPtr->stopAllCars();

    // We call TASK 4
    {
        std::vector<unsigned int> order = analysisPtr->zeroTopZeroRace();
        printOrderStats(cars,order);
    }



    return 0;
}
