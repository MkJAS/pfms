#include "ranger.h"
#include "laser.h"
#include "sonar.h"
#include "rangerfusion.h"
#include "cell.h"
#include <iostream>
#include <chrono>
#include <thread>




int main ()
{
    std::vector<RangerInterface*> sensors;
    Laser ls1;
    Sonar snr1;
    Sonar snr2;
    sensors.push_back(&ls1);
    sensors.push_back(&snr1);
    sensors.push_back(&snr2);
    
    std::cout<<"\nSensor 1"<<std::endl;
    ls1.getSetParams();
    std::cout<<"\nSensor 2"<<std::endl;
    snr1.getSetParams();
    std::cout<<"\nSensor 3"<<std::endl;
    snr2.getSetParams();
    
    std::cout<<"\n"<<std::endl;

    sensors[0]->setSensorPose({1,2,0});
    sensors[2]->setSensorPose({-3,3,(-45*M_PI/180)});
    
    if(!sensors[0]->setAngularResolution(35))
    {
        std::cout<<"Invalid Resolution. Previous value set. Refer to model specifications."<<std::endl;
    }
    //sensors[2]->setSensorPose({4,1,0});

    std::cout<<"How many cells to generate? Enter a number"<<std::endl;
    double numcells = 0;
    std::cin>>numcells;

    std::vector<Cell*> Cells;
    double x;
    double y;
    for (int i=0;i<numcells;i++)
    {
        Cells.push_back(new Cell());
        Cells[i]->getCentre(x,y);
        Cells[i]->setSide(0.6);
        std::cout<<x<<" "<<y<<std::endl;
    }
   
    std::cout<<"=================="<<std::endl;
    
    RangerFusion Fusion(sensors);
    Fusion.setCells(Cells);
    std::cout<<"TOTAL SONAR AREA: ";
    std::cout<<Fusion.getScanningArea()<<std::endl;

    std::cout<<"Cell state Key:\nUNKNOWN = 0\nFREE = 1\nOCCUPIED = -1"<<std::endl;
    std::chrono::seconds loop(1);
    while(1!=0)
    {
        Fusion.grabAndFuseData();
        for (auto it:Cells)
        {
            std::cout<<it->getState()<<std::endl;
        }
        std::this_thread::sleep_for(loop);
        std::cout<<"\n"<<std::endl;
        std::cout<<"Sonar 1 sequence number: "<<snr1.getSequenceNumber()<<std::endl;
        std::cout<<"Sonar 2 sequence number: "<<snr2.getSequenceNumber()<<std::endl;
        std::cout<<"Laser 1 sequence number: "<<ls1.getSequenceNumber()<<std::endl;
        
    }
    
}