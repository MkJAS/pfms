#include "sonar.h"

Sonar::Sonar()
{
    maxrange_ = 10;
    minrange_ = 0.2;
    resolution_ = 0;
    modelname_ = "SN-001";
    fov_ = 20;
    method_ = (ranger::SensingMethod::CONE);
    pose_.x = 0;
    pose_.y = 0;
    pose_.theta = 0;
    numReadings_ = 1;
    numSamples_ = 0;    
    fov_set_ = false;
}

void Sonar::getSetParams(void)
{
    char *MethodType[] =
    {
        "Cone",
        "Point"        
    };

    std::cout<<"Model name: "<<modelname_<<std::endl;
    std::cout<<"Sensing method: "<<MethodType[method_]<<std::endl;
    std::cout<<"Range: "<<minrange_<<"-"<<maxrange_<<" meters"<<std::endl;
    std::cout<<"Field of view: "<<fov_<<std::endl; 
    std::cout<<"Can fov be set: "<<std::boolalpha<<fov_set_<<std::endl;
}