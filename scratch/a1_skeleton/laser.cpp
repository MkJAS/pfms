#include "laser.h"

Laser::Laser()
{
    maxrange_ = 8;
    minrange_ = 0.2;
    resolution_ = 30;
    modelname_ = "SICK-XL";
    fov_ = 0;
    method_ = (ranger::SensingMethod::POINT);
    pose_.x = 0;
    pose_.y = 0;
    pose_.theta = 0;
    numReadings_ = 19;
    numSamples_ = 0;
    fov_set_ = false;
}


void Laser::getSetParams(void)
{
    char *MethodType[] =
    {
        "Cone",
        "Point"        
    };

    std::cout<<"Model name: "<<modelname_<<std::endl;
    std::cout<<"Sensing method: "<<MethodType[method_]<<std::endl;
    std::cout<<"Range: "<<minrange_<<"-"<<maxrange_<<" meters"<<std::endl;
    std::cout<<"Field of view: 180 degrees with variable resoltion."<<std::endl;
    std::cout<<"Can fov be set: "<<std::boolalpha<<fov_set_<<std::endl;

}