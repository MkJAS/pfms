#include "ranger.h"
#include <random>
#include <chrono>

Ranger::Ranger()
{
    maxrange_ = 8;
    minrange_ = 0.2;
    fov_ = 0;
    pose_.x = 0;
    pose_.y = 0;
    pose_.theta = 0;
    resolution_ = 10;
    method_ = ranger::SensingMethod::POINT;
    fov_set_ = false;
}

std::vector<double> Ranger::generateData()
{
    std::vector<double> data;

    long int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::normal_distribution<double> datarnd(5,6);
    double range = 0;
    for(unsigned int i=0;i<numReadings_;i++)
    {
        range = datarnd(gen);
        while(range>maxrange_ || range<minrange_)
        {
            range = datarnd(gen);
        }
        data.push_back(range);
        numSamples_++;
    }
    return data;
}

unsigned int Ranger::getAngularResolution(void)
{
    return resolution_;
}

bool Ranger::setAngularResolution(unsigned int resolution)
{
    if (method_ == ranger::SensingMethod::POINT)
    {
        if(resolution == 10 || resolution == 30)
        {
            resolution_ = resolution;
            numReadings_ = 1+180/resolution_;
            return true;
        }
        else 
        {
            return false;
        }
    }
    else 
    {
        resolution_ = 20;
        return false;
    }
    
}

ranger::SensorPose Ranger::getSensorPose(void)
{
    return pose_;
}

bool Ranger::setSensorPose(ranger::SensorPose pose)
{
    pose_.x = pose.x;
    pose_.y = pose.y;
    pose_.theta = pose.theta;

}

unsigned int Ranger::getFieldOfView(void)
 {
     return fov_;
 }

bool Ranger::setFieldOfView(unsigned int fov)
{
    if (fov_set_)
    {
        fov_ = fov;
        return true;
    }
    return false;
    
}

double Ranger::getMaxRange(void)
{
    return maxrange_;
}

double Ranger::getMinRange(void)
{
    return minrange_;
}

ranger::SensingMethod Ranger::getSensingMethod(void)
{
    return method_;
}

double Ranger::getSequenceNumber(void)
 {
     return numSamples_;
 }

void Ranger::getSetParams(void)
{

}


