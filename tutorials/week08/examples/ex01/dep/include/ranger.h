#ifndef RANGER_H
#define RANGER_H

#include "rangerinterface.h"
#include <string>
#include <sstream>
#include <random>
#include <chrono>
#include <cmath>


const double DATA_MEAN = 4.0;
const double DATA_STDEV = 5.0;


/**
 * @brief Parent Class for range-finding sensor types. Provides basic get/set functionality, and data generation methods.
 * 
 */
class Ranger: public RangerInterface
{
    public:
        //Default constructors should set all sensor attributes to a default value
        Ranger();
        ~Ranger();

        std::vector<double> generateData();

        //Essential getters for obtaining internal private variables
        unsigned int getAngularResolution(void);
        ranger::SensorPose getSensorPose(void);
        unsigned int getFieldOfView(void);
        double getMaxRange(void);
        double getMinRange(void);
        double getSampleNumber(void);
        ranger::SensingMethod getSensingMethod(void);
        std::string getFixedParameters();

        //Essential setters for setting internal private variables
        virtual bool setAngularResolution(unsigned int r) = 0;
        virtual bool setFieldOfView(unsigned int f) = 0;
        bool setSensorPose(ranger::SensorPose);


    protected:
        ranger::SensingMethod sensor_type_;
        std::string model_;
        unsigned int fov_;
        unsigned int angular_resolution_;
        double max_range_;
        double min_range_;
        ranger::SensorPose sensor_pose_;
    private:
        double constrainRange(double v);
        double constrainAngle(double v);
        std::default_random_engine * generator;
        std::normal_distribution<double> * range_distribution;

        double sample_number_;

};

#endif // RANGER_H
