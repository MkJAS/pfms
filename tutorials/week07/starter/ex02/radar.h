#ifndef RADAR_H      // An 'include guard' to prevent double declaration of any identifiers in this library
#define RADAR_H

#include <string>
#include <vector>
#include <random>

class Radar{
    public:

    Radar();
    std::vector<double> getData(void);	            // return a vector of targets, blocking call function
    double getScanningTime(void);                   // get Scanning Time
    void setScanningTime(double scanningTime);      // set Scanning Time
    void start();                                   // start the thread that awaits for data

private:
    void generateData(void);
    std::vector<double> data_;                 //targets as a vector
    double scanningTime_;
    const float maxDistance_ = 80.0;
    const unsigned int numTarget_ = 20;
    std::mt19937 *generator_;
    std::uniform_real_distribution<double> *value_;

};

#endif // SENSOR_H
