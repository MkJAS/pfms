#ifndef BOGIEPOSADV_H
#define BOGIEPOSADV_H

#include "bogiepos.h"

using namespace geometry_msgs;

struct velocity
{
    double vx;
    double vy;
    double m; //m for gradient
    double x0;
    double y0;
    long time;
};



class BogieposAdv : public Bogiepos
{
public:
    BogieposAdv();
    BogieposAdv(std::shared_ptr<Simulator> &sim,std::shared_ptr<Timer> &timer);
    ~BogieposAdv();


    void start();
    std::map<int,velocity> calcbogieVel();
    velocity getvel(std::pair<int,int>& it, std::vector<std::pair<Point,double>>& t1,std::vector<std::pair<Point,double>>& t0,std::vector<double>& times1, std::vector<double>& times0,double& pose);

protected:
    std::shared_ptr<Timer> timer_;

    std::map<int,velocity> bogie_ids;
    std::condition_variable Velcv_;
    std::mutex Velmtx_;
    bool getvelocities_;
    std::vector<std::pair<int,int>> associate(std::vector<std::pair<Point,double>>& vec1,std::vector<std::pair<Point,double>>& vec2);

};

#endif