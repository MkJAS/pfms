#ifndef CONTROLADV_H
#define CONTROLADV_H

#include "control.h"
#include "bogieposAdv.h"



class ControlAdv : public Control
{
public:
    ControlAdv();
    ControlAdv(std::shared_ptr<Simulator> &sim,std::shared_ptr<Timer> &timer);
    ~ControlAdv();
    void start();
    void setbogiexyth(std::map<int,velocity>& bogievel);

    void extrapbogiexy(velocity vel,double T,double& ty,double& tx);
    //bool getrestart();

private:
    std::condition_variable velcv_;
    std::mutex velmtx_;

    std::shared_ptr<Timer> timer_;
    std::map<int,velocity> bogieVels_;
    double velocity_;
    double Vtheta_;
    double bT_theta_;
    double bxT_;
    double byT_;
    std::atomic<bool> gotbogievels_;

    void calcintercept();
    double getFriendlyVxVy(double orient);
    void chase();
    void optvelocities();
    void idle();
};






#endif