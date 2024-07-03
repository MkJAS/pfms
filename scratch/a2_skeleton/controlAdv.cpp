#include "controlAdv.h"

#include <math.h>
#include "tf.h"
#include <time.h>
#include "timer.h"
#include <chrono>


using namespace tf;

ControlAdv::ControlAdv()
{
    // quickest_branch_time = 1000;
    // starting_node_ = 0;
    // gotglobal = false;
    // gotbogies_ = false;

}

ControlAdv::ControlAdv(std::shared_ptr<Simulator> &sim,std::shared_ptr<Timer> &timer)
{
    sim_ = sim;
    timer_ = timer;
    gotglobal_ = false;
    gotbogies_ = false;
    startmove_ = false;
    gotbogievels_ = false;
    gotpath_ = false;
    getvel_ = false;
    restart_ = false;
    base_start = true;
    lin_vel_ = MIN_vel;
    ang_vel_ = MAX_ang;
    opt_path_.resize(4,0);

}

ControlAdv::~ControlAdv()
{
    running_ = false;
    for(auto &t:threads_)
    {
        t.join();
    }
}

void ControlAdv::start()
{
    running_ = true;
    threads_.push_back(std::thread(&ControlAdv::idle,this));
    //threads_.push_back(std::thread(&Control::move,this));
    threads_.push_back(std::thread(&ControlAdv::calcintercept,this));
    //threads_.push_back(std::thread(&ControlAdv::optvelocities,this));
}


void ControlAdv::setbogiexyth(std::map<int,velocity>& bogievel)
{
   if(!gotbogievels_)
   {
    bogieVels_ = bogievel;
    gotbogievels_ = true;
    velcv_.notify_all();
    restart_ = true;
   }

}

void ControlAdv::calcintercept()
{
    while(running_)
    {
        lin_vel_ = MIN_vel;
        ang_vel_ = MAX_ang*0.5;
        std::unique_lock<std::mutex> lck(velmtx_);
        velcv_.wait(lck,[&](){return gotbogievels_==true;});
        int count = 0;
        
        for(int i=0;i<bogieVels_.size();i++)
        {
            restart_ = false;
            lin_vel_ = MIN_vel;
            ang_vel_ = MAX_ang*0.5;
            double t = (timer_->elapsed())/1000;
            velocity velb = bogieVels_[i];
            velb.x0 = velb.vx*t + velb.x0;
            velb.y0 = velb.vy*t + velb.y0;
            double v_lin = 500;
            Point point = sim_->getFriendlyPose().position;
            double orient = quaternionToYaw(sim_->getFriendlyPose().orientation);
            lin_vel_ = sim_->getFriendlyLinearVelocity();
            double vfx;
            double vfy;
            if(orient<0){orient = orient+2*M_PI;}
            double v_O;
            velocity_ = 500;
            bool check = true;
            double T = t + 1.5;
            bxT_;
            byT_;
            bT_theta_;
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            while(check)
            {   
                double tx;
                double ty;     
                extrapbogiexy(velb,T,ty,tx);
                bxT_ = tx;
                byT_ = ty;
                double dx = bxT_ - point.x;
                double dy = byT_ - point.y;
                bT_theta_ = atan2(dy,dx);
                if(bT_theta_<0){bT_theta_=bT_theta_+2*M_PI;}
                Vtheta_ = getFriendlyVxVy(bT_theta_);
                //orient = quaternionToYaw(sim_->getFriendlyPose().orientation);
                //if(orient<0){orient = orient +2*M_PI;}
                double turntime = abs((bT_theta_ - orient))/MAX_ang;
                while(turntime>T)
                {
                    T = turntime + 2;
                    bxT_ = velb.vx*T + velb.x0;
                    byT_ = velb.vy*T + velb.y0;
                    dx = bxT_ - point.x;
                    dy = byT_ - point.y;
                    bT_theta_ = atan2(dy,dx);
                    if(bT_theta_<0){bT_theta_=bT_theta_+2*M_PI;}
                    Vtheta_ = getFriendlyVxVy(bT_theta_);
                    turntime = abs((bT_theta_ - orient))/MAX_ang;
                }
                double signx = 1;
                double signy = 1;
                if(bxT_<point.x){signx = -1;}
                if(byT_<point.y){signy = -1;}

                double fxT = signx*velocity_*cos(Vtheta_)*(T-turntime) + point.x;
                double fyT = signy*velocity_*sin(Vtheta_)*(T-turntime) + point.y;
            
                if(abs(fxT-bxT_)<50 && abs(fyT-byT_)<50)
                {
                    //lin_vel_ = velocity;
                    std::cout<<fxT<<" "<<fyT<<std::endl;
                    std::cout<<bxT_<<" "<<byT_<<std::endl;
                    check = false;
                    break;
                }
                if(abs(fxT-bxT_)>50 || abs(fyT-byT_)>50 || count>500)
                {
                    if(velocity_+15>980 || velocity_-15<50 || count>500)
                    {
                        end = std::chrono::system_clock::now();
                        std::chrono::duration<double> elapsed_seconds = end - start;
                        double fuck = elapsed_seconds.count();
                        T = T + 0.5 + fuck;
                        start = std::chrono::system_clock::now();
                        velocity_ = 500;
                        count = 0;
                    }
                    if(abs(fxT)>abs(bxT_)+60 || abs(fyT)>abs(byT_)+60)
                    {
                        velocity_ = velocity_ - 20;
                    }
                    else
                    {
                    velocity_ = velocity_ + 20;  
                    }
                }
                count++;
            }
            check = true;
            chase();
        }    
    }
}

void ControlAdv::chase()
{
    int num_bogie = bogieVels_.size();
    double target = bT_theta_;
    double current;
    Pose pose;
    Point xy;
    double x,y,l;
    bool check = true;

    if(target<0){target = target+2*M_PI;}
    double lower = target-0.1;
    if(lower<0){lower = lower + 2*M_PI;}
    double upper = target+0.1;
    if(upper>2*M_PI){upper = upper - 2*M_PI;}
    if(lower>upper || upper<lower)
    {
        double temp = lower;
        double tempp = upper;
        upper = temp;
        lower = tempp;
    }
    lin_vel_ = MIN_vel;
    ang_vel_ = MAX_ang*0.5;
    double s = sim_->rangeBearingToBogiesFromFriendly().size();
    
    while(s == num_bogie)
    {
        current = quaternionToYaw(sim_->getFriendlyPose().orientation);
        std::cout<<current<<std::endl;
        std::cout<<lower<<std::endl;
        std::cout<<upper<<std::endl;
        if(current<0){current = current +2*M_PI;}
        if(current>lower && current<upper)
        {
            pose = sim_->getFriendlyPose();
            std::cout<<"CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"<<std::endl;
            xy.x = pose.position.x;
            xy.y = pose.position.y;
            current = quaternionToYaw(sim_->getFriendlyPose().orientation);
            if(current<0){current = current +2*M_PI;}
            x = (bxT_-xy.x)*cos(current)+(byT_-xy.y)*sin(current);
            y = -1*(bxT_-xy.x)*sin(current)+(byT_-xy.y)*cos(current);
            l = pow((pow(x,2)+pow(y,2)),0.5);
            gamma_ = 2*y/(l*l);
            getvel_ = true;
            optvelocities();                          
        }
        s = sim_->rangeBearingToBogiesFromFriendly().size();     
    }
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"<<std::endl;
    lin_vel_ = MIN_vel;
    ang_vel_ = MAX_ang*0.9;
}

double ControlAdv::getFriendlyVxVy(double orient)
{
    double v_O;
    if(orient<M_PI/2)
    {
        v_O = orient;
    }
    if(orient>M_PI/2 && orient<M_PI)
    {
        v_O = M_PI - orient;
    }
    if(orient>M_PI && orient<1.5*M_PI)
    {
        v_O = orient - M_PI;
    }
    if(orient>1.5*M_PI && orient<2*M_PI)
    {
        v_O = 2*M_PI - orient;
    }
    return v_O;
}

void ControlAdv::optvelocities()
{
    // while(running_)
    // {
    // std::unique_lock<std::mutex> lck(Velmtx_);
    // Velcv_.wait(lck,[&](){return getvel_==true;});
    std::cout<<"VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV"<<std::endl;
    std::cout<<"Gamma "<<gamma_<<std::endl;
    //lck.unlock();
    double V = velocity_;
    double W = gamma_*V;
    while(abs(V*W) > MAX_VA)
    {
        W = W - 0.005;
        W = gamma_*V;
    }
    //startmove_ = true;
    getvel_ = false;
    lin_vel_ = velocity_;
    ang_vel_ = W;
    std::cout<<"VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV"<<std::endl;
    std::cout<<"Lin vel "<<lin_vel_<<std::endl;
    std::cout<<"Ang vel "<<ang_vel_<<std::endl;
    //}

}

void ControlAdv::idle()
{
    while(running_)
    {        
        //Feed the watchdog control timer
        command(lin_vel_, ang_vel_);
    }
}

void ControlAdv::extrapbogiexy(velocity vel,double T,double& ty,double& tx)
{
    tx = vel.vx*T + vel.x0;
    ty = vel.vy*T + vel.y0;
}
