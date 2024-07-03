#include "bogiepos.h"
#include <math.h>
#include "dep/include/tf.h"

using namespace tf;
double degtorad = M_PI/180;

Bogiepos::Bogiepos()
{
    globalbogiesgot_ = false;
    friendbogiesgot_ = false;
    flying = true;

}

Bogiepos::Bogiepos(std::shared_ptr<Simulator> &sim)
{
    sim_ = sim;
    globalbogiesgot_ = false;
    friendbogiesgot_ = false;
    gottimes_ = false;
    flying = true;
    basic = true;
}

Bogiepos::~Bogiepos(){
  flying=false;
  //Join threads
  for(auto &t: threads_){ //Alen to check & why?
    t.join();
  }
}

void Bogiepos::start()
{
    flying = true;
    threads_.push_back(std::thread(&Bogiepos::calcbogieglobalandfriendly,this));
}

void Bogiepos::getfriendinfo()
{
        pos_friend_.x = sim_->getFriendlyPose().position.x;
        pos_friend_.y = sim_->getFriendlyPose().position.y;
        friend_theta_ = quaternionToYaw(sim_->getFriendlyPose().orientation);
        if(friend_theta_<0){friend_theta_=friend_theta_+2*M_PI;}
}

std::pair<Point,double> Bogiepos::calcglobalxy(Point rngbr, Point pos_frnd,double theta)
{
    Point Temp;
    double bearing;
    Temp.x = rngbr.x*cos(theta)-rngbr.y*sin(theta) + pos_frnd.x; //convert local coords to global
    Temp.y = rngbr.x*sin(theta)+rngbr.y*cos(theta) + pos_frnd.y;
    Temp.z = pow((pow(Temp.x,2)+pow(Temp.y,2)),0.5);
    bearing = atan2(Temp.y,Temp.x);                 //get bearing of bogie from base
    if(bearing<0){bearing = bearing + 2*M_PI;}      //convert to 0-360 deg
    //Pose pose = {Temp,{0,0,0,0}};
    
    return std::make_pair(Temp,bearing);
}

Point Bogiepos::calcfrndxy(std::pair<double,double> rngbr)
{
    Point Temp;
    Temp.x = rngbr.first*cos(2*M_PI-rngbr.second);
    Temp.y = rngbr.first*sin(M_PI-rngbr.second);
    Temp.z = rngbr.first; //Range
    return Temp;
}

void Bogiepos::calcbogieglobalandfriendly()
{
    std::vector<std::pair <double,double>> range_bear;    
    while(flying)
    {
        globalbogiesgot_ = false;
        friendbogiesgot_ = false;
        getfriendinfo();
        Point temp_pos_friend = pos_friend_;
        double temp_theta = friend_theta_;
        range_bear.clear();
        glb_bogies_pos_.clear();
        frnd_bogies_pos_.clear();
        rbf_ = sim_->rangeBearingToBogiesFromFriendly();
        
        std::unique_lock<std::mutex> timelck(timemtx_);
        times_.clear();
        for(auto it:rbf_) //take range and bearing readings out from vector and separate the timestamps
        {
            std::pair<double,double> temp;
            temp = std::make_pair(it.range,(it.bearing));
            range_bear.push_back(temp);
            times_.push_back(it.timestamp);
        }
        timelck.unlock(); 
        gottimes_ = true;
        Tmcv_.notify_all();

        std::unique_lock<std::mutex> lck2(mtx2_);
        for(int i=0;i<range_bear.size();i++)
        {
            Point temp = calcfrndxy(range_bear[i]);
            std::pair<Point,double> tempp;
            tempp = std::make_pair(temp,range_bear[i].second);
            frnd_bogies_pos_.push_back(tempp);
        }
        lck2.unlock();
        friendbogiesgot_ = true;
        FBcv_.notify_all();

        std::unique_lock<std::mutex> lck(mtx_);       
        for(int i=0;i<range_bear.size();i++) //boggie pos from global frame of ref given friendly pose
        {
            std::pair<Point,double> tempp = calcglobalxy(frnd_bogies_pos_[i].first,temp_pos_friend,temp_theta);
            glb_bogies_pos_.push_back(tempp);
        }
        lck.unlock();
        m_.clear();
        for(int i=0;i<glb_bogies_pos_.size();i++)
        {
            m_.insert({i,glb_bogies_pos_[i].first});
        }
        globalbogiesgot_ = true;
        GBcv_.notify_all();
        if(basic) //for avoiding testPose being called in advanced mode incorrectly
        {
            std::vector<Pose> pose;
            for(auto it:glb_bogies_pos_)
            {
                pose.push_back({it.first,{0,0,0,0}});
            }
            sim_->testPose(pose);
        }
        

    }
}
std::pair<Point,double> Bogiepos::getfriendlyxy0()
{
    Point point;
    double o;

    point = pos_friend_;
    o = friend_theta_;
    std::pair<Point,double> pair = std::make_pair(point,o);
    return pair;


}
std::vector<std::pair<Point,double>> Bogiepos::getbogieglobal()
{
    std::vector<std::pair<Point,double>> glb_bogies_pos;
    while(true)
    {
    std::unique_lock<std::mutex> lck(mtx_);
    GBcv_.wait(lck,[&](){return globalbogiesgot_==true;});
    glb_bogies_pos = glb_bogies_pos_;
    globalbogiesgot_ = false;
    lck.unlock();
    break;
    }
    return glb_bogies_pos;   
}

std::vector<double> Bogiepos::gettimes()
{
    std::vector<double> times;
    while(true)
    {
    std::unique_lock<std::mutex> lck(timemtx_);
    Tmcv_.wait(lck,[&](){return gottimes_==true;});
    times = times_;
    gottimes_ = false;
    lck.unlock();
   
    break;
    }
    return times;   
}

std::vector<std::pair<Point,double>> Bogiepos::getbogiefriendly()
{
    std::unique_lock<std::mutex> lck(mtx2_);
    FBcv_.wait(lck,[&](){return friendbogiesgot_==true;});
    std::vector<std::pair<Point,double>> frnd_bogies_pos = frnd_bogies_pos_;
    friendbogiesgot_ = false;
    lck.unlock();

    return frnd_bogies_pos;
    
}

Point Bogiepos::global2local(Point friendxy, double friendtheta, Point bogiepos)
{
    Point localxy;
    localxy.x = (bogiepos.x-friendxy.x)*cos(friendtheta)+(bogiepos.y-friendxy.y)*sin(friendtheta);
    localxy.y = -1*(bogiepos.x-friendxy.x)*sin(friendtheta)+(bogiepos.y-friendxy.y)*cos(friendtheta);
    return localxy;           
}

 std::vector<std::pair<int,int>> Bogiepos::rangebearassociate(std::vector<RangeBearingStamped> rbs,Point friendxy)
 {
    std::vector<std::pair<int,int>> assoc;
    for(int i=0;i<rbs.size();i++)
    {
        for(auto &it:m_) //can relate a given rbs given a known set of global bogie coordinate and known friendly position
        {                //since the distance from a bogie to the friendly must be the same
            double x = it.second.x - friendxy.x;
            double y = it.second.y - friendxy.y;
            double r = pow((pow(x,2)+pow(y,2)),0.5);
            if(r >= rbs[i].range - 100 && r <= rbs[i].range + 100)
            {
                assoc.push_back(std::make_pair(it.first,i));
            }
        }
    }
    return assoc;
 }

std::vector<std::pair<int,int>> Bogiepos::rangevelassociate(std::vector<RangeVelocityStamped> rvs)
 {
    std::vector<std::pair<int,int>> assoc;
    for(int i=0;i<rvs.size();i++)
    {
        for(auto &it:m_)
        {
            double x = it.second.x;
            double y = it.second.y;
            double r = pow((pow(x,2)+pow(y,2)),0.5);
            if(r >= rvs[i].range - 100 && r <= rvs[i].range + 100)
            {
                assoc.push_back(std::make_pair(it.first,i));
            }
        }
    }
    return assoc;
 }

 std::map<int,Point> Bogiepos::getmap()
 {
     return m_;
 }


