#include "bogieposAdv.h"
#include "timer.h"
#include "tf.h"
#include "types.h"

BogieposAdv::BogieposAdv(){}

BogieposAdv::BogieposAdv(std::shared_ptr<Simulator> &sim,std::shared_ptr<Timer> &timer)
{
    sim_ = sim;
    timer_ = timer;
    getvelocities_ = false;
    basic = false;
}

BogieposAdv::~BogieposAdv(){
  flying=false;
  //Join threads
  for(auto &t: threads_){ //Alen to check & why?
    t.join();
  }
}

void BogieposAdv::start()
{
    flying = true;
    threads_.push_back(std::thread(&BogieposAdv::calcbogieglobalandfriendly,this));
    threads_.push_back(std::thread(&BogieposAdv::calcbogieVel,this));
    //threads
}

std::map<int,velocity> BogieposAdv::calcbogieVel()
{
  
  while(flying)
  {
  int check = 0;
  std::vector<std::pair<Point,double>> t0;
  std::vector<std::pair<Point,double>> t1;
  std::vector<double> times0;
  std::vector<double> times1;
  int id = 0;
  bogie_ids.clear();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  while(check < 2)
  {
    timer_.reset();
    if(check == 0)
    {
      
      t0 = getbogieglobal();
      times0 = gettimes();
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      
    }
    if(check == 1)
    {
      
      times1 = gettimes();
      t1 = getbogieglobal();
      check = 2;
    }
    check++;
  }
  std::vector<std::pair<int,int>> markers = associate(t0,t1);
  std::vector<Pose> poses;

  for(auto it:markers)
  {
    double pose;
    velocity vel = getvel(it,t1,t0,times1,times0,pose);
    
   
    bogie_ids.insert({id,vel});
    id++;
   
    //pose.push_back({it.first,{0,0,0,0}});
    Quaternion quat = tf::yawToQuaternion(pose);
    Point pt = {t1[it.second].first.x,t1[it.second].first.y,t1[it.second].first.z};
    poses.push_back({pt,quat}); 
  }
  
  sim_->testPose(poses);
  check = 0;
  t0.clear();
  t1.clear();
  times0.clear();
  times1.clear();
  id = 0;
  return bogie_ids;
  }
}


std::vector<std::pair<int,int>> BogieposAdv::associate(std::vector<std::pair<Point,double>>& vec1,std::vector<std::pair<Point,double>>& vec2)
{
  std::vector<std::pair<int,int>> relate;
  for(int i=0;i<vec1.size();i++)
  {
    for(int j=0;j<vec2.size();j++)
    {
      if(abs(vec2[j].first.x-vec1[i].first.x)<100 || abs(vec2[j].first.y-vec1[i].first.y)<100)
      {
        if(abs(vec2[j].second-vec1[i].second)<0.2)
        {
          relate.push_back({i,j});
          break;
        }
        
      }
    }
  }
  return relate;
}

velocity BogieposAdv::getvel(std::pair<int,int>& it, std::vector<std::pair<Point,double>>& t1,std::vector<std::pair<Point,double>>& t0,std::vector<double>& times1, std::vector<double>& times0,double& pose)
{
    velocity vel;
    double diffx = t1[it.second].first.x - t0[it.first].first.x;
    double diffy = t1[it.second].first.y - t0[it.first].first.y;
    double difft = times1[it.second] - times0[it.first];
    
    vel.vx = (diffx/difft)*1000;
    vel.vy = (diffy/difft)*1000;
    vel.m = atan(diffy/diffx);
    vel.time = times1[it.second];
    vel.x0 = t1[it.second].first.x;
    vel.y0 = t1[it.second].first.y;
    if(diffx<0 && diffy<0)
    {
      pose = M_PI + vel.m;
    }
    if(diffx>0 && diffy>0)
    {
      pose = vel.m;
    }
    if(diffx<0 && diffy>0)
    {
      pose = M_PI + vel.m;
    }
    if(diffx>0 && diffy<0)
    {
      pose = 2*M_PI + vel.m;
    }

    return vel;
}
