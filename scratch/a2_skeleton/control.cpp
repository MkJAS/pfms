#include "control.h"
#include <math.h>
#include "tf.h"
#include <time.h>

using namespace tf;

Control::Control()
{
    // quickest_branch_time = 1000;
    // starting_node_ = 0;
    // gotglobal = false;
    // gotbogies_ = false;

}

Control::Control(std::shared_ptr<Simulator> &sim)
{
    sim_ = sim;
    gotglobal_ = false;
    gotbogies_ = false;
    startmove_ = false;
    gotpath_ = false;
    getvel_ = false;
    restart_ = false;
    base_start = true;
    lin_vel_ = 50;
    ang_vel_ = MAX_ang;
    opt_path_.resize(4,0);

}

Control::~Control()
{
    running_ = false;
    for(auto &t:threads_)
    {
        t.join();
    }
}

void Control::start()
{
    running_ = true;
    threads_.push_back(std::thread(&Control::idle,this));
    threads_.push_back(std::thread(&Control::calcpath,this));
    threads_.push_back(std::thread(&Control::optvelocities,this));
}

void Control::setbogiexyth(std::vector<std::pair<Point,double>>& bogiexyth,std::vector<std::pair<Point,double>>& bogieglb, std::pair<Point,double> frnd, std::map<int,Point> map)
{
    if(gotglobal_==false)
    {
        bogieglb_ = bogieglb;
        gotglobal_ = true;
        friend_orient = frnd.second;
        map_ = map;
    }
    if(gotbogies_==false)
    {
        bogiexyth_ = bogiexyth;
        gotbogies_ = true;
        cv_.notify_all();
    }
}

void Control::calcpath()
{
    while(running_)
    {
    restart_ = false;
    std::unique_lock<std::mutex> lck(mtxpos_);
    cv_.wait(lck,[&](){return gotbogies_==true;});
    POI.clear();
    BBgraph.clear();
    double orientation = 0;
    if(base_start == true)
    {
        POI = bogieglb_; //holds pair (x,y,Range)(Bearing) of bogie from base/origin
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        Point tpose = sim_->getFriendlyPose().position;
        orientation = quaternionToYaw(sim_->getFriendlyPose().orientation);
        //friend_orient = orientation;
        for(int i=0;i<bogieglb_.size();i++)
        {
            Point p;
            p.x = bogieglb_[i].first.x - tpose.x;
            p.y = bogieglb_[i].first.y - tpose.y;
            p.z = pow((pow(p.x,2)+pow(p.y,2)),0.5);
            double t = atan2(p.y,p.x);
            if(t<0){t=t+2*M_PI;}
            POI.push_back(std::make_pair(p,t));
        }
    }    
    
    lck.unlock();
    double alpha = 0; //angle of rotation required to point toward bogie(-CW +CCW)
    double index = 0;
    double branch_time = 0;

    ///Calculates time to each bogie from starting position ////////////
    for (auto it:POI) 
    {
        
        alpha = wrapalpha(it.second-orientation);
        double turn_time = abs(alpha/MAX_ang); //quickest possible turn time at V = 50m/s
        double straight_time = abs(it.first.z/MAX_vel); //quickest possible straight at Ang = 0;
        double POI_time = turn_time+straight_time; //total time

        std::pair<double,double> sort;
        sort = std::make_pair(POI_time,index); //time to bogie,index value of bogie from ORIGINAL POI vector
        POI_sort_.push_back(sort);             //push back into vector to hold for all bogies
        index++;  
    }
    std::sort(POI_sort_.begin(), POI_sort_.end());  //sort vector by shortest time first
    index = 0;
    /////////////////////////////////////////////////////////////////

    ////////////////Calculates the distance and angle between each and all bogies/////////////////
    for(int i=0;i<POI.size();i++)
    {
        D_A da;
        std::vector<std::pair<D_A,int>> BBdist_angle; //vector of pairs that holds time to bogieA from bogieB and index of bogieB
        index = 0;
        if(i!= POI.size())                                //position of THIS vector in top vector is index of bogieA
        {
        for(int j=0;j<POI.size();j++)
        {
            if(index!= POI.size() && index!=i && i!=j)
            {
            double D_x = POI[i].first.x - POI[j].first.x;
            double D_y = POI[i].first.y - POI[j].first.y;
            double D = pow(pow(D_x,2)+pow(D_y,2),0.5); //get distance between bogieA and bogieB
            double theta = atan2(D_y,D_x);             //get angle from global x axis from A to B
            if(theta<0){theta=theta+2*M_PI;}    
            da = {D,theta};       
            BBdist_angle.push_back(std::make_pair(da,index)); //distance and angle from Bogie(i) to bogie(i+1)
            }
            index++;
            if(index==POI.size()){index=0;}
        }         
        }
        BBgraph.push_back(BBdist_angle);
    }
    //////////////////////////////////////////////////////////////////////////
    opt_path_.clear();
    opt_path_.resize(POI.size(),0);
    getoptimalpath(opt_path_);
    move();
    }
}


void Control::move()
{
    clock_t t;
    clock_t t2;
///////////////////////////////////////////////////////////////////////////////////
    Pose pose;
    XY xy;      
    gamma_ = 0;
    double orient,x,y,l,alph,V,W,current,target;
    bool loop;
    bool check;
    int num_bogie = POI.size();
    double sign;
    std::vector<std::pair<int,int>> assoc;
    int count = 0;
    double i = 0;
    int skipn = 5;
    
    //current = quaternionToYaw(sim_->getFriendlyPose().orientation);

    while(count != POI.size())
    {
        if(i==skipn)
        {
            i++;
            if(i>=POI.size())
            {
                break;
            }
        }
        assoc.clear();
        sign = 1;
        pose = sim_->getFriendlyPose();
        xy.x = pose.position.x;
        xy.y = pose.position.y;   
        if(base_start == true)
        {
            target = POI[opt_path_[i]].second;
        }
        else
        {
            double distx = bogieglb_[opt_path_[i]].first.x - xy.x;
            double disty = bogieglb_[opt_path_[i]].first.y - xy.y;
            target = atan2(disty,distx);
        }
        
        //target = atan2(diffy,diffx);
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
      
        loop = false;        
        current = quaternionToYaw(sim_->getFriendlyPose().orientation);
        if(current<0){current = current +2*M_PI;}
        startmove_ = false;
        check = false;
        if(current<M_PI)
        {
            if(target<current || target>current+M_PI)
            {sign = -1;}
        }
        if(current>M_PI)
        {
            if(target<current && target>((current+M_PI)-2*M_PI))
            {sign = -1;}
        }
        
        while(startmove_== false && check == false)
        {
            check = false;
            lin_vel_ = MIN_vel;
            ang_vel_ = MAX_ang*sign;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));                
            if(current>lower && current<upper)
            {
                check = true;
                if(loop == false)
                {
                pose = sim_->getFriendlyPose();
                xy.x = pose.position.x;
                xy.y = pose.position.y;
                current = quaternionToYaw(sim_->getFriendlyPose().orientation);
                if(current<0){current = current +2*M_PI;}
                x = (bogieglb_[opt_path_[i]].first.x-xy.x)*cos(current)+(bogieglb_[opt_path_[i]].first.y-xy.y)*sin(current);
                y = -1*(bogieglb_[opt_path_[i]].first.x-xy.x)*sin(current)+(bogieglb_[opt_path_[i]].first.y-xy.y)*cos(current);
                l = pow((pow(x,2)+pow(y,2)),0.5);
                gamma_ = 2*y/(l*l);
                getvel_ = true;
                Velcv_.notify_all();                          
                loop = true;
                }
            }
            current = quaternionToYaw(sim_->getFriendlyPose().orientation);
            if(current<0){current = current +2*M_PI;}
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }            
        double s = sim_->rangeBearingToBogiesFromFriendly().size();
        t2 = clock();
        t = clock() + 1000;
        while(s == num_bogie)
        {
            
            if(t-t2>600)
            {
            pose = sim_->getFriendlyPose();
            xy.x = pose.position.x;
            xy.y = pose.position.y;
            current = quaternionToYaw(sim_->getFriendlyPose().orientation);
            if(current<0){current = current +2*M_PI;}
            x = (bogieglb_[opt_path_[i]].first.x-xy.x)*cos(current)+(bogieglb_[opt_path_[i]].first.y-xy.y)*sin(current);
            y = -1*(bogieglb_[opt_path_[i]].first.x-xy.x)*sin(current)+(bogieglb_[opt_path_[i]].first.y-xy.y)*cos(current);
            if(Simulator::AIRSPACE_SIZE-abs(xy.x)<=300 || Simulator::AIRSPACE_SIZE-abs(xy.x)<=300)
            {
                lin_vel_ = MIN_vel;
                ang_vel_ = MAX_ang;
                double point = atan2((bogieglb_[opt_path_[i]].first.y - xy.y),(bogieglb_[opt_path_[i]].first.x - xy.x));
                if(point<0){point = point + 2*M_PI;}
                while(current != point)
                {
                    lin_vel_ = MIN_vel;
                    ang_vel_ = MAX_ang;                    
                }
            }
            l = pow((pow(x,2)+pow(y,2)),0.5);
            gamma_ = 2*y/(l*l);
            getvel_ = true;
            Velcv_.notify_all();
            t2 = clock();
            }
            t = clock(); 
            //lin_vel_ = lin_vel_;
            //ang_vel_ = ang_vel_;
            //std::this_thread::sleep_for(std::chrono::milliseconds(600));
            s = sim_->rangeBearingToBogiesFromFriendly().size();
        }
        assoc = rangebearassociate(sim_->rangeBearingToBogiesFromFriendly(),sim_->getFriendlyPose().position);
        for(auto it:assoc)
        {
            if(it.first == i-1 && opt_path_[i]!=0)
            {
                i--;
                skipn = i+1;
            }
        }        
        num_bogie--;
        lin_vel_ = MIN_vel;
        ang_vel_ = MAX_ang;
        base_start = false;
        i++;
        count++;
    } 
    gotbogies_ = false;
    gotglobal_ = false;
    lin_vel_ = MIN_vel;
    ang_vel_ = MAX_ang;
    restart_ = true;
}

void Control::idle()
{
    while(running_)
    {        
        //Feed the watchdog control timer
        command(lin_vel_, ang_vel_);
    }
}

void Control::optvelocities()
{
    while(running_)
    {
    std::unique_lock<std::mutex> lck(Velmtx_);
    Velcv_.wait(lck,[&](){return getvel_==true;});
    lck.unlock();
    double V = 1000;
    double W = gamma_*V;
    while(abs(V*W) > MAX_VA)
    {
        V = V - 10;
        W = gamma_*V;
    }
    startmove_ = true;
    getvel_ = false;
    lin_vel_ = V;
    ang_vel_ = W;
    }
}


void Control::getoptimalpath(std::vector<int>& optpath)
{
     //////////////////////////////Graph search/optimal graph path///////////////////////
    std::vector<int> graphpath_time;
    double quickest_branch_time = 1000;
    double BBquickest_time = 1000;
    double branch_time = 0;
    int starting_node_;
    double current_orient;

    for(int i=0;i<BBgraph.size();i++)
    {
        graph_path.clear();
        int check = 0;
        branch_time = 0;

        branch_time = branch_time + POI_sort_[i].first;
        starting_node_ = POI_sort_[i].second;
        current_orient = POI[starting_node_].second; //orientation angle once at bogie(i)
        if(current_orient<0){current_orient=current_orient+2*M_PI;}
        if(current_orient>2*M_PI){current_orient=current_orient-2*M_PI;}
        graph_path.push_back(starting_node_);
        
        int next_node = 0;
        
        for(int j=0;j<BBgraph.size();j++)
        {
            if(graph_path.size()!=POI.size())
            {
            BBquickest_time = 1000;
            next_node = graphsearch(starting_node_,BBquickest_time,current_orient);
            starting_node_ = next_node;
            graph_path.push_back(next_node);
            branch_time = BBquickest_time + branch_time;
            if(branch_time>quickest_branch_time)
            {
                check = 1;
                break;
            }
            }
        }
        if(branch_time<quickest_branch_time)
        {
            quickest_branch_time = branch_time;
        }
        if(check != 1)
        {
            graphpath_time = graph_path;
        }
    }
    optpath = graphpath_time;
}


int Control::graphsearch(int node, double& quickest_time,double& current_orient)
{
    
    int next_node = 0;
    bool notvisited;
    int num_nodes = 0;
    
    for(int i=0;i<BBgraph[node].size();i++)
    {
        notvisited = true;
        for(auto it:graph_path)
        {
            if(BBgraph[node][i].second == it)
            {
                notvisited = false;
                num_nodes++;
            }
        }        
        if(notvisited)
        {
            //current_orient = (BBgraph[node][i].second); //orientation once at bogie(i)
            // double x = POI[node].first.x;
            // double y = POI[node].first.y;
            // double x1 = POI[BBgraph[node][i].second].first.x;
            // double y1 = POI[BBgraph[node][i].second].first.y;
            // double ang = atan2((y1-y),(x1-x));
            // if(ang<0){ang = ang+2*M_PI;}

            double ang =  BBgraph[node][i].first.angle;

            double angle = relateOrienttoBBangle(current_orient,ang);
            double dist = BBgraph[node][i].first.dist;
            double time = abs(angle/MAX_ang) + abs(dist/MAX_vel);
            if(time<quickest_time)
            {
                quickest_time = time;
                next_node = BBgraph[node][i].second;
                current_orient = BBgraph[node][i].first.angle;
            }
        }
        if(num_nodes==3){quickest_time = 0;}        
    }
    return next_node;
}


double Control::wrapalpha(double bearing)
{
    double alpha;
    if(bearing<0){bearing = bearing+2*M_PI;}
    if(bearing>360){bearing = bearing-2*M_PI;}
    if(bearing>=0 && bearing<=M_PI)
    {
        alpha = bearing;
    }
    if(bearing>=M_PI && bearing<=2*M_PI)
    {
        alpha = bearing - 2*M_PI;
    }

    return alpha;
}

double Control::relateOrienttoBBangle(double orient, double bearing)
{
    double alpha;
    alpha = bearing - orient;
    
    if(alpha>2*M_PI){alpha = alpha-2*M_PI;}
    if(alpha<0){alpha = alpha+2*M_PI;}
    alpha = wrapalpha(alpha);

    return alpha;
}


void Control::command(double lin, double ang)
{
    std::unique_lock<std::mutex> lock(Cmdmtx_);
    sim_->controlFriendly(lin,ang);
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

bool Control::getrestart()
{
    return restart_;
}


 std::vector<std::pair<int,int>> Control::rangebearassociate(std::vector<RangeBearingStamped> rbs,Point friendxy)
 {
    std::vector<std::pair<int,int>> assoc;
    for(int i=0;i<rbs.size();i++)
    {
        for(int j=0;j<map_.size();j++) //can relate a given rbs given a known set of global bogie coordinate and known friendly position
        {                //since the distance from a bogie to the friendly must be the same
            double x = map_[opt_path_[j]].x - friendxy.x;
            double y = map_[opt_path_[j]].y - friendxy.y;
            double r = pow((pow(x,2)+pow(y,2)),0.5);
            if(r >= rbs[i].range - 100 && r <= rbs[i].range + 100)
            {
                int key;
                for(auto it:map_)
                {
                    if(it.second == map_[opt_path_[j]])
                    {
                        key = it.first;
                    }
                }
                assoc.push_back(std::make_pair(key,i));
            }
        }
    }
    return assoc;
 }