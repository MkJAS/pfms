#include "analysis.h"
#include "tf.h"
#include "tf2.h"

using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{

}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
vector<double> Analysis::timeToImpact(Pose origin){

    //The consts you will need are
    //Display::G_MAX 6
    //Display::V_TERM 50 
    //Display::V_MAX 1000
    vector<double> times;
    RangeBearingStamped rbstamped;
    double theta;
    double A_MAX = 6*9.81/Display::V_TERM;
    for(auto it:goals_)
    {
        rbstamped = tf2::global2local(it,origin);
        theta = rbstamped.bearing;
        if(rbstamped.bearing>M_PI)
        {
            theta = 2*M_PI-rbstamped.bearing;
        }
        
        double time = rbstamped.range/Display::V_MAX + theta/A_MAX;
        times.push_back(time);
    }

    
    return times;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
AdjacencyList Analysis::exportGraph(){

   // vector<vector<pair<double,unsigned int>>

    //std::make_pair(odom.at(i), i));
    AdjacencyList graph;
    double index = 0;

    for(int i=0;i<goals_.size();i++)
    {
        vector<pair<double,unsigned int>> edge;
        index = 0;
        if(i!=goals_.size())
        {
        for(int j=0;j<goals_.size();j++)
        {
            if(index!=goals_.size() && index!=i)
            {
            
            double x = goals_[i].x - goals_[index].x;
            double y = goals_[i].y - goals_[index].y;
            double dist = pow((pow(x,2)+pow(y,2)),0.5);
            edge.push_back(std::make_pair(dist,index));
                     
            }
            index++;
            if(index==goals_.size()){index = 0;}
        } 
        }
        graph.push_back(edge);
    }
    

    return graph;
}
