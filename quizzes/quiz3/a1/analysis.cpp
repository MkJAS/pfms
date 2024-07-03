#include "analysis.h"

// using std::vector;
// using std::pair;

//! @todo
//! TASK 1 - We need to store cars in the Analysis class, how to do this?
Analysis::Analysis(std::vector<Car*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{

}

Analysis::Analysis(std::vector<Car*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    std::vector<std::pair<double,int>> sortedpair;
    for(int i=0;i<order.size();i++)
    {
        double odom = cars_[i]->getOdometry();
        sortedpair.push_back(std::make_pair(odom,i));
    }
    std::sort(sortedpair.begin(),sortedpair.end());
    for(int i=0;i<order.size();i++)
    {
        order[sortedpair[i].second] = i;
    }

    return order;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){

    std::vector<unsigned int> order(cars_.size(),0);
    std::vector<double> old_odom;
    std::vector<bool> car_done(cars_.size(),false);
    int index = 0;
    int place = 0;
    
    
    for(int i=0;i<cars_.size();i++)
    {
        old_odom.push_back(cars_[i]->getOdometry());
    }

    while(std::find(car_done.begin(),car_done.end(),false)!=car_done.end())
    {
        index = 0;
        for(int i = 0;i<cars_.size();i++)
        {
            if(!car_done[i])
            {
                cars_[i]->accelerate();
            
                if(cars_[i]->getOdometry()-old_odom[index]>distance)
                {
                    order.at(index) = place++;
                    car_done.at(index) = true;
                }
            }
            index++;
        }
    
    }
    return order;
}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars()
{
    //std::vector<Car*> &cars = cars_;
    int count = 0;
    while(count<cars_.size())
    {
        for (int i=0;i<cars_.size();i++) 
        {
            if(cars_[i]->getCurrentSpeed() > 0 ) 
            {
                cars_[i]->decelerate();
            }
            if(cars_[count]->getCurrentSpeed()<=0){count++;}
            
        }
        
    }


}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace(){

    std::vector<unsigned int> order(cars_.size(),0);//Creating a vector, same size as cars with all zeros
    int index = 0;
    int place = 0;
    std::vector<bool> car_done(cars_.size(),false);
    std::vector<bool> car_dec(cars_.size(),false);
    std::vector<Car*> &cars = cars_;


    for(auto car:cars_)
    {
        car->accelerate();
    }
    while(std::find(car_done.begin(),car_done.end(),false)!=car_done.end())
    {

        index = 0;
        for(int i = 0;i<cars_.size();i++)
        {
            if(!car_dec[i])
            {
                cars_[i]->accelerate();
            }
            if(cars_[i]->getCurrentSpeed()>=cars_[i]->getTopSpeed() || car_dec[i])
            {
                cars_[i]->decelerate();
                car_dec[i] = true;
            }

            if(cars_[i]->getCurrentSpeed()<=0 && car_done[i]!=true)
            {
                order.at(index) = place++;
                car_done.at(index) = true;
            }
            index++;
        }



        // for(int i=0;i<cars_.size();i++)
        // {
        //     if(cars_[i]->getCurrentSpeed()>=cars_[i]->getTopSpeed()||cars_[i]->getCurrentSpeed()>0)
        //     {
        //         cars_[i]->decelerate();
        //     }
        //     if(cars_[i]->getCurrentSpeed()<=0)
        //     {
        //         order[i] = index;
        //         index++;
        //     }
        // }
    }
    
    return order;
}

// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}
