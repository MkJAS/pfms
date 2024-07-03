#include "mockbogiepos.h"

Mockbogiepos::Mockbogiepos()
{

}

void Mockbogiepos::getsimrbf(std::vector<RangeBearingStamped> rbs, Point friendxy, double friendorient)
{
    Point temp_pos_friend = friendxy;
    double temp_theta = friendorient;
    mock_frnd_bogies_pos_.clear();
    mock_glb_bogies_pos_.clear();
    
    std::vector<std::pair <double,double>> range_bear; 
    for(auto it:rbs)
        {
            std::pair<double,double> temp;
            temp = std::make_pair(it.range,(it.bearing));
            range_bear.push_back(temp);
        }

    for(int i=0;i<range_bear.size();i++)
        {
            Point temp = calcfrndxy(range_bear[i]);
            std::pair<Point,double> tempp;
            tempp = std::make_pair(temp,range_bear[i].second);
            mock_frnd_bogies_pos_.push_back(tempp);
        }

    for(int i=0;i<range_bear.size();i++) //boggie pos from global frame of ref (i hope)
        {
            std::pair<Point,double> T = calcglobalxy(mock_frnd_bogies_pos_[i].first,temp_pos_friend,temp_theta);
            mock_glb_bogies_pos_.push_back(T);
        }
}

std::vector<std::pair<Point,double>> Mockbogiepos::getlocal()
{
    return mock_frnd_bogies_pos_;
}

std::vector<std::pair<Point,double>> Mockbogiepos::getglobal()
{
    return mock_glb_bogies_pos_;
}

void Mockbogiepos::setmap()
{
    m_.clear();
    for(int i=0;i<glb_bogies_pos_.size();i++)
    {
        m_.insert({i,glb_bogies_pos_[i].first});
    }
    map = m_;
}

void Mockbogiepos::setglb_bogie_pos_(std::vector<std::pair<Point,double>> b)
{
    glb_bogies_pos_ = b;
    setmap();
}