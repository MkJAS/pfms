#ifndef MOCKBOGIEPOS_H
#define MOCKBOGIEPOS_H

#include "../bogiepos.h"
#include "../dep/include/types.h"

class Mockbogiepos: public Bogiepos
{
public:    
    Mockbogiepos();
    void getsimrbf(std::vector<RangeBearingStamped> rbs, Point friendxy, double friendorient);
    std::vector<std::pair<Point,double>> getlocal();
    std::vector<std::pair<Point,double>> getglobal();
    void setmap();
    void setglb_bogie_pos_(std::vector<std::pair<Point,double>>);
    std::map<int,Point> map;




private:
    std::vector<std::pair<Point,double>> mock_frnd_bogies_pos_;
    std::vector<std::pair<Point,double>> mock_glb_bogies_pos_;


};







#endif // MockBogiepos_H