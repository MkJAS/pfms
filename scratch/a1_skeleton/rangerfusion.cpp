#include "rangerfusion.h"
#include "rangerinterface.h"
#include "analysis.h"
#include "rangerfusioninterface.h"
#include <math.h>
#include <algorithm>
#include <deque>
#include <iostream>

double degtorad = M_PI/180;

RangerFusion::RangerFusion(std::vector<RangerInterface*> rangers)
{
    rangers_ = rangers;
    data_.clear();
}


void RangerFusion::setCells(std::vector<Cell*> cells)
{
    cells_= cells;
}

/**
     * @brief Does two operations (1) Calls each ranger to generate 
     * data and uses this data to determine colissions with provided container 
     * of cells (2) Generates a 'fusion' of the data based on collision conditions 
     * as descibed in Assignment 2 specification
*/
void RangerFusion::grabAndFuseData()
{
    std::vector<double> rangerdata;                 //create vector to store RawData
    rangerdata.clear();    
    std::vector<std::vector<double>> allrangerdata;     //create a vector of vectors
    allrangerdata.clear();
                                                        //generate and pass RawData into said vector
    for (unsigned int i =0; i<rangers_.size();i++)
    {
        rangerdata = rangers_[i]->generateData();
        allrangerdata.push_back(rangerdata);            //store vector of RawData in vector of vectors
        rangerdata.clear();
    }
    data_ = allrangerdata;
    
    double Xcentre=0,Ycentre=0;
    double side = 0;
    
    std::vector<std::vector<geometry_msgs::Point>> data_coordinates;
    std::vector<geometry_msgs::Point> points;

    for(unsigned int j=0;j<rangers_.size();j++) //for all sensors-do;
    {
        if (rangers_[j]->getSensingMethod() == ranger::SensingMethod::POINT) //if sensor is a laser do;
        {
            points.clear();
            ranger::SensorPose sensorPose = rangers_[j]->getSensorPose(); //get sensor coordinates and angle offset
            double theta = sensorPose.theta/degtorad;
            //we take laser readings as the 0th element being the first laser on the right, going counter clockwise
            geometry_msgs::Point laserStart{sensorPose.x,sensorPose.y};
            points.push_back(laserStart); //store sensor centre as first vector element 0
            double laserAngle = rangers_[j]->getAngularResolution(); //get laser increments
            geometry_msgs::Point laserEnd = {0,0};
            for (unsigned int k=0;k<1+(180/laserAngle);k++) //iterate for number of readings, based on resolution.
                                                            //eg. res of 10 = 180/10+1 = 19 readings
            {
                //to define end POS of laser reading we take the value and divide into its horizontal and vertical components
                //using the cos and sin of the angle of the reading which can be found from the resolution*the reading number
                laserEnd.x = laserStart.x + allrangerdata[j][k]*cos(2*M_PI-degtorad*((laserAngle*k)+theta));
                laserEnd.y = laserStart.y + allrangerdata[j][k]*sin(M_PI-degtorad*((laserAngle*k)+theta));
                points.push_back(laserEnd);
                
            }
                
        }
        else if (rangers_[j]->getSensingMethod() == ranger::SensingMethod::CONE) //if sensor is a cone then do;
        {
            points.clear();
            double sonarangle = (rangers_[j]->getFieldOfView())/2;  //sonar FOV/2. angle between each edge and centre mark
            ranger::SensorPose sensorPose = rangers_[j]->getSensorPose(); 
            geometry_msgs::Point sonarStart{sensorPose.x,sensorPose.y}; //location of sonar start
            points.push_back(sonarStart);
            double theta = sensorPose.theta/degtorad;                            //angle of rotation of sensor
            double radius = allrangerdata[j][0];                        //radius of sector/sonar
            
            geometry_msgs::Point sonarcorner1; //sonar cone has 3 points. 1 at its begining at 2 and the ends of cone
            geometry_msgs::Point sonarcorner2;
            //we assume that at 0 angle offset, sonar faces straight, normal to the x plane, hence the 90+
            sonarcorner1.x = sonarStart.x + radius*cos(2*M_PI - (degtorad)*(90+sonarangle+theta));
            sonarcorner1.y = sonarStart.y + radius*sin(M_PI - (degtorad)*(90+sonarangle+theta));
            //add pose offset  convert angled lines into its horizontal and vertical components using sin0 cos0. 
            //minusing PI to ensure signs of value pertains to correct section on cartesian plane
            
            sonarcorner2.x = sonarStart.x + radius*cos(2*M_PI - (degtorad)*(90-sonarangle+theta));
            sonarcorner2.y = sonarStart.y + radius*sin(M_PI - (degtorad)*(90-sonarangle+theta));
            points.push_back(sonarcorner1);
            points.push_back(sonarcorner2);
        }   
            data_coordinates.push_back(points); 
    }
    
    for (unsigned int i =0; i<cells_.size();i++) //iterate check through each cell
    {
        cells_[i]->getCentre(Xcentre,Ycentre);
        side = cells_[i]->getSide();
        
        //we label corners alphabetically as starting at the top left corner and going counter clockwise
        geometry_msgs::Point a{Xcentre-(side/2),Ycentre+(side/2)};
        geometry_msgs::Point b{Xcentre-(side/2),Ycentre-(side/2)};
        geometry_msgs::Point c{Xcentre+(side/2),Ycentre-(side/2)};
        geometry_msgs::Point d{Xcentre+(side/2),Ycentre+(side/2)};
        for (int j=0;j<rangers_.size();j++)         //for all sensors do;
        {
            if (rangers_[j]->getSensingMethod() == ranger::SensingMethod::POINT) //if sensor is laser do;
            {
                for(int l =0;l<data_coordinates[j].size()-1;l++)
                {
                    if(cells_[i]->getState() != cell::State::OCCUPIED) //if cell is already OCCUPIED we do not need to check its state again so skip
                    {
                    if (data_coordinates[j][l+1].x<c.x && data_coordinates[j][l+1].x>a.x && data_coordinates[j][l+1].y<a.y && data_coordinates[j][l+1].y>c.y) 
                    {//check if laser end is within the bounds of the 4 cell corners
                        cells_[i]->setState(cell::State::OCCUPIED);
                    }
                    if(cells_[i]->getState() != cell::State::OCCUPIED)
                    {
                        if (LaserIntersect(data_coordinates[j][0],data_coordinates[j][l+1],a,b,c,d))
                        {//check if line between laser end and start intersect any cell edges
                            cells_[i]->setState(cell::State::FREE);
                        }
                    }
                    }
                }
                
            }
            else if (rangers_[j]->getSensingMethod() == ranger::SensingMethod::CONE)
            {
                double sonarangle = (rangers_[j]->getFieldOfView())/2; //angle between sonar edge and sonar centre
                ranger::SensorPose sensorPose = rangers_[j]->getSensorPose();
                double theta = sensorPose.theta/degtorad;                            //angle of rotation of sensor
                double radius = allrangerdata[j][0];                        //sonar reading
                if(cells_[i]->getState() != cell::State::OCCUPIED) //if cell is already OCCUPIED we do not need to check its state again so skip
                {
                if (SonarOccupied(data_coordinates[j][0],data_coordinates[j][1],theta,sonarangle,radius,a,b,c,d))
                {//check if points along curve exist within bounds of cells 4 corners
                    cells_[i]->setState(cell::State::OCCUPIED);
                }
                if (cells_[i]->getState() != cell::State::OCCUPIED)
                {//check if any of cell corners are within sonar bounds or if any cell edges intersect with sonar edges
                    if(SonarFree(theta,radius,sonarangle,data_coordinates[j][0],data_coordinates[j][1],a,b))
                    {
                      cells_[i]->setState(cell::State::FREE);
                    }
                    if(SonarFree(theta,radius,sonarangle,data_coordinates[j][0],data_coordinates[j][1],b,c))
                    {
                      cells_[i]->setState(cell::State::FREE);
                    }
                    if(SonarFree(theta,radius,sonarangle,data_coordinates[j][0],data_coordinates[j][1],c,d))
                    {
                      cells_[i]->setState(cell::State::FREE);
                    }
                    if(SonarFree(theta,radius,sonarangle,data_coordinates[j][0],data_coordinates[j][1],d,a))
                    {
                      cells_[i]->setState(cell::State::FREE);
                    }
                }
                }
            }
        }
    }
}

std::vector<std::vector<double>> RangerFusion::getRawRangeData()
{
    return data_;
}


/**
     * @brief Returns the total scanning area possible with CONE based scanners supplied
     * A union of all areas https://en.wikipedia.org/wiki/Union_(set_theory)
     *
     * @return double Total area coverage
     *
     * @sa grabAndFuseData
     */
double RangerFusion::getScanningArea()
{
    double isolatedArea = 0;
    bool hasintersect = false;
    bool addcorners = false;
    int check = 0;
    double radius = 0;
    double sonarangle = 0;
    double max = 0;
    std::vector<unsigned int> indexcheck; //vector to keep track of sonar intersections
    std::vector<geometry_msgs::Point> sonarcorners;
    std::vector<std::vector<geometry_msgs::Point>> allsonarcorners;
    std::vector<geometry_msgs::Point> all_POI;      //vector to store ALL points of intersection
    std::deque<geometry_msgs::Point>  POI;          //deque storing all points NOT WITHIN a sonar area
//MUST RECALCULATE THE POINTS OF THE SONAR BECAUSE THEY ARE NOW MODELLED AS ISOCELES TRIANGLE WHERE THE 
//HEIGH IS EQUAL TO 10 IN LENGTH, AND NOT CONES. AS SUCH THE CORNER POINT COORDINATE WILL BE SLIGHTLY DIFFERENT
    for(unsigned int i=0;i<rangers_.size();i++) //loop to store all the corners of every sonar created
    {
        if (rangers_[i]->getSensingMethod() == ranger::SensingMethod::CONE) //for all sonar sensors do;
        {
            max = rangers_[i]->getMaxRange();  //in this case max = 10
            sonarangle = (rangers_[i]->getFieldOfView())/2;
            radius = max/cos(sonarangle*degtorad); //radius, i.e sonar edges, i.e. triangle hypontenueses
            ranger::SensorPose sensorPose = rangers_[i]->getSensorPose();
            double theta = sensorPose.theta/degtorad;                            //angle of rotation of sensor
            
            geometry_msgs::Point sonarStart{sensorPose.x,sensorPose.y}; //location of sonar start
            sonarcorners.push_back(sonarStart); //store sonar centre as first element 0
            geometry_msgs::Point sonarcorner1; 
            geometry_msgs::Point sonarcorner2;
            //we assume that at 0 angle offset, sonar faces straight, normal to the x plane, hence the 90+
            sonarcorner2.x = sonarStart.x + radius*cos(2*M_PI - (degtorad)*(90+sonarangle+theta));
            sonarcorner2.y = sonarStart.y + radius*sin(M_PI - (degtorad)*(90+sonarangle+theta));
            //add pose offset  convert angled lines into its horizontal and vertical components using sin0 cos0. 
            //minusing PI to ensure signs of value pertains to correct section on cartesian plane
            sonarcorner1.x = sonarStart.x + radius*cos(2*M_PI - (degtorad)*(90-sonarangle+theta));
            sonarcorner1.y = sonarStart.y + radius*sin(M_PI - (degtorad)*(90-sonarangle+theta));
            sonarcorners.push_back(sonarcorner1);
            sonarcorners.push_back(sonarcorner2);
            
            allsonarcorners.push_back(sonarcorners); //store all corner points in vec of vec
            sonarcorners.clear();
        }
    }
/////////////////////////////////////////////////////////////////////////////////////////////////
    //create lines from these points are check if they intersect with other lines
    for(unsigned int i=0;i<allsonarcorners.size();i++)
    {
        //get 3 points of sonar i
        geometry_msgs::Point a = allsonarcorners[i][0];
        geometry_msgs::Point b = allsonarcorners[i][1];
        geometry_msgs::Point c = allsonarcorners[i][2];
        for(unsigned int j=i;j<allsonarcorners.size();j++) //check points of sonar i against all other sonar
        {
            if(j != allsonarcorners.size()-1) //statement to avoid seg fault in the case where j+1 points to a vec element that doesnt exist
            {   //3 points of sonar j+1
                geometry_msgs::Point a1 = allsonarcorners[j+1][0];
                geometry_msgs::Point b1 = allsonarcorners[j+1][1];
                geometry_msgs::Point c1 = allsonarcorners[j+1][2];
                hasintersect = Checkallsides(a,b,c,a1,b1,c1,all_POI);//function checks all sides of sonari with sonarj+1
            }                                                        //and adds any intersection to all_POI vector
            if(check == 0 && hasintersect) //if just ONE intersect has occured, add all 6 sonar corners to all_POI
            {                              //if NO intersect has occured, DO NOT add corners of sonar i to all_POI
                addcorners = true; 
                check = 1;
            }
        }
        if(addcorners) //add the corners of those sonars that have intersected
        {
            if(i==0) //conditional to add sonar i ONLY once at the very beginning otherwise it will be skipped
            {
                all_POI.push_back(a);
                all_POI.push_back(b);
                all_POI.push_back(c);
            }
            if(i != allsonarcorners.size()-1)
            {
                all_POI.push_back(allsonarcorners[i+1][0]);
                all_POI.push_back(allsonarcorners[i+1][1]);
                all_POI.push_back(allsonarcorners[i+1][2]);
            }
        }
        if(!addcorners) //if no intersection for sonar i were found against any of the other sonar then sonar i must be isolated
        {               //as such its area must be calculated on its own and added to the rest at the end of calculation
            isolatedArea = isolatedArea + max*(max*tan(sonarangle*(M_PI/180)));
        }
        addcorners = false; //reset values
        check = 0;
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////
    for(unsigned int i=0;i<all_POI.size();i++)  //find which point are INSIDE a sonar and remove them from list
    {                                           //since we only need the points on the outside of the polygon
        
        geometry_msgs::Point chkpoint = all_POI[i]; //check point i againt all sonars
        for(unsigned int l=0;l<allsonarcorners.size();l++)
        {
            geometry_msgs::Point m = allsonarcorners[l][0];
            geometry_msgs::Point n = allsonarcorners[l][1];
            geometry_msgs::Point o = allsonarcorners[l][2];
            if(isInsideSonar(m,n,o,chkpoint))   //if point i exists within JUST ONE sonar DO NOT add to POI
            {
                check = 1;
                
            }
        }
        if(check == 1){indexcheck.push_back(i);}
        check = 0;
    }
    int k = 0;
    if(indexcheck.size()>0)
    {
    for(int i=0;i<all_POI.size();i++)
    {
        if(i!=indexcheck[k])
        {
            POI.push_back(all_POI[i]); //push back points that WERE NOT saved in index in previous checks
        }
        if(i==indexcheck[k])
        {
            k++;
        }
    }
    }

//Sort the points so that the order of POI is counter clockwise from the positive x axis as the calculation for area
//requires the order of points to be in a CCW rotation
//NOTE: For the case of a point being at the origin, i.e (0,0), the point fails to be ordered correctly since it is being
//compared to the origin, which in this case is itself.
//I HAVE BEEN UNABLE TO COME UP WITH A SOLUTION TO THIS CASE, HOWEVER IT DOES SORT EVERY OTHER POINT CORRECTLY
    double term1 = 0;
    double term2 = 0;
    geometry_msgs::Point centroid;
    unsigned int s = POI.size();
    
    std::vector<geometry_msgs::Point> POI_order; //vector to store POIs as polar coordinates

////Calculate the centroid of the polygon
    double cent_x = 0;
    double cent_y = 0;
    for(int z=0;z<POI.size();z++)
    {
        cent_x += POI[z].x; //add all x and y values
        cent_y += POI[z].y;
        centroid.x = cent_x/POI.size(); //find average of x and y
        centroid.y = cent_y/POI.size();
    }
/////Calculate the angels and distances of the points from the centroid   
    for(int z=0;z<POI.size();z++)
    {
        double dist = pow((pow((POI[z].x-centroid.x),2)+pow((POI[z].y-centroid.y),2)),0.5);        
        double angle = fmod((atan2((POI[z].y-centroid.y),(POI[z].x-centroid.x))*180/M_PI+360),360); //minus centroid to move polygon so that centroid is at 0,0
        POI_order.push_back({dist,angle});
    }
    
//Bubble sort esque
    if(s != 0)
    { 
    for (int i = 0; i < s-1; i++)
    {
        for (int j = 0; j < s-i-1; j++)
        {
            if(POI_order[j+1].y==POI_order[j].y)
            {
                if(POI_order[j+1].x<POI_order[j].x)
                {std::swap(POI[j],POI[j+1]);}
            }
            else if(POI_order[j+1].y<POI_order[j].y)
            {
                std::swap(POI[j],POI[j+1]);
                std::swap(POI_order[j],POI_order[j+1]);
            }
        } 
    }
    
//begin calculation of area based on link provided by Alen     
    for(unsigned int i=0;i<s;i++)
    {
        if(i == POI.size()-1) //when at last element - do last*first
        {
            term1 = term1 + POI[i].x*POI[0].y;
            term2 = term2 + POI[i].y*POI[0].x;
        }
        term1 = term1 + POI[i].x*POI[i+1].y;
        term2 = term2 + POI[i].y*POI[i+1].x;

    }
    }
    double area = (term1-term2)/2 + isolatedArea; //add isolated areas on top
    area_ = area;
    return area;
}


