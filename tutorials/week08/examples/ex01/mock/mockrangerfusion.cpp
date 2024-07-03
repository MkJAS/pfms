#include "mockrangerfusion.h"
#include <iostream>


//Colours of actors within the sensors
static const cv::Scalar CLR_SONAR     = cv::Scalar(160, 255, 175);
static const cv::Scalar CLR_LASER  = cv::Scalar(0, 0, 255);
static const cv::Scalar CLR_SENSOR      = cv::Scalar(255, 0, 0);
static const cv::Scalar CLR_MAP       = cv::Scalar(255, 160, 160);
static const cv::Scalar CLR_CELL       = cv::Scalar(255, 240, 190);

static const double MAP_RES             = 0.025; //! Map size
static const double FUSION_SIZE           = 25.0; //Will produce a square map of size (metres)
// The 'centre' reference for sensors is offset from zero angle (positive x axis)
static const double ANG_OFFSET = 90.0;  // The X,Y axis is offset by 90 deg from standard X,Y
static const std::string WINDOW_NAME = "mockfusion visualise"; // Name of gui window


MockRangerFusion::MockRangerFusion(std::vector<RangerInterface*> rangers) :
    RangerFusion(rangers),rangers_(rangers)
{

    pixel_map_size_ = static_cast<unsigned int>(FUSION_SIZE / MAP_RES);
    //Step 1: Draw the sensors
    fusion_ = cv::Mat(static_cast<int>(pixel_map_size_), static_cast<int>(pixel_map_size_), CV_8UC3, CLR_MAP);

}


void MockRangerFusion::grabAndFuseData() {
    RangerFusion::grabAndFuseData();

    std::vector<std::vector<double>> data_ = RangerFusion::getRawRangeData();

    fusion_.setTo(CLR_MAP);

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WINDOW_NAME,800,800);
    cv::waitKey(1); //WaitKey is unavoidable. It will insert a 1ms delay.

    // for each cell, check each sensor.
    for(auto cell : cells_) {

        //check each sensor
        // Iterator for Ranger and Data vectors
        std::vector<RangerInterface *>::iterator it_r = rangers_.begin();
        std::vector<std::vector<double>>::iterator it_d = data_.begin();

        // Iterate through both containers together
        for(;it_r < rangers_.end() && it_d < data_.end(); ++it_r, ++it_d) {


            //Let's get the sensor pose
            ranger::SensorPose sensorPose =  (*it_r)->getSensorPose();

            // For each sonar
            if((*it_r)->getSensingMethod() == ranger::CONE) {
                // Angle for the centre of the sensor
                // Relative to 0 (positive x-axis)
                double theta = ANG_OFFSET + (sensorPose.theta*180/M_PI);
                double fov = (*it_r)->getFieldOfView();

                // Because data and rangers have equal number of elements, we can access both with our counter i
                // The radar data is a single value, so use front()
                double range = it_d->front();

                // Needed for our gui, drawing an elipse
                int axis = std::round(range / MAP_RES);

                double startAngle = (theta-fov/2);
                double endAngle = (theta+fov/2);

                // Elipse needs angle in degrees
                cv::ellipse(fusion_,convertToPoint(sensorPose),cv::Size(axis,axis),
                            0,-startAngle,-endAngle,CLR_SONAR);

                // We need to convert angle to radians
                startAngle*= M_PI/180;
                endAngle*= M_PI/180;

                //cv::circle(fusion_,convertToPoint(sensorPose),axis, CLR_SONAR);
                ranger::SensorPose sonarPoint;
                sonarPoint.x = sensorPose.x+ range*cos(startAngle);
                sonarPoint.y = sensorPose.y+ range*sin(startAngle);
                cv::line(fusion_,convertToPoint(sensorPose),convertToPoint(sonarPoint),CLR_SONAR);
                sonarPoint.x = sensorPose.x+ range*cos(endAngle);
                sonarPoint.y = sensorPose.y+ range*sin(endAngle);
                cv::line(fusion_,convertToPoint(sensorPose),convertToPoint(sonarPoint),CLR_SONAR);


            // If the sensor is a laser
            }else if((*it_r)->getSensingMethod() == ranger::POINT) {

                // The laser has multiple beams
                // For this cell, we must check each beam, iterate through them

                // Counter for which beam we are on
                // Used to calculate the beam angle
                int j = 0;
                std::vector<double>::iterator it = it_d->begin();

                // For each beam
                for(; it < it_d->end(); ++it, ++j) {


                    // The angle of the current beam relative to zero (+ve x-axis)
                    double theta = ANG_OFFSET + (sensorPose.theta*180/M_PI)  - ((*it_r)->getFieldOfView() * 0.5)
                            + (int((*it_r)->getAngularResolution()) * j);

                    theta*=M_PI/180;

                    ranger::SensorPose laserDot;
                    laserDot.x = sensorPose.x + *it * cos(theta);
                    laserDot.y = sensorPose.y + *it * sin(theta);

                    cv::line( fusion_,
                            convertToPoint(sensorPose),
                            convertToPoint(laserDot),
                            CLR_LASER
                        );


                }
            }
            else{
                std::cout << "--------------------------------------------------------" << std::endl;
                std::cout << "UNKNOWN SENOSR TYPE, type not initialised in default constructor" << std::endl;
                std::cout << "--------------------------------------------------------" << std::endl;
                std::ostringstream oss;
                oss << "UNKNOWN SENOSR TYPE";
                std::string info = oss.str();
                std::cout << info << std::endl;
                cv::putText(fusion_, info, {2,15}, cv::FONT_HERSHEY_SIMPLEX, 0.5, CLR_SENSOR);
            }

            drawCell(*cell,CLR_CELL);
        }
    }


    cv::imshow(WINDOW_NAME, fusion_);
    cv::waitKey(0);


}

void MockRangerFusion::setCells(std::vector<Cell*> cells) {
    RangerFusion::setCells(cells);
    cells_ = cells;

//    for(auto cell:cells){
//        double x,y;
//        cell->getCentre(x,y);
//        double w = cell->getSide();
//        //represents the top left corner of rectangle
//        cv::Point tl = convertToPoint({x-(w*0.5), y+(w*0.5),0 } );
//        cv::Point br = convertToPoint({x+(w*0.5), y-(w*0.5),0 } );
//        std::cout << "Cell cent [x,y,w]" << x << "," << y << "," << w << std::endl;
//                     " [x,y]" << tl.x << "," << tl.y <<
//                     " [x,y]" << br.x << "," << br.y << std::endl;
//    }

}


cv::Point MockRangerFusion::convertToPoint(ranger::SensorPose ord) {
  int convertedX;
  int convertedY;

  if(ord.x >= 0){
    convertedX = static_cast<int>((pixel_map_size_ / 2) + std::abs(std::round(ord.x / MAP_RES)));
  } else {
    convertedX = static_cast<int>((pixel_map_size_ / 2) - std::abs(std::round(ord.x / MAP_RES)));
  }

  if (ord.y >= 0){
    convertedY = static_cast<int>((pixel_map_size_ / 2) - std::abs(std::round(ord.y / MAP_RES)));
  } else {
    convertedY = static_cast<int>((pixel_map_size_ / 2) + std::abs(std::round(ord.y / MAP_RES)));
  }

  return cv::Point(convertedX, convertedY);
}

void MockRangerFusion::drawCell(Cell cell, cv::Scalar colour){

    double x,y;
    cell.getCentre(x,y);
    double w = cell.getSide();
    //represents the top left corner of rectangle
    cv::Point tl = convertToPoint({x-(w*0.5), y+(w*0.5),0 } );
    cv::Point br = convertToPoint({x+(w*0.5), y-(w*0.5),0 } );

    cv::rectangle(fusion_,tl,br,colour);
    cv::imshow(WINDOW_NAME, fusion_);
}

