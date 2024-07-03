#ifndef MOCKRANGERFUSION_H
#define MOCKRANGERFUSION_H

#include "rangerfusion.h"
#include "rangerinterface.h"
#include <vector>
#include "cell.h"

#include <opencv2/opencv.hpp>

using ranger::SensorPose;

class MockRangerFusion : public RangerFusion
{
public:
    MockRangerFusion(std::vector<RangerInterface*> rangers);

    void grabAndFuseData();
    void setCells(std::vector<Cell*> cells);

private:
    std::vector<Cell*> cells_;
    std::vector<RangerInterface*> rangers_;

    void drawCell(Cell cell, cv::Scalar colour);
    cv::Point convertToPoint(ranger::SensorPose ord);

    cv::Mat fusion_;
    unsigned int pixel_map_size_;               /*!< Pixel size of the airspace */

};

#endif // MOCKRANGERFUSION_H

