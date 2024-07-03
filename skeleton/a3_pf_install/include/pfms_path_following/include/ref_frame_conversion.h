/**
 * @file    ref_frame_conversion.h
 * @ingroup Helper
 * @class   RefFrameConversion
 * @brief   Makes conversions between global, local and pixel reference frames
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef REF_FRAME_CONVERSION_H
#define REF_FRAME_CONVERSION_H

//_____________________________________________________________________________ Includes

#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include <iostream>

//_____________________________________________________________________________ Class Definition
class RefFrameConversion{

//_____________________________________________________________________________ Class Public Members
public:

    /**
     * @brief Construct a new Ref Frame Conversion object
     * 
     */
    RefFrameConversion();

    /**
     * @brief Construct a new Ref Frame Conversion object
     * 
     * @param nh nodehandle
     */
    RefFrameConversion(ros::NodeHandle nh);
    
    /**
     * @brief Destroy the Ref Frame Conversion object
     * 
     */
    ~RefFrameConversion();

    /**
     * @brief Converts global point to local point
     * 
     * @param global point
     * @param vehicle point
     * @return geometry_msgs::Point local point
     */
    geometry_msgs::Point globalToLocal(const geometry_msgs::Point global, const geometry_msgs::Point vehicle);

    /**
     * @brief Converts local point to pixel position
     * 
     * @param local point
     * @return std::pair<double, double> {pixel x, pixel y}
     */
    std::pair<double, double> localToPixel(const geometry_msgs::Point local);
    
    /**
     * @brief Converts pixel position to local point
     * 
     * @param pixel_x 
     * @param pixel_y 
     * @return geometry_msgs::Point local point
     */
    geometry_msgs::Point PixelToLocal(const double pixel_x, const double pixel_y);

    /**
     * @brief Converts local point to global point
     * 
     * @param local point
     * @param vehicle point
     * @return geometry_msgs::Point global point
     */
    geometry_msgs::Point LocalToGlobal(const geometry_msgs::Point local, const geometry_msgs::Point vehicle);

    /**
     * @brief Converts global point to pixel position 
     * 
     * @param global point
     * @param vehicle point
     * @return std::pair<double, double> pixel x, pixel y 
     */
    std::pair<double, double> globalToPixel(geometry_msgs::Point global, geometry_msgs::Point vehicle);

    /**
     * @brief Converts pixel position to global point
     * 
     * @param pixel_x 
     * @param pixel_y 
     * @param vehicle point
     * @return geometry_msgs::Point global point 
     */
    geometry_msgs::Point pixelToGlobal(double pixel_x, double pixel_y, geometry_msgs::Point vehicle);

    /**
     * @brief Gets map size from paramater server
     * 
     * @return double 
     */
    double getmapSize();

    /**
     * @brief Gets resolution from parameter server
     * 
     * @return double 
     */
    double getresolution();
    
//_____________________________________________________________________________ Class Private Members
private:
    ros::NodeHandle nh_;    //!< nodehandle

    bool debug_;            //!< debug switch
    double resolution_;     //! size of OgMap in pixels
    double mapSize_;        //! size of OgMap in pixels
};
#endif //REF_FRAME_CONVERSION_H