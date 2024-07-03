#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

/*!
 *  \brief     Ranger Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived sensor classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning   students MUST NOT change this class (the header file)
 */


namespace ranger {

    typedef enum {
      CONE, /*!< Cone based sesnors, closest point reported within the entire cone */
      POINT /*!< Point based sesnors, closest point reported at a point */
    } SensingMethod; /*!< Available sensing methods related to Point or Cone based sensors */

    struct SensorPose{
        double x;/*!< sensor position x axis [m] */
        double y;/*!< sensor position y axis [m] */
        double theta;/*!< sensor angle [radians] */
    };
}

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class

/**
 * @brief Specifies the functionality for the Ranger Class, your Ranger
 * class must inherit from it. <b> You MUST NOT edit this file </b>.
 *
 */
class RangerInterface
{
public:
  RangerInterface(){};

  //Generates raw data for sensor
  virtual std::vector<double> generateData() = 0;

  /**
  Getter for Angular resolution
  @return angular resolution [deg]
  */
  virtual unsigned int getAngularResolution(void) = 0;

  /**
  Getter for sensor pose
  @return sensor pose
  */
  virtual ranger::SensorPose getSensorPose(void) = 0;

  /**
  Getter for field of view, for POINT based sensors FOV is zero
  @return field of view [deg]
  */
  virtual unsigned int getFieldOfView(void) = 0;

  /**
  Getter for maximum range
  @return maximum rage [m]
  */
  virtual double getMaxRange(void) = 0;

  /**
  Getter for mimimum range
  @return minimum rage [m]
  */
  virtual double getMinRange(void) = 0;

  /**
  Getter for sensing method
  @return Sensing Method \sa Sensging Method
  */
  virtual ranger::SensingMethod getSensingMethod(void) = 0;

  /**
  Set angular resolution method
  @param resolution in [degrees]
  @return true if resolution supported and actioned, false is not - previous setting used
  */
  virtual bool setAngularResolution(unsigned int resolution) = 0;

  /**
  Set sensor pose
  @param pose SensorPose
  @return true if offset actioned, false otherwise
  */
  virtual bool setSensorPose(ranger::SensorPose pose) = 0;

  /**
  Set field of view
  @param field of view [degrees]
  @return true if field of view actioned, false otherwise
  */
  virtual bool setFieldOfView(unsigned int fov) = 0;

};

#endif // RANGERFUSIONINTERFACE_H
