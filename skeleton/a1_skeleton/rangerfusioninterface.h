#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"
#include "cell.h"

/**
 * @brief Specifies the required interface for your RangerFusion class your ranger fusion
// class must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */

/*!
 *  \brief     Ranger Interface Class
 *  \details
 *  Specifies the required interface for your RangerFusion class your ranger fusion
 * class must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning   students MUST NOT change this class (the header file)
 */


class RangerFusionInterface
{
public:
    RangerFusionInterface(){};

    /**
     * @brief Accepts the container of cells.
     *
     * @param cells
     */
    virtual void setCells(std::vector<Cell*> cells) = 0;

    /**
     * @brief Does two operations (1) Calls each ranger to generate data and uses this data to determine colissions with provided container of cells (2) Generates a 'fusion' of the data based on collision conditions as descibed in Assignment 2 specification
     *
     */
    virtual void grabAndFuseData() = 0;

    /**
     * @brief Returns the raw data from all sensors in the ranger container within a vector of vectors
     * The raw data is updated every time a new fusion is requested (grabAndFuseDat). The raw data is the data used for collision checking. If no fusion has occured the vector shall be empty.
     *
     * @return std::vector<std::vector<double>>  the outer elements of the vector related to the rangers, the inner elements of vector are the respective range readings

     *
     * @sa grabAndFuseData
     */
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    /**
     * @brief Returns the total scanning area possible with CONE based scanners supplied
     * A union of all areas https://en.wikipedia.org/wiki/Union_(set_theory)
     *
     * @return double Total area coverage
     *
     * @sa grabAndFuseData
     */
    virtual double getScanningArea() = 0;


};

#endif // RANGERFUSIONINTERFACE_H
