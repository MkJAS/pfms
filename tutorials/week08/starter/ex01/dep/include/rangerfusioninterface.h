#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"
#include "cell.h"

/**
 * @brief Specifies the required interface for your RangerFusion class your ranger fusion
// class must inherit from it, You MUST NOT edit this file.
 * 
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
     * @brief Calls each ranger to generate data and combines randomly generated sensor data with provided container of cells. Generates a 'fusion' of the data based on collision conditions.
     *
     */
    virtual void grabAndFuseData() = 0;

    /**
     * @brief Returns the raw data from any sensors in the ranger container - as per requirement C4
     * The raw data is updated every time a new fusion is requested. The raw data will match the preceeding fusion if it is called between fusions.
     *
     * @return std::vector<std::vector<double>>
     */
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

};

#endif // RANGERFUSIONINTERFACE_H
