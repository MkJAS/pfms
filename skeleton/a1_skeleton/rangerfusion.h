#ifndef RANGERFUSION_H
#define RANGERFUSION_H

#include <vector>
#include "rangerfusioninterface.h"
#include "rangerinterface.h"

class RangerFusion: public RangerFusionInterface
{
public:
    /**
    The Default constructor sets the cell centre to values within the #MAP_SIZE\n
    @sa RangerFusionInterface and @sa RangerInterface for more information
    */
  RangerFusion(std::vector<RangerInterface*> rangers);


private:
  //
  std::vector<std::vector<double>> data_; //!< This is to cater for getRawRangeData (which returns the raw data that was used for fusion))
  std::vector<RangerInterface*> rangers_; //!< A private copy of rangers @sa RangerInterface
  std::vector<Cell*> cells_; //!< A private copy of cells @sa Cell

};

#endif // RANGERFUSION_H
