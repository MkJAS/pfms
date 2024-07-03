#ifndef ANALYSISINTERFACE_H
#define ANALYSISINTERFACE_H

#include <vector>
#include "shape.h"
#include "line.h"

// The AnalysisInterface is a class which specifies the minimum
// required interface for your Analisys class must inherit from it
//
// Note that AnalysisInterface and Analysis only know about shape
// and have no knowledge of the Derived Classes
class AnalysisInterface
{
public:
    AnalysisInterface(){}; // Default contrsuctor is empty

    // Returns a container of bools if intersects exist
    virtual std::vector<bool> intersectsLine() = 0;

    // Return a vector with the count of circles, triangles and vectors respectively
    virtual std::vector<int> detectShapes() =0;

};

#endif // ANALYSISINTERFACE_H
