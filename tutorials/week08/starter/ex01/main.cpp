#include "sonar.h"
#include "laser.h"
#include "cell.h"
#include "rangerfusion.h"

#include <iostream>
#include <chrono>
#include <thread>

using std::cout;
using std::endl;
using std::cin;
using std::vector;

std::string strState(cell::State state)
{
    // converts a cell state to a string
    if (state == cell::OCCUPIED) return "occupied";
    else if (state == cell::FREE) return "free";
    else return "unknown";
}

void printFixedSensorParameters(Ranger* sensor)
{
    cout << "Data type: " << sensor->getSensingMethod() << endl;
    cout << "Range: " << sensor->getMinRange() << "-" << sensor->getMaxRange() << "m" << endl;
    cout << "FOV: " << sensor->getFieldOfView() << " deg" << endl;
}

void printVariableSensorParameters(Ranger* sensor)
{
    ranger::SensorPose pose = sensor->getSensorPose();
    cout << "pose " << pose.x << " " << pose.y << " " << pose.theta << endl;
    if (sensor->getAngularResolution()) {
        cout << "Angular resolution: " << sensor->getAngularResolution() << " deg" << endl;
    }
}

int main()
{
    // initialise the sensors
    Laser laser1;
    Sonar sonar1, sonar2;

    vector<RangerInterface*> rangers = {&laser1, &sonar1, &sonar2};

    // query fixed sensor parameters
    cout <<  "---Fixed sensor parameters---" << endl;
    for (auto sensor : rangers)
    {
        printFixedSensorParameters((Ranger*)sensor);
        cout << endl;
    }

    // set variable sensor parameters
    sonar1.setSensorPose({0,0,-30*M_PI/180});
    sonar2.setSensorPose({0,0,30*M_PI/180});

    // query variable sensor parameters
    cout <<  "---Variable sensor parameters---" << endl;
    for (auto sensor : rangers)
    {
        printVariableSensorParameters((Ranger*)sensor);
        cout << endl;
    }

    vector<Cell*> cells;

    // request number of cells from user
    unsigned int n_cells;
    cout << "Input number of cells: "; cin >> n_cells;
    for (int i = 0; i < n_cells; ++i)
    {
        cells.push_back(new Cell);
        cells.back()->setSide(1); // make cell bigger so that it's more likely to intersect
    }

    RangerFusion fusion(rangers);
    fusion.setCells(cells);


    // continuously print to screen the status of the cells
    while (1)
    {
        fusion.grabAndFuseData();

        // print cells
        cout << endl;
        for (auto c : cells)
        {
            double cellx, celly;
            c->getCentre(cellx,celly);
            std::cout << "Cell: (" << cellx << ", " << celly << "), State: " << strState(c->getState()) << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
