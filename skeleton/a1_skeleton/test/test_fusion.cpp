#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SingleLaserFusionTest, LaserOccupied) {

    //  Laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 0.0000
    //  ranges: 5.74748 1.92962 1.11586 2.51407 2.68647 3.50850 4.16129
    //  Cell parameters
    //  centre [x,y]=[0.0000,2.5141]
    //
    // Fusion OCCUPIED (LASER INTERSECTS)

    RangerMockLaser r1(180, 30, {0.0,0.0,0.0}, {5.74748, 1.92962, 1.11586, 2.51407, 2.68647, 3.50850, 4.16129});

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0.0000,2.5141);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}


TEST (SingleLaserFusionTest, LaserFree) {
    //Laser
    //res: 30.0000
    //pose: 0.0000, 0.0000, 0.0000
    //ranges: 6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292,
    //Cell parameters
    //centre [x,y]=[0.0000,5.1492]

    //
    // Fusion FREE (LASER Through);
    //

    RangerMockLaser r1(180, 30, {0.0,0.0,0.0}, {6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292});

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0.0000,5.1492);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1};
    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


TEST (DataFusionTest, LaserMissesSonarIntersects) {
    //  Parameters laser
    //  res: 30.0000
    //  pose: 0.0000, 0.0000, 0.0000
    //  ranges: 3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023,
    //  Sonar laser
    //  res: 20.0000
    //  pose: 0.0000, 0.0000, 0.0000
    //  ranges: 1.75965
    //  Cell parameters
    //  centre [x,y]=[0.0000,1.7597]
    //
    // Fusion OCCUPIED (LASER Misses, SONAR INTERSECTS);


    RangerMockLaser r1(180, 30, {0.0,0.0,0.0}, {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023});
    RangerMockSonar r2(20, 20, {0.0,0.0,0.0}, {1.75965});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0.0000,1.7597);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
