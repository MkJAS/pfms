#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"

using namespace std;


TEST (RangerFusionInterface, Laser) {

    vector<double> rawLaser = {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023};
    vector<double> rawSonar = {1.75965};

    RangerMockLaser r1(180, 30, {0.0,0.0,0.0}, rawLaser);
    RangerMockSonar r2(20, 20, {0.0,0.0,0.0}, rawSonar);
    RangerMockLaser r3(180, 30, {0.0,0.0,0.0}, rawLaser);

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0.0000,1.7597);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };

    RangerFusion rf(sensors);
    rf.setCells(cells);

    vector<vector<double>> data;

    data = rf.getRawRangeData();

    EXPECT_EQ(0,data.size()); // Should be zero as grab and fuse not called yet


    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();

    data = rf.getRawRangeData();

    ASSERT_EQ(3,data.size()); // Should be zero as grab and fuse not called yet
    EXPECT_TRUE(std::equal(rawLaser.begin(), rawLaser.end(), data.at(0).begin()));
    EXPECT_TRUE(std::equal(rawSonar.begin(), rawSonar.end(), data.at(1).begin()));
    EXPECT_TRUE(std::equal(rawLaser.begin(), rawLaser.end(), data.at(2).begin()));

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
