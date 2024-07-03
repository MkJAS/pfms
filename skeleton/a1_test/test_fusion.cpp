#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "../rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermockradar.h"


//Test that multiple sensors produce a larger rawDataArray
TEST (MultipleSensorsTest, LaserAndSonarSimple) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, offset=0}
  //  - (1) Sonar {fov=60, angular_res=20, offset=0}
  //
  // Produces two vectors, the first with 7 measurements and the second with
  // 1.
  Sonar s;
  s.setOffset(0);

  Laser l;
  l.setAngularResolution(30);
  l.setOffset(0);

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf;
  rf.setRangers(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();


  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(7, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}

TEST (MultipleSensorsTest, LaserAndSonarWithOffset) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=10, offset=0}
  //  - (1) Sonar {fov=60, angular_res=20, offset=90}
  //
  // Offset should not alter the raw data produced.
  // Produces two vectors, the first with 19 measurements and the second with
  // 3.
  Sonar s;
  s.setOffset(90);

  Laser l;
  l.setAngularResolution(10);
  l.setOffset(0);

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf;
  rf.setRangers(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();
  rf.getRawRangeData();

  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(19, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SingleLaserFusionTest, LaserOccupied) {

  //  Laser
  //  res: 30.0000
  //  off: 0.0000
  //  ranges: 5.74748 1.92962 1.11586 2.51407 2.68647 3.50850 4.16129
  //  Cell parameters
  //  centre [x,y]=[0.0000,2.5141]
  //
  // Fusion OCCUPIED (LASER INTERSECTS)

  RangerMockLaser r1(180, 30, 0, {5.74748, 1.92962, 1.11586, 2.51407, 2.68647, 3.50850, 4.16129});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,2.5141);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1};
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (SingleLaserFusionTest, LaserMisses) {
//  Laser
//  res: 30.0000
//  off: 0.0000
//  ranges: 3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767,
//  Cell parameters
//  centre [x,y]=[0.0000,8.2649]
//
// Fusion OCCUPIED (LASER Misses);


  RangerMockLaser r1(180, 30, 0, {3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,8.2649);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1};
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::UNKNOWN};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::UNKNOWN,cells.at(0)->getState());
}


TEST (SingleLaserFusionTest, LaserFree) {
//Laser
//res: 30.0000
//off: 0.0000
//ranges: 6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292,
//Cell parameters
//centre [x,y]=[0.0000,5.1492]

//
// Fusion FREE (LASER Through);
//

  RangerMockLaser r1(180, 30, 0, {6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,5.1492);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1};
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::FREE,cells.at(0)->getState());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (SingleSonarFusionTest, SonarUnknown) {

  //  Sonar
  //  res: 20.0000
  //  off: 0.0000
  //  ranges: 1.55115
  //  Cell parameters
  //  centre [x,y]=[0.0000,2.5141]
  //
  // Fusion UNKNOWN

  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,2.5141);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::UNKNOWN};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::UNKNOWN,cells.at(0)->getState());
}

TEST (SingleSonarFusionTest, SonarOccupied) {

  //  Sonar
  //  res: 20.0000
  //  off: 0.0000
  //  ranges: 1.55115
  //  Cell parameters
  //  centre [x,y]=[0.0000,2.5141]
  //
  // Fusion UNKNOWN

  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,1.55115);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (SingleSonarFusionTest, SonarFree) {

  //  Sonar
  //  res: 20.0000
  //  off: 0.0000
  //  ranges: 1.55115
  //  Cell parameters
  //  centre [x,y]=[0.0000,2.5141]
  //
  // Fusion UNKNOWN

  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,1.25);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::FREE,cells.at(0)->getState());
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


TEST (DataFusionTest, LaserIntersectsSonarFree) {
//  Laser
//  res: 30.0000
//  off: 0.0000
//  ranges: 3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767,
//  Sonar
//  res: 20.0000
//  off: 0.0000
//  ranges: 10.93034
//  Cell parameters
//  centre [x,y]=[0.0000,4.2649]
//
// Fusion OCCUPIED (LASER INTERSECTS, SONAR indicates FREE);


  RangerMockLaser r1(180, 30, 0, {3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767});
  RangerMockSonar r2(20, 20, 0, {10.93034});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,4.2649);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (DataFusionTest, LaserMissesSonarIntersects_Edge) {
//  Parameters laser
//  res: 30.0000
//  off: 0.0000
//  ranges: 3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023,
//  Sonar laser
//  res: 20.0000
//  off: 0.0000
//  ranges: 1.75965
//  Cell parameters
//  centre [x,y]=[0.3056,1.7329]
//
// Fusion OCCUPIED (LASER Misses, SONAR INTERSECTS);
//
// In case students take the reported range at the edge of sonar

  RangerMockLaser r1(180, 30, 0, {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023});
  RangerMockSonar r2(20, 20, 0, {1.75965});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.3056,1.7329);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (DataFusionTest, LaserMissesSonarIntersects_Centre) {
//  Parameters laser
//  res: 30.0000
//  off: 0.0000
//  ranges: 3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023,
//  Sonar laser
//  res: 20.0000
//  off: 0.0000
//  ranges: 1.75965
//  Cell parameters
//  centre [x,y]=[0.0000,1.7597]
//
// Fusion OCCUPIED (LASER Misses, SONAR INTERSECTS);
//
// In case students take the reported range at the centre of sonar

  RangerMockLaser r1(180, 30, 0, {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023});
  RangerMockSonar r2(20, 20, 0, {1.75965});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,1.7597);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}


TEST (DataFusionTest, LaserFreeSonarMisses) {
//Laser
//res: 30.0000
//off: 0.0000
//ranges: 6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292,
//Sonar
//res: 20.0000
//off: 0.0000
//ranges: 0.68255
//Cell parameters
//centre [x,y]=[0.0000,5.1492]

//
// Fusion FREE (LASER Through, SONAR Misses);
//
// In case students take the reported range at the centre of sonar

  RangerMockLaser r1(180, 30, 0, {6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292});
  RangerMockSonar r2(20, 20, 0, {0.68255});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,5.1492);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::FREE,cells.at(0)->getState());
}

TEST (DefaultDataFusionTest, LaserFreeSonarsMiss) {

//Laser
//res: 30.0000
//off: 0.0000
//ranges: 6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819,
//Sonar
//res: 20.0000
//off: -30.0000
//ranges: 4.81335
//Sonar 2
//res: 20.0000
//off: 30.0000
//ranges: 7.02009
//Cell parameters
//centre [x,y]=[0.0000,3.3096]


  RangerMockLaser r1(180, 30, 0, {6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819});
  RangerMockSonar r2(20, 20, -30, {4.81335});
  RangerMockSonar r3(20, 20, 30, {7.02009});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,3.3096);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::FREE,cells.at(0)->getState());
}

TEST (DefaultDataFusionTest, LaserIntersectsSonarsMiss) {

//Laser
//res: 30.0000
//off: 0.0000
//ranges: 6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819,
//Sonar
//res: 20.0000
//off: -30.0000
//ranges: 4.81335
//Sonar 2
//res: 20.0000
//off: 30.0000
//ranges: 7.02009
//Cell parameters
//centre [x,y]=[0.0000,4.3096]


  RangerMockLaser r1(180, 30, 0, {6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819});
  RangerMockSonar r2(20, 20, -30, {4.81335});
  RangerMockSonar r3(20, 20, 30, {7.02009});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,4.3096);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (DefaultDataFusionTest, LaserMissSonarsFree) {

//Laser
//res: 30.0000
//off: 0.0000
//ranges: 6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819,
//Sonar
//res: 20.0000
//off: -30.0000
//ranges: 4.81335
//Sonar 2
//res: 20.0000
//off: 30.0000
//ranges: 7.02009
//Cell parameters
//centre [x,y]=[-2.5000,4.3096]


  RangerMockLaser r1(180, 30, 0, {6.46624,0.67168,3.31421,4.30963,3.45104,5.32351,5.09819});
  RangerMockSonar r2(20, 20, -30, {4.81335});
  RangerMockSonar r3(20, 20, 30, {7.02009});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-2.5000,4.3096);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::FREE};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::FREE,cells.at(0)->getState());
}

TEST (DefaultDataFusionTest, LaserMissOneSonarFreeOtherOccupied) {

//  Laser
//  res: 30.0000
//  off: 0.0000
//  ranges: 1.53474,0.45429,4.57736,7.07856,5.41957,1.68538,3.07755,
//  Sonar
//  res: 20.0000
//  off: 30.0000
//  ranges: 7.47947
//  Sonar 2
//  res: 20.0000
//  off: 30.0000
//  ranges: 15.70988
//  Cell parameters
//  centre [x,y]=[-2.5581,7.0284]

  RangerMockLaser r1(180, 30, 0, {1.53474,0.45429,4.57736,7.07856,5.41957,1.68538,3.07755});
  RangerMockSonar r2(20, 20, 20, {7.47947});
  RangerMockSonar r3(20, 20, 20, {15.70988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-2.5581,7.0284);
  cells.at(0)->setSide(1.0);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::OCCUPIED};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::OCCUPIED,cells.at(0)->getState());
}

TEST (MultiSonarFusionTest, AllSenorsMiss) {

  RangerMockLaser r1(180, 30, 0, {1.53474,0.45429,4.57736,7.07856,5.41957,1.68538,3.07755});
  RangerMockSonar r2(20, 20, 0, {7.47947});
  RangerMockSonar r3(20, 20, 30, {15.70988});
  RangerMockSonar r4(20, 20, -30, {15.70988});
  RangerMockSonar r5(20, 20, 60, {15.70988});
  RangerMockSonar r6(20, 20, -60, {15.70988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.5,8.0);
  cells.at(0)->setSide(0.2);

  RangerFusion rf;
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 ,&r4 ,&r5 ,&r6 };
  rf.setRangers(sensors);
  rf.setCells(cells);

  State expected = {State::UNKNOWN};

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(State::UNKNOWN,cells.at(0)->getState());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
