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

//////////////////////////////////////////////MY TESTS/////////////////////////////////////////////////////

///////////////////////////////////////////////TEST 1///////////////////////////////////////////////////////
TEST (UserLaserTests, angleRes30) {
/*
 * Laser
 * res: 30.0000
 * pose: 1.0, 2.0, 0 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *
*/
    RangerMockLaser r1(180, 30, {1.0, 2.0, 0}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});

    std::vector<Cell*> cells;
    for (unsigned long i=0; i<4; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1);
    }
    cells.at(0)->setCentre(2, 2);
    cells.at(1)->setCentre(5.5, 2);
    cells.at(2)->setCentre(1, 5);
    cells.at(3)->setCentre(-7.5, 8.5);

    std::vector<RangerInterface *> sensors = {&r1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(3)->getState());
}
//////////////////////////////////////////TEST 2////////////////////

TEST (UserLaserTests, thetaneg30) {
/*
 * Laser
 * res: 30.0000
 * pose: 1.0, 2.0, -30 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *
*
*/
    RangerMockLaser r1(180, 30, {1.0, 2.0, (-30*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});

    std::vector<Cell*> cells;
    for (unsigned long i=0; i<4; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1);
    }
    cells.at(0)->setCentre(2, 2);
    cells.at(1)->setCentre(5.5, 2);
    cells.at(2)->setCentre(1, 5);
    cells.at(3)->setCentre(-7.5, 8.5);

    std::vector<RangerInterface *> sensors = {&r1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
    EXPECT_EQ(cell::FREE,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(3)->getState());
}

/////////////////////////////////////////////////TEST 3/////////////////////////////
TEST (UserLaserTests, thetaneg45) {
/*
 * Laser
 * res: 30.0000
 * pose: -3.0, 2.0, -45 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *
*/
    RangerMockLaser r1(180, 30, {-3.0, 2.0, (-45*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});

    std::vector<Cell*> cells;
    for (unsigned long i=0; i<4; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(0.5);
    }
    cells.at(0)->setCentre(0, 5);
    cells.at(1)->setCentre(-7.62, 6.79);
    cells.at(2)->setCentre(-1, 0);
    cells.at(3)->setCentre(-7.5, 8.5);

    std::vector<RangerInterface *> sensors = {&r1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(3)->getState());
}

///////////////////////////////////////////TEST 4///////////////////////////////
TEST (UserLaserTests, TwoLasers) {
/*
 * Laser 1
 * res: 30.0000
 * pose: -4.0, 2.0, -50 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 * Laser 2 
 * res: 30.0000
 * pose: 4.0, 2.0, 45 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *
*/
    RangerMockLaser r1(180, 30, {-4.0, 2.0, (-50*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});
    RangerMockLaser r2(180, 30, {4.0, 2.0, (45*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});

    std::vector<Cell*> cells;
    for (unsigned long i=0; i<4; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(0.6);
    }
    cells.at(0)->setCentre(3.5, 3.5);
    cells.at(1)->setCentre(-7.62, 6.79);
    cells.at(2)->setCentre(-1, 0);
    cells.at(3)->setCentre(0, 3);

    std::vector<RangerInterface *> sensors = {&r1,&r2};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
    EXPECT_EQ(cell::FREE,cells.at(1)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(2)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(3)->getState());
}

/////////////////////////////////TEST 5/////////////////////////

TEST (UserLaserTests, TwoLasersv2) {
/*
 * Laser 1
 * res: 30.0000
 * pose: -4.0, 2.0, 180 deg
 * ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 * Laser 2 
 * res: 10
 * pose: 2.0, -2.5.0, 45 deg
 * ranges: 6.9, 5.58, 4.35, 4.25, 7.56, 2.1, 1.98,5.68,6.58,4.25,1.05,7.68,
    5.36,4.23,5.78,2.45,5.69,5.69,5.23
 *
*/
    RangerMockLaser r1(180, 30, {-4.0, 2.0, 180*M_PI/180}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});
    RangerMockLaser r2(180, 10, {2, -2.5, 45*M_PI/180}, {6.9, 5.58, 4.35, 4.25, 7.56, 2.1, 1.98,5.68,6.58,4.25,1.05,7.68,
    5.36,4.23,5.78,2.45,5.69,5.69,5.23});

    std::vector<Cell*> cells;
    for (unsigned long i=0; i<4; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1.2);
    }
    cells.at(0)->setCentre(-4, 0);
    cells.at(1)->setCentre(-5, 0.58);
    cells.at(2)->setCentre(-1, 0);
    cells.at(3)->setCentre(1.5, 0.3);

    std::vector<RangerInterface *> sensors = {&r1,&r2};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(1)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(2)->getState());
    EXPECT_EQ(cell::FREE,cells.at(3)->getState());
}

//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

////////////////////////////////SONAR TESTS/////////////////////////////////

///////////////////////////////TEST 1////////////////////////////////////////

TEST (UserSonarTests, singleSonar) {
    //  Sonar
    //  res: 20.0000
    //  pose: 0.0000, 0.0000, 0.0000
    //  ranges: 6.5

    RangerMockSonar s1(20, 20, {0.0,0.0,0.0}, {6.5});
    std::vector<Cell*> cells;
    for (int i = 0; i <6; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1.22);
    }

    cells.at(0)->setCentre(0,6);
    cells.at(1)->setCentre(-0.2,1.5);
    cells.at(2)->setCentre(0.3,2.7);
    cells.at(3)->setCentre(1.1,4.44);
    cells.at(4)->setCentre(2.1,3);
    
    
    std::vector<RangerInterface *> sensors = {&s1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
    EXPECT_EQ(cell::FREE,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::FREE,cells.at(3)->getState());
    EXPECT_EQ(cell::UNKNOWN,cells.at(4)->getState());
}

//////////////////////////////////////////////TEST 2////////////////////////////

TEST (UserSonarTests, theta60) {
    //  Sonar
    //  res: 20.0000
    //  pose: 3,-2.5,60
    //  ranges: 8.9

    RangerMockSonar s1(20, 20, {3,-2.5,60*M_PI/180}, {8.9});
    std::vector<Cell*> cells;
    for (int i = 0; i <6; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(0.76);
    }

    cells.at(0)->setCentre(0,0);
    cells.at(1)->setCentre(-3.8,3.2);
    cells.at(2)->setCentre(-4.5,2.7);
    cells.at(3)->setCentre(-4,0);
    cells.at(4)->setCentre(-3,1);
    
    
    std::vector<RangerInterface *> sensors = {&s1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(1)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(2)->getState());
    EXPECT_EQ(cell::FREE,cells.at(3)->getState());
    EXPECT_EQ(cell::FREE,cells.at(4)->getState());
}

//////////////////////////////////////////////TEST 3////////////////////////////

TEST (UserSonarTests, 2sonar) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: 3,-2.5,60
    //  ranges: 8.9
    //  Sonar 2
    //  res: 20.0000
    //  pose: 4,4.5,130
    //  ranges: 9.8

    RangerMockSonar s1(20, 20, {3,-2.5,60*M_PI/180}, {8.9});
    RangerMockSonar s2(20, 20, {4,4.5,130*M_PI/180}, {9.8});
    std::vector<Cell*> cells;
    for (int i = 0; i <6; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1.35);
    }

    cells.at(0)->setCentre(0,0);
    cells.at(1)->setCentre(4,4.5);
    cells.at(2)->setCentre(-1.5,2);
    cells.at(3)->setCentre(-4,0);
    cells.at(4)->setCentre(-3,1);
    
    
    std::vector<RangerInterface *> sensors = {&s1,&s2};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());
    EXPECT_EQ(cell::FREE,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(3)->getState());
    EXPECT_EQ(cell::FREE,cells.at(4)->getState());
}



//////////////////////////////////////////////TEST 4////////////////////////////

TEST (UserSonarTests, SonarandLaser) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: 3,-2.5,60
    //  ranges: 8.9
    
    /*
 *  Laser 1
 *  res: 30.0000
 *  pose: -4.0, 2.0, 180 deg
 *  ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *
*/
    RangerMockLaser r1(180, 30, {-1, 3.0, (180*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});

    RangerMockSonar s1(20, 20, {3,-2.5,(60*M_PI/180)}, {8.9});
    
    std::vector<Cell*> cells;
    for (int i = 0; i <6; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(3.5);
    }

    cells.at(0)->setCentre(0,0);
    cells.at(1)->setCentre(4,4.5);
    cells.at(2)->setCentre(-1.5,2);
    cells.at(3)->setCentre(-4,0);
    cells.at(4)->setCentre(-3,1);
    
    
    std::vector<RangerInterface *> sensors = {&s1,&r1};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
    EXPECT_EQ(cell::FREE,cells.at(1)->getState());
    EXPECT_EQ(cell::FREE,cells.at(2)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(3)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(4)->getState());
}

//////////////////////////////////////////////TEST 5////////////////////////////

TEST (UserSonarTests, 2Sonarand2Laser) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: -5,10,-60
    //  ranges: 8.9
    //  Sonar 2
    //  res: 20.0000
    //  pose: 4,4.5,130
    //  ranges: 9.8
    /*
 *  Laser 1
 *  res: 30.0000
 *  pose: -4.0, 2.0, 200 deg
 *  ranges: 3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85
 *  Laser 2
 *  res: 30.0000
 *  pose: -6, -5.5, 45 deg
 *  ranges: 6.9, 5.58, 4.35, 4.25, 7.56, 2.1, 1.98,5.68,6.58,4.25,1.05,7.68,
    5.36,4.23,5.78,2.45,5.69,5.69,5.23
 * 
 * 
 *
*/
    RangerMockLaser r1(180, 30, {-4, 3.0, (200*M_PI/180)}, {3.58, 6.58, 7.5, 4.25, 4.23, 3.54, 6.85});
    RangerMockLaser r2(180, 10, {-6, -5.5, (45*M_PI/180)}, {6.9, 5.58, 4.35, 4.25, 7.56, 2.1, 1.98,5.68,6.58,4.25,1.05,7.68,
    5.36,4.23,5.78,2.45,5.69,5.69,5.23});

    RangerMockSonar s1(20, 20, {-5,-10,0}, {8.9});
    RangerMockSonar s2(20, 20, {4,4.5,(130*M_PI/180)}, {9.8});
    
    std::vector<Cell*> cells;
    for (int i = 0; i <6; i++) {
        cells.push_back(new Cell());
        cells.at(i)->setSide(1.25);
    }

    cells.at(0)->setCentre(-6.5,-3.2);
    cells.at(1)->setCentre(-5.5,-1.4);
    cells.at(2)->setCentre(-2.5,-2);
    cells.at(3)->setCentre(-5,-3);
    cells.at(4)->setCentre(-3,1);
    
    
    std::vector<RangerInterface *> sensors = {&s1,&r1,&s2,&r2};

    RangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(1)->getState());
    EXPECT_EQ(cell::OCCUPIED,cells.at(2)->getState());
    EXPECT_EQ(cell::FREE,cells.at(3)->getState());
    EXPECT_EQ(cell::FREE,cells.at(4)->getState());
}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

///////////////////A//////////////////AREA TESTS/////////////////////////
//////////////////////////////////////////////TEST 1////////////////////////////

TEST (UserAreaTests, 1Sonar) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: -3,-2.5,-60
    //  ranges: 8.9
    //  Sonar 2
    //  res: 20.0000
    //  pose: 4,4.5,130
    //  ranges: 8.9
    /*
*/

    RangerMockSonar s1(20, 20, {-5,-10,0}, {10});
    RangerMockSonar s2(20, 20, {4,4.5,130}, {10});
    
        std::vector<RangerInterface *> sensors = {&s1};

    RangerFusion rf(sensors);
    
    //Pull the raw data into internal storage variable
    double area = rf.getScanningArea();
    EXPECT_NEAR(area,17.633,1e-3);

}

///////////////////////////////////////////TEST 2/////////////////////////

TEST (UserAreaTests, 3Sonars) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: 1,1,0
    //  ranges: 10
    //  Sonar 2
    //  res: 20.0000
    //  pose: 3,0,0
    //  ranges: 10
    //  Sonar 3
    //  res: 20.0000
    //  pose: 2,-3,0
    //  ranges: 10
    /*
*/

    RangerMockSonar s1(20, 20, {1,1,0}, {10});
    RangerMockSonar s2(20, 20, {3,0,0}, {10});
    RangerMockSonar s3(20, 20, {2,-3,0}, {10});
    
        std::vector<RangerInterface *> sensors = {&s1,&s2,&s3};

    RangerFusion rf(sensors);
    
    //Pull the raw data into internal storage variable
    double area = rf.getScanningArea();
    EXPECT_NEAR(area,39.0036,0.25);

}

///////////////////////////////////////////TEST 3/////////////////////////

TEST (UserAreaTests, 2Sonars) {
    //  Sonar 1
    //  res: 20.0000
    //  pose: 1,1,0
    //  ranges: 10
    //  Sonar 2
    //  res: 20.0000
    //  pose: 3,0,0
    //  ranges: 10
    

    RangerMockSonar s1(20, 20, {1,1,0}, {10});
    RangerMockSonar s2(20, 20, {3,0,0}, {10});
    
    
        std::vector<RangerInterface *> sensors = {&s1,&s2};

    RangerFusion rf(sensors);
    
    //Pull the raw data into internal storage variable
    double area = rf.getScanningArea();
    EXPECT_NEAR(area,32.6639,1);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
