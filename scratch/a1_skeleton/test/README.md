## Compiling and Usage

Compile from this directroy via below
```
mkdir build; cd build; cmake ..; make; 
```

## Raw TESTS

Run the following command from within the build directory:
```
./rawTests; ./fusionTests
```

There is 1 test case group with two tests: LaserAndSonar and LaserAndSonarWithOffset

```
[==========] Running 2 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 2 tests from MultipleSensorsTest
[ RUN      ] MultipleSensorsTest.LaserAndSonarSimple
[       OK ] MultipleSensorsTest.LaserAndSonarSimple (0 ms)
[ RUN      ] MultipleSensorsTest.LaserAndSonarWithOffset
[       OK ] MultipleSensorsTest.LaserAndSonarWithOffset (0 ms)
[----------] 2 tests from MultipleSensorsTest (1 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test case ran. (1 ms total)
[  PASSED  ] 2 tests.

```

The tests map to criteria in the Google Marking Sheet highlighted Orange

## Fusion Tests


Run the following command from within the build directory:
```
./fusionTests
```

There are 2 test case groups 
*  SingleLaserFusionTest
 Test for LaserFree and LaserOccupied
*  DataFusionTest
 Test for Laser Misses and Sonar Intersects

```
[==========] Running 3 tests from 2 test cases.
[----------] Global test environment set-up.
[----------] 2 tests from SingleLaserFusionTest
[ RUN      ] SingleLaserFusionTest.LaserOccupied
[       OK ] SingleLaserFusionTest.LaserOccupied (0 ms)
[ RUN      ] SingleLaserFusionTest.LaserFree
[       OK ] SingleLaserFusionTest.LaserFree (0 ms)
[----------] 2 tests from SingleLaserFusionTest (0 ms total)

[----------] 1 test from DataFusionTest
[ RUN      ] DataFusionTest.LaserMissesSonarIntersects
[       OK ] DataFusionTest.LaserMissesSonarIntersects (0 ms)
[----------] 1 test from DataFusionTest (0 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 2 test cases ran. (0 ms total)
[  PASSED  ] 3 tests.
```


