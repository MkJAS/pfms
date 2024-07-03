Quiz 3
======

Part A
------

We are in charge of a drag race for vehicles and will design a class `Analysis` to handle this. The class has two constructors (one of which takes a visualiser). Modify the main to create three vehicles with provided specifications. You will need to utilise the car library provided to determine how to create an object of `Car`, refer to class [Car](./a1/dep/include/car.h) . You will need to pass the cars to the [Analysis](./a1/analysis.h) for further processing, look at the constructor of `Analysis`.

We have a `demoRace` that is just a small demo to show how to accelerate and decelerate the cars.  You have to keep accelerating to go faster, and decelerate to slow down. You will need to create some cars to run the demo race, it shows the car on a circular track as they race. Unit tests are provided for Task 2 and Task 3.

**TASK 1- Sorting cars by odometry**

Implement the `sortByOdometry` sorting algorithm, that will sort the vehicles per kilometres in their odometry, sorted in ascending order). You can get some inspiration in completing this using pairs (the value to be sorted as first element, and the previous index as second element) from [sorting pars, Get some inspiration from](https://www.geeksforgeeks.org/keep-track-of-previous-indexes-after-sorting-a-vector-in-c-stl/)

Example below:

| Car Number | Place      | vector Content  |
| ---------- | ---------- | -------------- |
| Car 0      | 12788.9    | 0              |
| Car 1      | 68833.1    | 2              |
| Car 2      | 56686      | 1              |

**TASK 2 - Standard drag race **

Create a drag racing algorithm, where each vehicle will race a distance of 1km and then stop. So the logic for `dragRace` is:

  1. Accelerating each car 
  2. When all cars cover 1000m (1km) the race is finished, we declare the winners as the first cars to cover (travel) 1km. 

 Return the order of cars in terms of the order they reached the distance.

An example below:

| Car Number | Place                   | Vector Content |
| ---------- | ----------------------- | -------------- |
| Car 0      | 1st to cover 1km        | 0              |
| Car 1      | 3rd (last) to cover 1km | 2              |
| Car 2      | 2nd to cover 1km        | 1              |

HINTS: (1) use a loop to do this by going over a container of vehicles (2) what do we need to query at beginning of race (3) what function needs to be called during race (4) how to determine what functions needs to be called to terminate the race.

**TASK 3 - Stop all cars  **

Though we did a drag race, we didn't stop the vehicles (decelerate them to zero speed). Create a function `stopAllCars` in the `Analysis` class to achieve this. This function needs to stop (bring speed to zero) for all cars.

HINTS: (1) use a loop to do this by going over a container of vehicles, (2) what other information needs to be stored to know if to further decelerate each vehicle

**TASK 4 -  Zero - Top Speed - Zero race**

Initial drag race (in TASK 2) favoured high horse power, we have designed another race that is more about power to weight ratios and breaking force. All cars have to reach their individual top speed and then go back to zero speed.  So the logic for `zeroTopZeroRace` is

 1. Accelerate all cars
 2. As each individual vehicle reaches top speed decelerate that car and keep the others that have not reached their top speed accelerating
 3. When all vehicles have stopped, terminate the race

 Return the order of vehicles that reached zero speed (the car MUST have reached top speed before coming back to zero speed)

Example below:

| Car Number | Reached zero speed | Vector Content |
| ---------- | ------------------ | -------------- |
| Car 0      | 3rd (last)         | 2              |
| Car 1      | 1st                | 0              |
| Car 2      | 2nd                | 1              |



HINTS: (1) use a loop to do this by going over a container of vehicles (2) what function(s) needs to be called during race (3) what other information needs to be stored to know when to decelerate each vehicle and where would you store this


Part B
--------------------

You have been provided a library that implements a [Radar](./a2/dep/radar.h). sensor. The  header file describes the public functions available in  [Radar](./a2/dep/radar.h). The library is compiled, as such you only have the header file and the dynamic library.

The radar has been designed to cater for two possible max distances and this results in two scanning speeds (the scanning speed depends on the maximum distance we want the radar to detect targets from). The documentation for the sensor has no indication of the two scanning speeds (while the available distance settings are in the header file). Our goal is to empirically determine the scanning speed.

We will use the fact that the radar has a start function, which kicks off a thread inside the radar that continuously obtains radar data. We have created a thread that will run a function `computeScanningSpeed` and this function will gather a number of samples from the radar and use the time taken to get this number of samples. The scanning speed would therefore be the total time taken to get the samples, divided by the number of samples acquired (It is better to get the time as an average value over the number of samples so choose **samples = 100**) . For timing we can use chrono that comes with C++11 as it can not only time things, it also allows to determine a duration that from two times, [refer to this example for using chrono](https://en.cppreference.com/w/cpp/chrono/steady_clock/now)

**TASK 1 - Determine Scanning time for shorter of the max distance settings**

Set the max distance for the radar to shorter distance, implement `computeScanningSpeed`

**TASK 2 - Determine Scanning time for longer of the max distance settings**

Set the max distance for the radar to longer distance, check your implementation of `computeScanningSpeed`

