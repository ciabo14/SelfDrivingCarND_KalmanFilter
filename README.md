# SelfDrivingCarND_KalmanFilter

Implementation of the Extended Kalman filter starting from the Udacity lessons. The Kalman filter is used for a sensor fusion application where data from lidar and radar are fused together for the estimation of the current position of an object

## Repository structure

The repository is made of 3 different folders:

1.  src: includes all the src files required for executable build and to execute the kalman filter;
2.  data: includes all the original data provided by udacity and the results output files;
3.  executable: includes the executable of the program;
4.  utilities: includes the python script to create the graphs starting from the data in the data folder. The path for the input/output file in the python are hard coded.

## USAGE instruction:

In order to execute the executable in the executable file you should follow the following usage instruction:

**Usage instructions:**
*
./ExtendedKF.exe \n
path/to/input.txt \n
path/to/output.txt \n
[r|l|b] for [radar only|laser only|both]
*
The executables is compiled in order to be executed with the single usage for lidar or radar or with the usage of both. The usage of a single sensor will select data from the input file in order to use only L|R or data from all the sensors.

##Results

The computed RMSE, also output of the executable, is stricty dependant on the sensor used. The results for the provided files are:

1. INPUT file 1
  a.  LASER ONLY
  Accuracy - RMSE (px,py,vx,vy) respectively:
  0.0989253
  0.117927
  0.613519
  0.551892
  b.  RADAR ONLY
  Accuracy - RMSE (px,py,vx,vy) respectively:
  0.161012
  0.167555
  0.609599
  0.576283
  c.  BOTH SENSORS 
  Accuracy - RMSE (px,py,vx,vy) respectively:
  0.0667526
  0.0604895
  0.53823
  0.541913
2. INPUT file 2
  a.  LASER ONLY
  0.218625
  0.195462
  0.528297
  0.422171
  b.  RADAR ONLY
  0.212349
  0.237221
  0.116234
  0.183869
  c.  BOTH SENSORS
  0.199216
  0.19122
  0.41498
  0.418222
