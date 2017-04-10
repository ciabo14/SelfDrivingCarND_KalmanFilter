# SelfDrivingCarND_KalmanFilter

Implementation of the Extended Kalman filter starting from the Udacity lessons. The Kalman filter is used for a sensor fusion application where data from lidar and radar are fused together for the estimation of the current position of an object

## Repository structure

The repository is made of 3 different folders:
src: includes all the src files required for executable build and to execute the kalman filter
data: includes all the original data provided by udacity and the results output files
executable: includes the executable of the program.
utilities: includes the python script to create the graphs starting from the data in the data folder. The path for the input/output file in the python are hard coded.


## USAGE instruction:
the 



Usage instructions:
./ExtendedKF.exe
path/to/input.txt
path/to/output.txt
[r|l|b] for [radar only|laser only|both]
