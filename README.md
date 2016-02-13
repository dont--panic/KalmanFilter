# KalmanFilter

The idea of creating this repository was to practice making a Kalman Filter in C++. This is a very interesting topic for me and I do hope to use this in some projects in the future. My intent is to make a Kalman Filter class that would support multiple different array sizes and Degrees Of Freedom (DOF), and it would eventually extend the standard Kalman Filter (KF) to include the Extended Kalman Filter (EKF) and the Unscented Kalman Filter (UKF). 

## Development Path

I will first make a plan of attack to allow the class to be built in stages. 

1. KF : Input is a scaler (1 variable, ex: temperature measurement) (linear assumption)
  * This will implement the base class of equations without matrices
2. KF : Input is now a matrix with x and xdot (1 DOF with position and velocity) (linear assumption)
  * This will extend the base class to include the ability to handle calculation in matrix form
  * This will also extend the calculation to include velocity as an input
3. KF : Input matrix with 2 DOF with position and velocity (linear assumption)
  * This will extend the matrix for 2 DOF
4. KF : Input matrix with 3 DOF with position and velocity (linear assumption)
  * This will extend the matrix for 3 DOF
5. EKF : Implement the Extended Kalman Filter for small non-linearity  
6. UKF : Implement the Unscented Kalman Filter for a better non-linear approximation





## Notes:

This project will need to have the following included:

Eigen3 from : https://bitbucket.org/eigen/eigen/