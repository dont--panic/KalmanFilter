/*
 * main.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */

//#include <Eigen/Dense>
#include <iostream>
#include <random>
#include "KalmanFilter.h"
#include <cmath>


int main(int argc, char** argv)
{
	// this is the initial value, in this case its temperature
	double ititial_condition_temp = 35 ;
	// this is the initial error estimate
	double initial_condition_error_in_estimate = 2;
	// this is the error in the measurement
	double error_in_measurement = 1;
	// this is the actual temperature
	double actual_temp = 50;

	// this initializes the filter
	KalmanFilter testKF(ititial_condition_temp, initial_condition_error_in_estimate, error_in_measurement);

	for (int i = 0; i < 20; ++i)
	{
		// this creates a random noise to add to the signal
		double r = ((double) rand() / (RAND_MAX)) - (error_in_measurement/2);

		// this simulates taking a measurement
		testKF.TakeMeasurement(actual_temp+r, error_in_measurement);

		// this prints out the values to the screen.
		testKF.PublishState(actual_temp);

		// this iterates the calculation
		testKF.Iterate();
	}
	testKF.PublishState(actual_temp);


	return 0;
}

