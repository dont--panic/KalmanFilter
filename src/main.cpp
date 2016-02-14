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
	double ititial_condition_temp = 35 ;
	double initial_condition_error_in_estimate = 2;
	double error_in_measurement = 1;
	double actual_temp = 50;



	KalmanFilter testKF(ititial_condition_temp, initial_condition_error_in_estimate, error_in_measurement);


	testKF.PublishState(actual_temp);

	for (int i = 0; i < 20; ++i)
	{
		actual_temp = 50;

		double r = ((double) rand() / (RAND_MAX)) - (error_in_measurement/2);

		testKF.TakeMeasurement(actual_temp+r, error_in_measurement);

		testKF.PublishState(actual_temp);

		testKF.Iterate();
	}
	testKF.PublishState(actual_temp);


	return 0;
}

