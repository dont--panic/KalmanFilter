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
	double ICtemp = 35 ;
	double ICErrorInEstimate = 2;
	double errorInMeasurement = 4;
	double actualTemp = 50;



	KalmanFilter testKF(ICtemp, ICErrorInEstimate, errorInMeasurement);


	testKF.publishState(actualTemp);

	for (int i = 0; i < 20; ++i)
	{
		actualTemp = 50 + 2*exp(-i);

		double r = ((double) rand() / (RAND_MAX)) - (errorInMeasurement/2);

		testKF.takeMeasurement(actualTemp+r, errorInMeasurement);

		testKF.publishState(actualTemp);

		testKF.iterate();
	}
	testKF.publishState(actualTemp);


	return 0;
}

