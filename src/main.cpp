/*
 * main.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */

//#include <Eigen/Dense>
#include <iostream>

#include "KalmanFilter.h"


int main(int argc, char** argv)
{
	double temp = 0 ;
	double tempCov = 0;

	double newTemp=0;
	double newCov=0;


	KalmanFilter testKF(temp, tempCov);



	testKF.publishState();

	for (int i = 0; i < 20; ++i)
	{
		std::cout << "timeStep: " << i << std::endl;

		testKF.takeMeasurement(newTemp+i, newCov+i);

		//testKF.log();
		testKF.iterate();
	}
	testKF.publishState();


	return 0;
}

