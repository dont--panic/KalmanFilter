/*
 * KalmanFilter.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */


#include "KalmanFilter.h"
#include <fstream>
#include <random>

KalmanFilter::KalmanFilter(double ICState, double ICStateCov)
{
	_state = ICState;
	_cov = ICStateCov;
}

KalmanFilter::~KalmanFilter()
{

}


void KalmanFilter::iterate()
{

}

void KalmanFilter::publishState()
{

	std::cout << "State:" << std::endl;
	std::cout << _state << std::endl;
	std::cout << std::endl;
	std::cout << "Covariance" << std::endl;
	std::cout << _cov << std::endl;


}

double KalmanFilter::getState()
{
	return _state;
}

double KalmanFilter::getCovariance()
{
	return _cov;
}

void KalmanFilter::log()
{

}

void KalmanFilter::takeMeasurement(double InState, double InCov)
{
	_stateMin1 = _state;
	_covMin1 = _cov;

	_state = InState;
	_cov = InCov;
}

