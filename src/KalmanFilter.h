/*
 * KalmanFilter.h
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */

#ifndef KalmanFilter_H_
#define KalmanFilter_H_

#include "KalmanFilter.h"

#include <math.h>
//#include <Eigen/Dense>
#include <iostream>

class KalmanFilter
{
public:
	KalmanFilter(double ICState, double ICStateCov);
	virtual ~KalmanFilter();

	void iterate();
	void publishState();
	double getState();
	double getCovariance();
	void log();
	void takeMeasurement(double InState, double InCov);


private:

	double _alpha = 0;
	double _beta = 0;
	double _kappa = 0;
	double _lambda = 0;
	double _c = 0;
	double _gamma = 0;

	double _measurement = 0;

	double _state = 0;
	double _cov = 0;
	double _stateMin1 = 0;
	double _covMin1 = 0;


};

#endif /* KalmanFilter_H_ */
