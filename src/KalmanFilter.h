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
#include <fstream>
#include <random>
#include <sstream>

class KalmanFilter
{
public:
	KalmanFilter(double ICState, double ICErrorInEstimate, double ICErrorInMeasurement);
	virtual ~KalmanFilter();

	void Iterate();
	void PublishState();
	void PublishState(double actualState);
	double GetState();
	double GetCovariance();
	void Log();
	void Log(double actualState);
	void TakeMeasurement(double InState);
	void TakeMeasurement(double InState, double ErrorInMeasurement);

private:

	double _alpha = 0;
	double _beta = 0;
	double _kappa = 0;
	double _lambda = 0;
	double _c = 0;
	double _gamma = 0;

	double _measurement = 0;
	double _error_in_measurement = 0;
	double _error_in_estimate = 0;
	double _error_in_estimate_minus_1 =0;

	double _kalman_gain = 0;
	double _estimate = 0;
	double _estimate_minus_1 = 0;

	double _state = 0;


	double CalculateKalmanGain();
	double CalculateEstimate();
	double CalculeErrorInEstimate();

};

#endif /* KalmanFilter_H_ */
