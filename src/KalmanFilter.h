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

	void iterate();
	void publishState();
	void publishState(double actualState);
	double getState();
	double getCovariance();
	void log();
	void log(double actualState);
	void takeMeasurement(double InState);
	void takeMeasurement(double InState, double ErrorInMeasurement);

private:

	double _alpha = 0;
	double _beta = 0;
	double _kappa = 0;
	double _lambda = 0;
	double _c = 0;
	double _gamma = 0;

	double _measurement = 0;
	double _errorInMeasurement = 0;
	double _errorInMeasurementMin1 = 0;
	double _errorInEstimate = 0;
	double _errorInEstimateMin1 =0;

	double _KG = 0;
	double _estimate = 0;
	double _estimateMin1 = 0;

	double _state = 0;
	double _stateMin1 = 0;

	double _actualState = 0;


	double calculateKalmanGain();
	double calculateEstimate();
	double calculeErrorInEstimate();

};

#endif /* KalmanFilter_H_ */
