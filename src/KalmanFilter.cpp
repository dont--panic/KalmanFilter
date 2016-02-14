/*
 * KalmanFilter.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */


#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double par_initial_concition_state, double par_initial_concition_error_in_estimate, double par_initial_concition_error_in_measurement)
{
	_state = par_initial_concition_state;
	_error_in_estimate = par_initial_concition_error_in_estimate;
	_error_in_measurement = par_initial_concition_error_in_measurement;
}

KalmanFilter::~KalmanFilter(){}


void KalmanFilter::Iterate()
{
	CalculateKalmanGain();
	CalculateEstimate();
	CalculeErrorInEstimate();

}


double KalmanFilter::CalculateKalmanGain()
{
	_kalman_gain = _error_in_estimate / (_error_in_estimate + _error_in_measurement) ;
	return _kalman_gain;
}

double KalmanFilter::CalculateEstimate()
{
	_estimate_minus_1 = _estimate;

	_estimate = _estimate_minus_1 + _kalman_gain * ( _state - _estimate_minus_1);
	return _estimate;
}

double KalmanFilter::CalculeErrorInEstimate()
{
	_error_in_estimate_minus_1 = _error_in_estimate;

	_error_in_estimate = (1- _kalman_gain)*_error_in_estimate_minus_1;

	return _error_in_estimate;
}



void KalmanFilter::PublishState()
{
	PublishState(NAN);
}

void KalmanFilter::PublishState(double par_actual_state)
{
	std::ostringstream outstring;

	outstring << "State Estimate: " << _estimate << " Error In Estimate: " << _error_in_estimate << " Kalman Gain: " << _kalman_gain;

	if(par_actual_state != NAN)
	{
		outstring << " Actual State: " << par_actual_state ;
	}

	std::cout << outstring.str() << std::endl;
}

double KalmanFilter::GetState()
{
	return _state;
}

double KalmanFilter::GetCovariance()
{
	return _error_in_estimate;
}


void KalmanFilter::Log()
{
	Log(NAN);
}

void KalmanFilter::Log(double par_actual_state)
{
	std::ofstream stateLog;
	stateLog.open("KalmanFilterLog.csv", std::ios_base::app);
	stateLog << "State Estimate, " << _estimate << ",Error In Estimate, " << _error_in_estimate << ",Kalman Gain, " << _kalman_gain;

	if(par_actual_state != NAN)
	{
		stateLog << ",Actual State," << par_actual_state ;
	}

	stateLog << "\n";

	stateLog.close();

}

void KalmanFilter::TakeMeasurement(double par_in_state)
{
	TakeMeasurement(par_in_state, NAN);
}


void KalmanFilter::TakeMeasurement(double par_in_state, double ErrorInMeasurement)
{
	_state = par_in_state;


	if(ErrorInMeasurement != NAN)
	{
		_error_in_measurement = ErrorInMeasurement;
	}

}

