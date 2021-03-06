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

// this calculates the Kalman Filter based on current inputs
void KalmanFilter::Iterate()
{
	CalculateKalmanGain();
	CalculateEstimate();
	CalculeErrorInEstimate();
}

// this calculates the Kalman Gain
double KalmanFilter::CalculateKalmanGain()
{
	_kalman_gain = _error_in_estimate / (_error_in_estimate + _error_in_measurement) ;
	return _kalman_gain;
}

// this calculates the estimated location
double KalmanFilter::CalculateEstimate()
{
	_estimate_minus_1 = _estimate;

	_estimate = _estimate_minus_1 + _kalman_gain * ( _state - _estimate_minus_1);
	return _estimate;
}

// this calculates the error in the estimate
double KalmanFilter::CalculeErrorInEstimate()
{
	_error_in_estimate_minus_1 = _error_in_estimate;

	_error_in_estimate = (1- _kalman_gain)*_error_in_estimate_minus_1;

	return _error_in_estimate;
}


// this is calls the regular publish without the actual state
void KalmanFilter::PublishState()
{
	PublishState(NAN);
}

// this publishes or writes to the screen the current information of the system.
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

// this gets the current state estimate
double KalmanFilter::GetStateEstimate()
{
	return _estimate;
}

// this gets the current Coveriance
double KalmanFilter::GetCovariance()
{
	return _error_in_estimate;
}

// this calls the log function without the actual state
void KalmanFilter::Log()
{
	Log(NAN);
}

// this creates a comma seperated log file.
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

//this calles the take measurement function without an update to the Error in measurement
void KalmanFilter::TakeMeasurement(double par_in_state)
{
	TakeMeasurement(par_in_state, NAN);
}

// this takes a measurement and sets the error in measurement if needed.
void KalmanFilter::TakeMeasurement(double par_in_state, double ErrorInMeasurement)
{
	_state = par_in_state;


	if(ErrorInMeasurement != NAN)
	{
		_error_in_measurement = ErrorInMeasurement;
	}

}

