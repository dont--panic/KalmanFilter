/*
 * KalmanFilter.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: Joe
 */


#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double ICState, double ICErrorInEstimate, double ICErrorInMeasurement)
{
	_state = ICState;
	_errorInEstimate = ICErrorInEstimate;
	_errorInMeasurement = ICErrorInMeasurement;
}

KalmanFilter::~KalmanFilter(){}


void KalmanFilter::iterate()
{
	calculateKalmanGain();
	calculateEstimate();
	calculeErrorInEstimate();

}


double KalmanFilter::calculateKalmanGain()
{
	_KG = _errorInEstimate / (_errorInEstimate + _errorInMeasurement) ;
	return _KG;
}

double KalmanFilter::calculateEstimate()
{
	_estimateMin1 = _estimate;

	_estimate = _estimateMin1 + _KG * ( _state - _estimateMin1);
	return _estimate;
}

double KalmanFilter::calculeErrorInEstimate()
{
	_errorInEstimateMin1 = _errorInEstimate;

	_errorInEstimate = (1- _KG)*_errorInEstimateMin1;

	return _errorInEstimate;
}



void KalmanFilter::publishState()
{
	publishState(NAN);
}

void KalmanFilter::publishState(double actualState)
{
	std::ostringstream outstring;

	outstring << "State: " << _state << " Error In Estimate: " << _errorInEstimate;

	if(actualState != NAN)
	{
		outstring << " Actual State: " << actualState ;
	}

	std::cout << outstring.str() << std::endl;
}

double KalmanFilter::getState()
{
	return _state;
}

double KalmanFilter::getCovariance()
{
	return _errorInEstimate;
}


void KalmanFilter::log()
{
	log(NAN);
}

void KalmanFilter::log(double actualState)
{
	std::ofstream stateLog;
	stateLog.open("KalmanFilterLog.csv", std::ios_base::app);
	stateLog << _state << "," << _errorInEstimate ;

	if(actualState != NAN)
	{
		stateLog << "," << actualState ;
	}

	stateLog << "\n";

	stateLog.close();

}

void KalmanFilter::takeMeasurement(double InState)
{
	takeMeasurement(InState, NAN);
}


void KalmanFilter::takeMeasurement(double InState, double ErrorInMeasurement)
{
	_stateMin1 = _state;

	_state = InState;


	if(ErrorInMeasurement != NAN)
	{
		_errorInMeasurement = ErrorInMeasurement;
	}

}

