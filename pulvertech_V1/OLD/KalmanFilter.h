//#include "Arduino.h"
#include <math.h>

//#ifndef SimpleKalmanFilter_h
//#define SimpleKalmanFilter_h


/*Application example:

  SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);
  float x = analogRead(A0);
  float estimated_x = kf.updateEstimate(x);
*/

/*
  e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
   e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
   q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.

 */

class SimpleKalmanFilter
{

public:
    SimpleKalmanFilter(float mea_e, float est_e, float q);
    float updateEstimate(float mea);
    void setMeasurementError(float mea_e);
    void setEstimateError(float est_e);
    void setProcessNoise(float q);
    float getKalmanGain();

private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;

};

//#endif



SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
{
    _err_measure = mea_e;
    _err_estimate = est_e;
    _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;

    return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
    _err_measure = mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
    _err_estimate = est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
    _q = q;
}

float SimpleKalmanFilter::getKalmanGain()
{
    return _kalman_gain;
}