// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	ACM_PI.cpp
/// @brief	Generic PI algorithm

#include <math.h>

#include "APM_PI.h"


int32_t APM_PI::get_p(int32_t error)
{
	return (float)error * _kp;
}

int32_t APM_PI::get_i(int32_t error, float dt)
{
	if(dt != 0){
		_integrator += ((float)error * _ki) * dt;

		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
	}
	return _integrator;
}

int32_t APM_PI::get_pi(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt);
}

void
APM_PI::reset_I()
{
	_integrator = 0;
}
