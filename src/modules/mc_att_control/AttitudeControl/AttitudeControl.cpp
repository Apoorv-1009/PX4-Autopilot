/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

namespace
{
// Rotation vector taking world-z onto q's thrust direction (zero yaw content).
Vector3f extractTiltState(const Quatf &q)
{
	const Vector3f e_z_d = q.dcm_z();
	const Vector3f e_z_world(0.f, 0.f, 1.f);
	const Vector3f tilt_axis = e_z_world.cross(e_z_d);
	const float tilt_axis_norm = tilt_axis.norm();

	if (tilt_axis_norm > FLT_EPSILON) {
		const float tilt_angle = acosf(math::constrain(e_z_world.dot(e_z_d), -1.f, 1.f));
		return (tilt_axis / tilt_axis_norm) * tilt_angle;
	}

	return Vector3f{};
}
} // namespace

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

void AttitudeControl::setAttitudeSetpoint(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	Quatf qd_normalized = qd;
	qd_normalized.normalize();

	// Tilt-only into the model; yaw is handled by _yawspeed_setpoint in update().
	// The model auto-resets on dt <= 0 or dt > max_step.
	_tilt_filter.update(dt, extractTiltState(qd_normalized));

	_attitude_setpoint_q = qd_normalized;
	_yawspeed_setpoint = yawspeed_setpoint;
}

void AttitudeControl::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
	_attitude_setpoint_q.normalize();
	// Re-extract tilt and reseed the model so any tilt component of the heading
	// reset isn't interpreted as real motion on the next update.
	_tilt_filter.reset(extractTiltState(_attitude_setpoint_q));
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	if (_ff_enabled) {
		// Tilt FF from the reference model (world frame); rotate to body.
		rate_setpoint += q.rotateVectorInverse(_tilt_filter.getRate());

		// Feed forward the yaw setpoint rate.
		// _yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
		// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
		// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
		// and multiply it by the yaw setpoint rate (_yawspeed_setpoint).
		// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
		// such that it can be added to the rates setpoint.
		if (std::isfinite(_yawspeed_setpoint)) {
			rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
		}
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
