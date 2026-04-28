/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf());
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlConvergenceTest : public ::testing::Test
{
public:
	AttitudeControlConvergenceTest()
	{
		_attitude_control.setProportionalGain(Vector3f(.5f, .6f, .3f), .4f);
		_attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));
	}

	void checkConvergence()
	{
		int i; // need function scope to check how many steps
		Vector3f rate_setpoint(1000.f, 1000.f, 1000.f);

		_attitude_control.setAttitudeSetpoint(_quat_goal, 0.f);

		for (i = 100; i > 0; i--) {
			// run attitude control to get rate setpoints
			const Vector3f rate_setpoint_new = _attitude_control.update(_quat_state);
			// rotate the simulated state quaternion according to the rate setpoint
			_quat_state = _quat_state * Quatf(AxisAnglef(rate_setpoint_new));
			_quat_state = -_quat_state; // produce intermittent antipodal quaternion states to test against unwinding problem

			// expect the error and hence also the output to get smaller with each iteration
			if (rate_setpoint_new.norm() >= rate_setpoint.norm()) {
				break;
			}

			rate_setpoint = rate_setpoint_new;
		}

		EXPECT_EQ(_quat_state.canonical(), _quat_goal.canonical());
		// it shouldn't have taken longer than an iteration timeout to converge
		EXPECT_GT(i, 0);
	}

	AttitudeControl _attitude_control;
	Quatf _quat_state;
	Quatf _quat_goal;
};

TEST_F(AttitudeControlConvergenceTest, AttitudeControlConvergence)
{
	const int inputs = 8;

	const Quatf QArray[inputs] = {
		Quatf(),
		Quatf(0, 1, 0, 0),
		Quatf(0, 0, 1, 0),
		Quatf(0, 0, 0, 1),
		Quatf(0.698f, 0.024f, -0.681f, -0.220f),
		Quatf(-0.820f, -0.313f, 0.225f, -0.423f),
		Quatf(0.599f, -0.172f, 0.755f, -0.204f),
		Quatf(0.216f, -0.662f, 0.290f, -0.656f)
	};

	for (int i = 0; i < inputs; i++) {
		for (int j = 0; j < inputs; j++) {
			printf("--- Input combination: %d %d\n", i, j);
			_quat_state = QArray[i];
			_quat_goal = QArray[j];
			_quat_state.normalize();
			_quat_goal.normalize();
			checkConvergence();
		}
	}
}

TEST(AttitudeControlTest, YawWeightScaling)
{
	// GIVEN: default tuning and pure yaw turn command
	AttitudeControl attitude_control;
	const float yaw_gain = 2.8f;
	const float yaw_sp = .1f;
	Quatf pure_yaw_attitude(cosf(yaw_sp / 2.f), 0, 0, sinf(yaw_sp / 2.f));
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), .4f);
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(pure_yaw_attitude, 0.f);

	// WHEN: we run one iteration of the controller
	Vector3f rate_setpoint = attitude_control.update(Quatf());

	// THEN: no actuation in roll, pitch
	EXPECT_EQ(Vector2f(rate_setpoint), Vector2f());
	// THEN: actuation error * gain in yaw
	EXPECT_NEAR(rate_setpoint(2), yaw_sp * yaw_gain, 1e-4f);

	// GIVEN: additional corner case of zero yaw weight
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), 0.f);
	// WHEN: we run one iteration of the controller
	rate_setpoint = attitude_control.update(Quatf());
	// THEN: no actuation (also no NAN)
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlFeedforwardTest : public ::testing::Test
{
public:
	AttitudeControlFeedforwardTest()
	{
		_attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), 0.4f);
		_attitude_control.setRateLimit(Vector3f(10.f, 10.f, 10.f));
	}

	// Push a constant-rate ramp around the given body axis until the reference model is settled.
	// First call uses dt < 0 to reset the model to the current sample (matches the wrapper's
	// behaviour on the very first setpoint after boot).
	Quatf rampSetpoint(const Vector3f &body_rate, float yawspeed_sp, int steps)
	{
		Quatf q_d;

		for (int i = 0; i < steps; i++) {
			q_d = q_d * Quatf(AxisAnglef(body_rate * kDt));
			_attitude_control.setAttitudeSetpoint(q_d, yawspeed_sp, (i == 0) ? -1.f : kDt);
		}

		return q_d;
	}

	AttitudeControl _attitude_control;

	static constexpr float kDt = 0.004f;                                    // 250 Hz setpoint rate
	static constexpr float kOmegaN = AttitudeControl::kTiltFFNaturalFreq;   // 10 rad/s
	static constexpr int kSettleSteps = 200;                                // ~ 8 / omega_n, well-settled
};

TEST_F(AttitudeControlFeedforwardTest, ConstantSetpointGivesNoFeedforward)
{
	// GIVEN: a constant tilted setpoint repeated with valid dt
	const Quatf q_d(AxisAnglef(Vector3f(0.1f, 0.f, 0.f)));

	for (int i = 0; i < kSettleSteps; i++) {
		_attitude_control.setAttitudeSetpoint(q_d, 0.f, (i == 0) ? -1.f : kDt);
	}

	// WHEN: vehicle is at the setpoint (no error)
	const Vector3f rate_setpoint = _attitude_control.update(q_d);

	// THEN: rate setpoint is zero — non-moving SP gives a zero model rate output
	EXPECT_NEAR(rate_setpoint.norm(), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, RollRampProducesRollFeedforward)
{
	// GIVEN: a steady roll ramp, vehicle perfectly tracking
	const float omega = 0.5f;
	const Quatf q_d = rampSetpoint(Vector3f(omega, 0.f, 0.f), 0.f, kSettleSteps);

	// WHEN: vehicle is at the setpoint (no error → P term = 0)
	const Vector3f rate_setpoint = _attitude_control.update(q_d);

	// THEN: the entire rate setpoint comes from the tilt FF and lies on the roll axis
	EXPECT_NEAR(rate_setpoint(0), omega, 0.05f * omega);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, PitchRampProducesPitchFeedforward)
{
	// GIVEN: a steady pitch ramp, vehicle perfectly tracking
	const float omega = 0.5f;
	const Quatf q_d = rampSetpoint(Vector3f(0.f, omega, 0.f), 0.f, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(q_d);

	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), omega, 0.05f * omega);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, YawRampOnlyAnalyticalFeedforwardContributes)
{
	// GIVEN: a yaw ramp with the analytical yawspeed FF set to the same rate.
	// Yaw rotation does not change the desired thrust direction, so the tilt
	// reference model sees a constant input and produces zero output. The yaw FF
	// thus comes solely from the analytical yaw_sp_move_rate path — no projection
	// or double-counting needed because yaw never enters the model.
	const float omega = 0.5f;
	const Quatf q_d = rampSetpoint(Vector3f(0.f, 0.f, omega), omega, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(q_d);

	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), omega, 0.05f * omega);
}

TEST_F(AttitudeControlFeedforwardTest, FeedForwardDisabledSuppressesContribution)
{
	// GIVEN: a settled roll-ramp FF
	const float omega = 0.5f;
	const Quatf q_d = rampSetpoint(Vector3f(omega, 0.f, 0.f), 0.f, kSettleSteps);

	// WHEN: FF is disabled (e.g. autotune active)
	_attitude_control.setFeedForwardEnabled(false);
	Vector3f rate_setpoint = _attitude_control.update(q_d);

	// THEN: no FF applied even though the model still holds the ramp rate
	EXPECT_NEAR(rate_setpoint.norm(), 0.f, 1e-3f);

	// AND WHEN: re-enabled, the FF returns immediately (model state preserved)
	_attitude_control.setFeedForwardEnabled(true);
	rate_setpoint = _attitude_control.update(q_d);

	EXPECT_NEAR(rate_setpoint(0), omega, 0.05f * omega);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}
