#include <gtest/gtest.h>

#include "winch_hold_to_run.hpp"

using sbus_serial::WinchAction;
using sbus_serial::WinchCmd;
using sbus_serial::WinchHoldToRun;

namespace
{

// Drive the machine into winch mode, armed, with the given direction already
// acknowledged by the driver. Returns the time of the last tick.
double ArmedAndRunning(WinchHoldToRun & w, double stick, WinchAction expected, double t = 0.0)
{
	EXPECT_EQ(w.update(true, true, 0.0, t), WinchAction::None);  // centre arms
	t += 0.05;
	EXPECT_EQ(w.update(true, true, stick, t), expected);
	w.onResponse(true, t);
	return t;
}

}  // namespace

TEST(WinchHoldToRun, FreshMachineMakesNoBootTimeCall)
{
	WinchHoldToRun w;
	EXPECT_EQ(w.update(false, false, 0.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(false, true, 0.0, 0.05), WinchAction::None);
	EXPECT_EQ(w.commanded(), WinchCmd::Stop);
}

TEST(WinchHoldToRun, IgnoresStickOutsideWinchMode)
{
	WinchHoldToRun w;
	for (int i = 0; i < 5; ++i)
		EXPECT_EQ(w.update(false, true, 100.0, i * 0.05), WinchAction::None);
	EXPECT_EQ(w.commanded(), WinchCmd::Stop);
}

TEST(WinchHoldToRun, RequiresCentreBeforeFirstDirection)
{
	WinchHoldToRun w;
	// Enter winch mode with the stick already held full: nothing may start.
	EXPECT_EQ(w.update(true, true, 100.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 100.0, 0.05), WinchAction::None);
	EXPECT_FALSE(w.armed());
	// Centre once, then full: now it runs.
	EXPECT_EQ(w.update(true, true, 0.0, 0.10), WinchAction::None);
	EXPECT_TRUE(w.armed());
	EXPECT_EQ(w.update(true, true, 100.0, 0.15), WinchAction::CallUnwind);
}

TEST(WinchHoldToRun, FullPositiveStickCallsUnwindExactlyOnce)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, 100.0, WinchAction::CallUnwind);
	EXPECT_EQ(w.commanded(), WinchCmd::Unwind);
	for (int i = 1; i <= 20; ++i)
		EXPECT_EQ(w.update(true, true, 100.0, t + i * 0.05), WinchAction::None);
}

TEST(WinchHoldToRun, FullNegativeStickCallsWind)
{
	WinchHoldToRun w;
	ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	EXPECT_EQ(w.commanded(), WinchCmd::Wind);
}

TEST(WinchHoldToRun, InvertSwapsDirections)
{
	WinchHoldToRun::Config cfg;
	cfg.invert = true;
	WinchHoldToRun w(cfg);
	ArmedAndRunning(w, 100.0, WinchAction::CallWind);
}

TEST(WinchHoldToRun, PartialDeflectionCountsAsStopZone)
{
	WinchHoldToRun w;
	EXPECT_EQ(w.update(true, true, 0.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 89.0, 0.05), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -89.0, 0.10), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 90.0, 0.15), WinchAction::CallUnwind);
}

TEST(WinchHoldToRun, ReturnToCentreCallsStopExactlyOnce)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	EXPECT_EQ(w.update(true, true, 0.0, t + 0.05), WinchAction::CallStop);
	w.onResponse(true, t + 0.05);
	EXPECT_EQ(w.commanded(), WinchCmd::Stop);
	for (int i = 2; i <= 10; ++i)
		EXPECT_EQ(w.update(true, true, 0.0, t + i * 0.05), WinchAction::None);
}

TEST(WinchHoldToRun, DirectionChangeGoesThroughStopFirst)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	// Stick flips straight to the other end: the driver refuses a direction
	// while active, so a stop must go out first.
	EXPECT_EQ(w.update(true, true, 100.0, t + 0.05), WinchAction::CallStop);
	w.onResponse(true, t + 0.05);
	EXPECT_EQ(w.update(true, true, 100.0, t + 0.10), WinchAction::CallUnwind);
	w.onResponse(true, t + 0.10);
	EXPECT_EQ(w.commanded(), WinchCmd::Unwind);
}

TEST(WinchHoldToRun, LeavingWinchModeStopsAndDisarms)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	EXPECT_EQ(w.update(false, true, -100.0, t + 0.05), WinchAction::CallStop);
	w.onResponse(true, t + 0.05);
	EXPECT_FALSE(w.armed());
	// Back into winch mode with the stick still held: must re-arm first.
	EXPECT_EQ(w.update(true, true, -100.0, t + 0.10), WinchAction::None);
}

TEST(WinchHoldToRun, DeadmanReleaseStops)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	EXPECT_EQ(w.update(true, false, -100.0, t + 0.05), WinchAction::CallStop);
}

TEST(WinchHoldToRun, NothingIsSentWhileACallIsInFlight)
{
	WinchHoldToRun w;
	EXPECT_EQ(w.update(true, true, 0.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 0.05), WinchAction::CallWind);
	EXPECT_TRUE(w.inFlight());
	// Operator centres before the driver answered: wait for the answer.
	EXPECT_EQ(w.update(true, true, 0.0, 0.10), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 0.0, 0.15), WinchAction::None);
	w.onResponse(true, 0.15);
	EXPECT_FALSE(w.inFlight());
	EXPECT_EQ(w.update(true, true, 0.0, 0.20), WinchAction::CallStop);
}

TEST(WinchHoldToRun, FailedDirectionIssuesStopAndRequiresRecentre)
{
	WinchHoldToRun w;
	EXPECT_EQ(w.update(true, true, 0.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 0.05), WinchAction::CallWind);
	w.onResponse(false, 0.05);  // e.g. "Winch device not ready" / "already active"
	EXPECT_EQ(w.update(true, true, -100.0, 0.10), WinchAction::CallStop);
	w.onResponse(true, 0.10);
	// Still holding the stick: no automatic retry.
	EXPECT_EQ(w.update(true, true, -100.0, 0.15), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 0.20), WinchAction::None);
	// Re-centre, then try again.
	EXPECT_EQ(w.update(true, true, 0.0, 0.25), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 0.30), WinchAction::CallWind);
}

TEST(WinchHoldToRun, FailedStopIsRetriedAfterRetryPeriod)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind, 1.0);
	EXPECT_EQ(w.update(true, true, 0.0, t + 0.05), WinchAction::CallStop);
	const double fail_t = t + 0.05;
	w.onResponse(false, fail_t);
	EXPECT_EQ(w.commanded(), WinchCmd::Wind);  // driver may still be running
	EXPECT_EQ(w.update(true, true, 0.0, fail_t + 0.10), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 0.0, fail_t + 0.40), WinchAction::None);
	EXPECT_EQ(w.update(true, true, 0.0, fail_t + 0.55), WinchAction::CallStop);
	w.onResponse(true, fail_t + 0.55);
	EXPECT_EQ(w.commanded(), WinchCmd::Stop);
}

TEST(WinchHoldToRun, UnansweredCallTimesOutAsFailure)
{
	WinchHoldToRun w;
	EXPECT_EQ(w.update(true, true, 0.0, 0.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 0.05), WinchAction::CallWind);
	// No response for longer than call_timeout_sec: treated as a failed
	// direction, so the next tick sends a stop.
	EXPECT_EQ(w.update(true, true, -100.0, 1.0), WinchAction::None);
	EXPECT_EQ(w.update(true, true, -100.0, 2.10), WinchAction::CallStop);
	EXPECT_TRUE(w.inFlight());
}

TEST(WinchHoldToRun, SignalLossTickStopsARunningWinch)
{
	WinchHoldToRun w;
	double t = ArmedAndRunning(w, -100.0, WinchAction::CallWind);
	// The timeout timer ticks with no mode and no enable.
	EXPECT_EQ(w.update(false, false, 0.0, t + 0.3), WinchAction::CallStop);
}
