/*
 * Hold-to-run decision logic for a winch driven by three Trigger services
 * (wind / unwind / stop). Pure logic, no ROS: the node feeds it the current
 * input frame and dispatches the service call it asks for.
 *
 * Rules:
 *  - Outside winch mode, or with the deadman released, the desired state is
 *    Stop and the machine is disarmed.
 *  - Arming: after entering winch mode the stick must be seen in the stop
 *    zone (|stick| < threshold) once before a direction is accepted.
 *  - Desired vs commanded: a service is called only when they differ, and
 *    only one call is in flight at a time.
 *  - A direction change goes through Stop first (the driver refuses a
 *    direction while already active).
 *  - A failed direction call is followed by a Stop and requires re-arming
 *    (no automatic retry of a direction).
 *  - A failed Stop is retried every stop_retry_sec.
 *  - An unanswered call is treated as failed after call_timeout_sec.
 */
#pragma once

namespace sbus_serial
{

enum class WinchCmd { Stop, Wind, Unwind };
enum class WinchAction { None, CallStop, CallWind, CallUnwind };

class WinchHoldToRun
{
public:
	struct Config
	{
		double threshold = 90.0;        // |stick| >= threshold counts as full deflection
		bool invert = false;            // false: +stick = unwind, -stick = wind
		double stop_retry_sec = 0.5;    // retry a failed stop after this long
		double call_timeout_sec = 2.0;  // give up on an unanswered call after this long
	};

	WinchHoldToRun() = default;
	explicit WinchHoldToRun(const Config & cfg) : cfg_(cfg) {}

	// One call per input frame (or per timer tick on signal loss).
	// Returns at most one service to call now.
	WinchAction update(bool winch_mode, bool enabled, double stick, double now_sec)
	{
		if (in_flight_)
		{
			if (now_sec - in_flight_since_ < cfg_.call_timeout_sec)
				return WinchAction::None;
			onResponse(false, now_sec);  // unanswered: treat as failed
		}

		const WinchCmd desired = desiredFrom(winch_mode, enabled, stick);
		if (desired == commanded_)
			return WinchAction::None;

		// Either we want to stop, or we want a direction while another one is
		// (or may be) running: both need a Stop first.
		if (desired == WinchCmd::Stop || commanded_ != WinchCmd::Stop)
		{
			if (stop_failed_ && now_sec - last_stop_attempt_ < cfg_.stop_retry_sec)
				return WinchAction::None;
			return begin(WinchCmd::Stop, now_sec);
		}

		return begin(desired, now_sec);
	}

	// Outcome of the action most recently returned by update().
	void onResponse(bool success, double now_sec)
	{
		if (!in_flight_)
			return;
		in_flight_ = false;

		if (success)
		{
			commanded_ = pending_;
			stop_failed_ = false;
			return;
		}

		if (pending_ == WinchCmd::Stop)
		{
			// Driver may still be running: keep commanded_ as is and retry later.
			stop_failed_ = true;
			last_stop_attempt_ = now_sec;
			return;
		}

		// Failed direction: assume the worst (it may have started), force a Stop
		// on the next tick, and make the operator re-centre before trying again.
		commanded_ = pending_;
		armed_ = false;
	}

	WinchCmd commanded() const { return commanded_; }
	bool armed() const { return armed_; }
	bool inFlight() const { return in_flight_; }

private:
	WinchCmd desiredFrom(bool winch_mode, bool enabled, double stick)
	{
		if (!winch_mode || !enabled)
		{
			armed_ = false;
			return WinchCmd::Stop;
		}

		const bool full_pos = stick >= cfg_.threshold;
		const bool full_neg = stick <= -cfg_.threshold;
		if (!full_pos && !full_neg)
		{
			armed_ = true;
			return WinchCmd::Stop;
		}
		if (!armed_)
			return WinchCmd::Stop;

		const bool unwind = cfg_.invert ? full_neg : full_pos;
		return unwind ? WinchCmd::Unwind : WinchCmd::Wind;
	}

	WinchAction begin(WinchCmd cmd, double now_sec)
	{
		pending_ = cmd;
		in_flight_ = true;
		in_flight_since_ = now_sec;
		switch (cmd)
		{
			case WinchCmd::Wind: return WinchAction::CallWind;
			case WinchCmd::Unwind: return WinchAction::CallUnwind;
			default: return WinchAction::CallStop;
		}
	}

	Config cfg_{};
	WinchCmd commanded_ = WinchCmd::Stop;  // driver starts stationary at boot
	WinchCmd pending_ = WinchCmd::Stop;
	bool armed_ = false;
	bool in_flight_ = false;
	double in_flight_since_ = 0.0;
	bool stop_failed_ = false;
	double last_stop_attempt_ = 0.0;
};

}  // namespace sbus_serial
