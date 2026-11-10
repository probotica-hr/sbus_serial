/*
 * Copyright 2018-2024 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#pragma once

/* SBUS -> commands_node
 *
 * Subscribes to:
 *	/sbus (Sbus)
 * Publishes:
 *	/output/sbus/cmd_vel (Twist or TwistStamped) - remap as necessary
 *  /joy_enable (Bool)
 *  /joy_commands (SbusCommands) - only when joystick enabled
 * Parameters:
 *  forwardChannelIndx (int): Channel index for forward/reverse. Default is 1
 *  turnChannelIndx (int): Channel index for turning. Default is 0
 *  lateralChannelIndx (int): Channel index for lateral (crab) movement. Default is -1 (disabled)
 *  modeChannelIndex (int): Channel index for 3-position mode switch. Default is -1 (disabled)
 *  modeChannelThresholdLow (double): Below this = mode 0. Default is -50.0
 *  modeChannelThresholdHigh (double): Above this = mode 2, between = mode 1. Default is 50.0
 *  winchModeIndex (int): Mode value (0/1/2) that is "winch mode". Default is -1 (disabled).
 *      In winch mode the twist is forced to zero and the winch stick drives the
 *      winch hold-to-run via three std_srvs/Trigger services (see winch_hold_to_run.hpp).
 *  winchChannelIndx (int): Channel index read as the winch stick. Default is 1 (forward stick)
 *  winchChannelThreshold (double): |value| at or above this = full deflection. Default is 90.0
 *  winchInvert (bool): false: +stick = unwind (pay out), -stick = wind (pull in). Default is false
 *  winchWindService / winchUnwindService / winchStopService (string): Trigger service names.
 *      Defaults: /winch/winch/wind, /winch/winch/unwind, /winch/winch/stop
 *  winchStopRetryPeriod (double): retry a failed stop after this many seconds. Default is 0.5
 *  winchServiceTimeout (double): treat an unanswered call as failed after this long. Default is 2.0
 *  winchMock (bool): dry run. No service is ever called; instead the name of the service
 *      that WOULD have been called is published on "winch_mock" (std_msgs/String,
 *      transient_local) once per change. Default is false
 */

#include "rclcpp/rclcpp.hpp"
#include "sbus_serial/msg/sbus.hpp"
#include "sbus_serial/msg/sbus_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "winch_hold_to_run.hpp"

#define FORWARD_CHANNEL_INDX 1 // Channel 2 (elevator)
#define TURN_CHANNEL_INDX 0	   // Channel 1 (ailerons)

class SbusCommands : public rclcpp::Node
{
public:
	explicit SbusCommands(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
	: Node("sbus_commands", options)
	{
		this->declare_parameter("forwardChannelIndx", FORWARD_CHANNEL_INDX);
		this->declare_parameter("turnChannelIndx", TURN_CHANNEL_INDX);
		this->declare_parameter("lateralChannelIndx", -1);
		this->declare_parameter("sbusMinValue", 0);
		this->declare_parameter("sbusMaxValue", 255);
		this->declare_parameter("minSpeed", -1.0);
		this->declare_parameter("maxSpeed", 1.0);
		this->declare_parameter("minTurn", -1.0);
		this->declare_parameter("maxTurn", 1.0);
		this->declare_parameter("minTurboSpeed", -1.0);
		this->declare_parameter("maxTurboSpeed", 1.0);
		this->declare_parameter("minLateral", -1.0);
		this->declare_parameter("maxLateral", 1.0);
		this->declare_parameter("minTurboTurn", -1.0);
		this->declare_parameter("maxTurboTurn", 1.0);
		this->declare_parameter("minTurboLateral", -1.0);
		this->declare_parameter("maxTurboLateral", 1.0);
		this->declare_parameter("useStamped", true);
		this->declare_parameter("frameId", "");
		this->declare_parameter("deadband", 0.0);
		this->declare_parameter("turboChannelIndx", -1);
		this->declare_parameter("turboChannelValue", 100.0);
		this->declare_parameter("enableUtilCommands", false);
		this->declare_parameter("enableControlChannelIndex", -1);
		this->declare_parameter("enableControlChannelValue", 100.0);
		this->declare_parameter("reverseTurn", false);
		this->declare_parameter("modeChannelIndex", -1);
		this->declare_parameter("modeChannelThresholdLow", -50.0);
		this->declare_parameter("modeChannelThresholdHigh", 50.0);
		this->declare_parameter("batteryChannelIndex", -1);
		this->declare_parameter("batteryChannelValue", 100.0);
		this->declare_parameter("batteryChannelToggle", false);
		this->declare_parameter("lightChannelIndex", -1);
		this->declare_parameter("lightChannelValue", 100.0);
		this->declare_parameter("lightChannelToggle", false);
		this->declare_parameter("hornChannelIndex", -1);
		this->declare_parameter("hornChannelValue", 100.0);
		this->declare_parameter("hornChannelToggle", true);
		this->declare_parameter("timeoutThreshold", 0.25);
		this->declare_parameter("refresh_rate_hz", 20);
		this->declare_parameter("winchModeIndex", -1);
		this->declare_parameter("winchChannelIndx", FORWARD_CHANNEL_INDX);
		this->declare_parameter("winchChannelThreshold", 90.0);
		this->declare_parameter("winchInvert", false);
		this->declare_parameter("winchWindService", "/winch/winch/wind");
		this->declare_parameter("winchUnwindService", "/winch/winch/unwind");
		this->declare_parameter("winchStopService", "/winch/winch/stop");
		this->declare_parameter("winchStopRetryPeriod", 0.5);
		this->declare_parameter("winchServiceTimeout", 2.0);
		this->declare_parameter("winchMock", false);

		this->get_parameter("forwardChannelIndx", forwardChannelIndx_);
		this->get_parameter("turnChannelIndx", turnChannelIndx_);
		this->get_parameter("lateralChannelIndx", lateralChannelIndx_);
		this->get_parameter("sbusMinValue", sbusMinValue_);
		this->get_parameter("sbusMaxValue", sbusMaxValue_);
		this->get_parameter("minSpeed", minSpeed_);
		this->get_parameter("maxSpeed", maxSpeed_);
		this->get_parameter("minTurn", minTurn_);
		this->get_parameter("maxTurn", maxTurn_);
		this->get_parameter("minTurboSpeed", minTurboSpeed_);
		this->get_parameter("maxTurboSpeed", maxTurboSpeed_);
		this->get_parameter("minLateral", minLateral_);
		this->get_parameter("maxLateral", maxLateral_);
		this->get_parameter("minTurboTurn", minTurboTurn_);
		this->get_parameter("maxTurboTurn", maxTurboTurn_);
		this->get_parameter("minTurboLateral", minTurboLateral_);
		this->get_parameter("maxTurboLateral", maxTurboLateral_);
		this->get_parameter("useStamped", useStamped_);
		this->get_parameter("frameId", frameId_);
		this->get_parameter("deadband", deadband_);
		this->get_parameter("turboChannelIndx", turboChannelIndx_);
		this->get_parameter("turboChannelValue", turboChannelValue_);
		this->get_parameter("enableUtilCommands", enableUtilCommands_);
		this->get_parameter("enableControlChannelIndex", enableControlChannelIndex_);
		this->get_parameter("enableControlChannelValue", enableControlChannelValue_);
		this->get_parameter("reverseTurn", reverseTurn_);
		this->get_parameter("modeChannelIndex", modeChannelIndex_);
		this->get_parameter("modeChannelThresholdLow", modeChannelThresholdLow_);
		this->get_parameter("modeChannelThresholdHigh", modeChannelThresholdHigh_);
		this->get_parameter("batteryChannelIndex", batteryChannelIndex_);
		this->get_parameter("batteryChannelValue", batteryChannelValue_);
		this->get_parameter("batteryChannelToggle", batteryChannelToggle_);
		this->get_parameter("lightChannelIndex", lightChannelIndex_);
		this->get_parameter("lightChannelValue", lightChannelValue_);
		this->get_parameter("lightChannelToggle", lightChannelToggle_);
		this->get_parameter("hornChannelIndex", hornChannelIndex_);
		this->get_parameter("hornChannelValue", hornChannelValue_);
		this->get_parameter("hornChannelToggle", hornChannelToggle_);
		this->get_parameter("timeoutThreshold", timeoutThreshold_);
		this->get_parameter("refresh_rate_hz", refreshRate_);
		this->get_parameter("winchModeIndex", winchModeIndex_);
		this->get_parameter("winchChannelIndx", winchChannelIndx_);
		sbus_serial::WinchHoldToRun::Config winchCfg;
		this->get_parameter("winchChannelThreshold", winchCfg.threshold);
		this->get_parameter("winchInvert", winchCfg.invert);
		this->get_parameter("winchStopRetryPeriod", winchCfg.stop_retry_sec);
		this->get_parameter("winchServiceTimeout", winchCfg.call_timeout_sec);
		this->get_parameter("winchMock", winchMock_);
		winch_ = sbus_serial::WinchHoldToRun(winchCfg);

		sbusRange_ = sbusMaxValue_ - sbusMinValue_;

		isFirstRead_ = true;
		timeout_threshold_ = rclcpp::Duration::from_seconds(timeoutThreshold_);

		sbus_sub_ = this->create_subscription<sbus_serial::msg::Sbus>("sbus", 1, std::bind(&SbusCommands::sbusCallback, this, std::placeholders::_1));

		if (useStamped_)
			cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("output/sbus/cmd_vel", 10);
		else
			cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("output/sbus/cmd_vel", 10);

		enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("joy_enable", 10);
		commands_pub_ = this->create_publisher<sbus_serial::msg::SbusCommands>("joy_commands", 10);

		if (winchModeIndex_ >= 0)
		{
			this->get_parameter("winchWindService", winchWindSrv_);
			this->get_parameter("winchUnwindService", winchUnwindSrv_);
			this->get_parameter("winchStopService", winchStopSrv_);
			if (winchMock_)
			{
				// transient_local so a late `ros2 topic echo` still sees the last commands
				winch_mock_pub_ = this->create_publisher<std_msgs::msg::String>(
					"winch_mock", rclcpp::QoS(10).transient_local());
			}
			else
			{
				winch_wind_client_ = this->create_client<std_srvs::srv::Trigger>(winchWindSrv_);
				winch_unwind_client_ = this->create_client<std_srvs::srv::Trigger>(winchUnwindSrv_);
				winch_stop_client_ = this->create_client<std_srvs::srv::Trigger>(winchStopSrv_);
			}
			RCLCPP_INFO(this->get_logger(), "Winch mode%s = mode %d on channel %d (threshold %.0f%s): %s / %s / %s",
				winchMock_ ? " (MOCK, no service is called, see topic winch_mock)" : "",
				winchModeIndex_, winchChannelIndx_, winchCfg.threshold, winchCfg.invert ? ", inverted" : "",
				winchWindSrv_.c_str(), winchUnwindSrv_.c_str(), winchStopSrv_.c_str());
		}

		if (enableControlChannelIndex_ < 0)
		{
			RCLCPP_WARN(this->get_logger(), "enableControlChannelIndex is %d — joystick enable will never be true. Set to a valid channel (0-8).", enableControlChannelIndex_);
		}

		RCLCPP_INFO(this->get_logger(), "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s", this->get_name(), sbusMinValue_, sbusMaxValue_, maxSpeed_, maxTurn_);

		last_msg_time_ = this->now();

		sbus_timer_ = this->create_wall_timer(
				std::chrono::milliseconds(1000 / refreshRate_),
				std::bind(&SbusCommands::timerCallback, this));
	}

private:
	rclcpp::Subscription<sbus_serial::msg::Sbus>::SharedPtr sbus_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
	rclcpp::Publisher<sbus_serial::msg::SbusCommands>::SharedPtr commands_pub_;
	rclcpp::Time last_msg_time_;
	rclcpp::TimerBase::SharedPtr sbus_timer_;
	rclcpp::Duration timeout_threshold_{rclcpp::Duration::from_seconds(0.0)};

	int sbusMinValue_;
	int sbusMaxValue_;
	int sbusRange_;
	double minSpeed_;
	double maxSpeed_;
	double minTurboSpeed_;
	double maxTurboSpeed_;
	double minTurn_;
	double maxTurn_;
	double minLateral_;
	double maxLateral_;
	double minTurboTurn_;
	double maxTurboTurn_;
	double minTurboLateral_;
	double maxTurboLateral_;
	double deadband_;
	int forwardChannelIndx_;
	int turnChannelIndx_;
	int lateralChannelIndx_;
	bool useStamped_;
	std::string frameId_;
	int turboChannelIndx_;
	double turboChannelValue_;
	int enableControlChannelIndex_;
	double enableControlChannelValue_;
	bool reverseTurn_;
	bool enableUtilCommands_;
	int modeChannelIndex_;
	double modeChannelThresholdLow_;
	double modeChannelThresholdHigh_;
	int batteryChannelIndex_;
	double batteryChannelValue_;
	bool batteryChannelToggle_;
	int lightChannelIndex_;
	double lightChannelValue_;
	bool lightChannelToggle_;
	int hornChannelIndex_;
	double hornChannelValue_;
	bool hornChannelToggle_;
	double currentBatteryState_;
	double currentLightState_;
	double currentHornState_;
	double previousHornState_;
	double previousBatteryState_;
	double previousLightState_;
	bool isFirstRead_;
	double timeoutThreshold_;
	int refreshRate_;

	int winchModeIndex_;
	int winchChannelIndx_;
	bool winchMock_;
	std::string winchWindSrv_, winchUnwindSrv_, winchStopSrv_;
	sbus_serial::WinchHoldToRun winch_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr winch_mock_pub_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr winch_wind_client_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr winch_unwind_client_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr winch_stop_client_;

	// -1 when no mode channel is configured, else 0/1/2 from the thresholds.
	int computeMode(const sbus_serial::msg::Sbus::SharedPtr & msg) const
	{
		if (modeChannelIndex_ == -1)
			return -1;
		const double modeValue = msg->mapped_channels[modeChannelIndex_];
		if (modeValue < modeChannelThresholdLow_)
			return 0;
		if (modeValue > modeChannelThresholdHigh_)
			return 2;
		return 1;
	}

	// Feed one input frame to the winch state machine and dispatch at most one
	// Trigger call. Single-threaded executor: all callbacks run on one thread.
	void tickWinch(bool winchMode, bool enabled, double stick)
	{
		if (winchModeIndex_ < 0)
			return;

		const double now = this->now().seconds();
		const auto action = winch_.update(winchMode, enabled, stick, now);
		if (action == sbus_serial::WinchAction::None)
			return;

		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
		const char * name = "";
		const std::string * service = nullptr;
		switch (action)
		{
			case sbus_serial::WinchAction::CallWind: client = winch_wind_client_; name = "wind"; service = &winchWindSrv_; break;
			case sbus_serial::WinchAction::CallUnwind: client = winch_unwind_client_; name = "unwind"; service = &winchUnwindSrv_; break;
			default: client = winch_stop_client_; name = "stop"; service = &winchStopSrv_; break;
		}

		if (winchMock_)
		{
			RCLCPP_INFO(this->get_logger(), "Winch (mock): %s -> would call %s", name, service->c_str());
			std_msgs::msg::String m;
			m.data = *service;
			winch_mock_pub_->publish(m);
			winch_.onResponse(true, now);  // pretend the driver acknowledged
			return;
		}

		if (!client->service_is_ready())
		{
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"Winch %s: service %s not available", name, client->get_service_name());
			winch_.onResponse(false, now);
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Winch: %s", name);
		auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
		client->async_send_request(request,
			[this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
			{
				const auto response = future.get();
				if (!response->success)
					RCLCPP_WARN(this->get_logger(), "Winch %s failed: %s", name, response->message.c_str());
				winch_.onResponse(response->success, this->now().seconds());
			});
	}

	void sbusCallback(const sbus_serial::msg::Sbus::SharedPtr msg)
	{
		double proportional;
		last_msg_time_ = this->now();

		bool turboMode = false;

		if (turboChannelIndx_ != -1)
		{
			turboMode = msg->mapped_channels[turboChannelIndx_] == turboChannelValue_;
		}

		double minSpeedFinal = turboMode ? minTurboSpeed_ : minSpeed_;
		double maxSpeedFinal = turboMode ? maxTurboSpeed_ : maxSpeed_;
		double minTurnFinal = turboMode ? minTurboTurn_ : minTurn_;
		double maxTurnFinal = turboMode ? maxTurboTurn_ : maxTurn_;
		double minLateralFinal = turboMode ? minTurboLateral_ : minLateral_;
		double maxLateralFinal = turboMode ? maxTurboLateral_ : maxLateral_;

		proportional = static_cast<double>(msg->mapped_channels[forwardChannelIndx_] - sbusMinValue_) / sbusRange_;
		double fwdSpeed = minSpeedFinal + (maxSpeedFinal - minSpeedFinal) * proportional;

		proportional = static_cast<double>(msg->mapped_channels[turnChannelIndx_] - sbusMinValue_) / sbusRange_;
		double turn = -(minTurnFinal + (maxTurnFinal - minTurnFinal) * proportional);

		double lateral = 0.0;
		if (lateralChannelIndx_ != -1)
		{
			proportional = static_cast<double>(msg->mapped_channels[lateralChannelIndx_] - sbusMinValue_) / sbusRange_;
			lateral = minLateralFinal + (maxLateralFinal - minLateralFinal) * proportional;
			if (std::abs(lateral) < deadband_)
				lateral = 0.0;
		}

		if (reverseTurn_ && fwdSpeed < 0)
			turn = -turn;

		if (std::abs(fwdSpeed) < deadband_)
			fwdSpeed = 0.0;
		if (std::abs(turn) < deadband_)
			turn = 0.0;

		bool joystickEnabled = (enableControlChannelIndex_ >= 0) &&
			(msg->mapped_channels[enableControlChannelIndex_] == enableControlChannelValue_);

		const int mode = computeMode(msg);
		const bool winchMode = (winchModeIndex_ >= 0) && (mode == winchModeIndex_);

		geometry_msgs::msg::Twist twist;
		if (joystickEnabled && !winchMode)
		{
			twist.linear.x = fwdSpeed;
			twist.linear.y = lateral;
			twist.angular.z = turn;
		}
		else
		{
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.angular.z = 0.0;
		}

		if (useStamped_)
		{
			geometry_msgs::msg::TwistStamped twistStamped;
			twistStamped.twist = twist;
			twistStamped.header.stamp = rclcpp::Time(msg->header.stamp);
			twistStamped.header.frame_id = frameId_;
			cmd_vel_stamped_pub_->publish(twistStamped);
		}
		else
		{
			cmd_vel_pub_->publish(twist);
		}

		std_msgs::msg::Bool enableControl;
		enableControl.data = joystickEnabled;
		enable_pub_->publish(enableControl);

		tickWinch(winchMode, joystickEnabled,
			static_cast<double>(msg->mapped_channels[winchChannelIndx_]));

		if (!joystickEnabled)
			return;

		if (!enableUtilCommands_ && modeChannelIndex_ == -1)
			return;

		sbus_serial::msg::SbusCommands sbusCommands;
		sbusCommands.mode = 0;
		sbusCommands.battery_charge = false;
		sbusCommands.lights = false;
		sbusCommands.horn = false;

		if (mode >= 0)
			sbusCommands.mode = mode;

		if (enableUtilCommands_)
		{
			if (lightChannelIndex_ < 0 || hornChannelIndex_ < 0 || batteryChannelIndex_ < 0)
			{
				throw rclcpp::exceptions::InvalidParameterValueException("Channel index value must be set to a positive number or is out of range (range: 0-8)");
				return;
			}
			if ((lightChannelToggle_ && lightChannelIndex_ < 6) || (hornChannelToggle_ && hornChannelIndex_ < 6) || (batteryChannelToggle_ && batteryChannelIndex_ < 6))
			{
				throw rclcpp::exceptions::InvalidParameterValueException("Selected channels (0-5) cannot be set to toggle. Change ChannelIndex or ChannelToggle parameters.");
				return;
			}
			if (isFirstRead_)
			{
				if (batteryChannelToggle_)
					previousBatteryState_ = msg->mapped_channels[batteryChannelIndex_];
				if (lightChannelToggle_)
					previousLightState_ = msg->mapped_channels[lightChannelIndex_];
				if (hornChannelToggle_)
					previousHornState_ = msg->mapped_channels[hornChannelIndex_];
				isFirstRead_ = false;
			}

			if (batteryChannelToggle_)
			{
				if (previousBatteryState_ != msg->mapped_channels[batteryChannelIndex_])
				{
					sbusCommands.battery_charge = true;
					previousBatteryState_ = msg->mapped_channels[batteryChannelIndex_];
				}
			}
			else
			{
				if (msg->mapped_channels[batteryChannelIndex_] == batteryChannelValue_)
					sbusCommands.battery_charge = true;
			}

			if (lightChannelToggle_)
			{
				if (previousLightState_ != msg->mapped_channels[lightChannelIndex_])
				{
					sbusCommands.lights = true;
					previousLightState_ = msg->mapped_channels[lightChannelIndex_];
				}
			}
			else
			{
				if (msg->mapped_channels[lightChannelIndex_] == lightChannelValue_)
					sbusCommands.lights = true;
			}

			if (hornChannelToggle_)
			{
				if (previousHornState_ != msg->mapped_channels[hornChannelIndex_])
				{
					sbusCommands.horn = true;
					previousHornState_ = msg->mapped_channels[hornChannelIndex_];
				}
			}
			else
			{
				if (msg->mapped_channels[hornChannelIndex_] == hornChannelValue_)
					sbusCommands.horn = true;
			}
		}

		commands_pub_->publish(sbusCommands);
	}

	void timerCallback()
	{
		rclcpp::Time now = this->now();

		if ((now - last_msg_time_) > timeout_threshold_)
		{
			if (useStamped_)
			{
				geometry_msgs::msg::TwistStamped twist;
				twist.header.stamp = now;
				twist.header.frame_id = frameId_;
				twist.twist.linear.x = 0.0;
				twist.twist.linear.y = 0.0;
				twist.twist.angular.z = 0.0;
				cmd_vel_stamped_pub_->publish(twist);
			}
			else
			{
				geometry_msgs::msg::Twist twist;
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				twist.angular.z = 0.0;
				cmd_vel_pub_->publish(twist);
			}

			std_msgs::msg::Bool enableControl;
			enableControl.data = false;
			enable_pub_->publish(enableControl);

			tickWinch(false, false, 0.0);
		}
	}
};
