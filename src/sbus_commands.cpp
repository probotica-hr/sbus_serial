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
 */

#include "rclcpp/rclcpp.hpp"
#include "sbus_serial/msg/sbus.hpp"
#include "sbus_serial/msg/sbus_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#define FORWARD_CHANNEL_INDX 1 // Channel 2 (elevator)
#define TURN_CHANNEL_INDX 0	   // Channel 1 (ailerons)

class SbusCommands : public rclcpp::Node
{
public:
	SbusCommands() : Node("sbus_commands")
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

		geometry_msgs::msg::Twist twist;
		if (joystickEnabled)
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

		if (!joystickEnabled)
			return;

		if (!enableUtilCommands_ && modeChannelIndex_ == -1)
			return;

		sbus_serial::msg::SbusCommands sbusCommands;
		sbusCommands.mode = 0;
		sbusCommands.battery_charge = false;
		sbusCommands.lights = false;
		sbusCommands.horn = false;

		if (modeChannelIndex_ != -1)
		{
			double modeValue = msg->mapped_channels[modeChannelIndex_];
			if (modeValue < modeChannelThresholdLow_)
				sbusCommands.mode = 0;
			else if (modeValue > modeChannelThresholdHigh_)
				sbusCommands.mode = 2;
			else
				sbusCommands.mode = 1;
		}

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
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SbusCommands>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
