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
 *	/output/sbus/cmd_vel (Twist) - remap as necessary
 *  /joystick_enable (Bool)
 *  /joystick_commands (SbusCommands)
 * Parameters:
 *  forwardChannelIndx (int): Channel index (not channel number!) for forward/reverse. Default is 1 (channel 2)
 *  turnChannelIndx (int): Channel index for turning. Default is 0 (channel 1)
 *  sbusMinValue (int): Minimum value for SBUS channels. Default is 0
 *  sbusMaxValue (int): Maximum value for SBUS channels. Default is 255
 *  minSpeed (double): Minimum speed in m/sec for output Twist. Default is -1.0
 *  maxSpeed (double): Maximum speed in m/sec for output Twist. Default is 1.0
 *  minTurn (double): Minimum turn rate in radians/sec for output Twist. Default is -1.0
 *  maxTurn (double): Maximum turn rate in radians/sec for output Twist. Default is 1.0
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
		// Read/set parameters
		this->declare_parameter("forwardChannelIndx", FORWARD_CHANNEL_INDX); // Channel index (not channel number!) for forward/reverse
		this->declare_parameter("turnChannelIndx", TURN_CHANNEL_INDX);		 // Channel index for turning
		this->declare_parameter("sbusMinValue", 0);							 // Minimum value for SBUS channels
		this->declare_parameter("sbusMaxValue", 255);						 // Maximum value for SBUS channels
		this->declare_parameter("minSpeed", -1.0);							 // Minimum speed in m/sec for output Twist
		this->declare_parameter("maxSpeed", 1.0);							 // Maximum speed in m/sec for output Twist
		this->declare_parameter("minTurn", -1.0);							 // Minimum turn rate in radians/sec for output Twist
		this->declare_parameter("maxTurn", 1.0);
		this->declare_parameter("minTurboSpeed", -1.0); // Minimum speed in m/sec for output Twist
		this->declare_parameter("maxTurboSpeed", 1.0);	// Maximum speed in m/sec for output Twist
		this->declare_parameter("minTurboTurn", -1.0);	// Minimum turn rate in radians/sec for output Twist
		this->declare_parameter("maxTurboTurn", 1.0);
		this->declare_parameter("useStamped", true); // Maximum turn rate in radians/sec for output Twist
		this->declare_parameter("frameId", "");		 // frame_id for TwistStamped message
		this->declare_parameter("deadband", 0.0);	 // Deadband is in received sbus values and is both plus and minus around the midpoint and is used for all channels
		this->declare_parameter("turboChannelIndx", -1);
		this->declare_parameter("turboChannelValue", 100.0);
		this->declare_parameter("disableCmdVelChannelIndx", -1);
		this->declare_parameter("disableCmdVelChannelValue", 100.0);
		this->declare_parameter("enableUtilCommands", false);
		this->declare_parameter("enableControlChannelIndex", -1);
		this->declare_parameter("enableControlChannelValue", 100.0);
		this->declare_parameter("batteryChannelIndex", -1);		// Channel index (not channel number!) for battery checking
		this->declare_parameter("batteryChannelValue", 100.0);	// Channel value required for changing states
		this->declare_parameter("batteryChannelToggle", false); // If set to true, publish only one message upon changing button state
		this->declare_parameter("lightChannelIndex", -1);
		this->declare_parameter("lightChannelValue", 100.0);
		this->declare_parameter("lightChannelToggle", false);
		this->declare_parameter("hornChannelIndex", -1);
		this->declare_parameter("hornChannelValue", 100.0);
		this->declare_parameter("hornChannelToggle", true);

		this->get_parameter("forwardChannelIndx", forwardChannelIndx_);
		this->get_parameter("turnChannelIndx", turnChannelIndx_);
		this->get_parameter("sbusMinValue", sbusMinValue_);
		this->get_parameter("sbusMaxValue", sbusMaxValue_);
		this->get_parameter("minSpeed", minSpeed_);
		this->get_parameter("maxSpeed", maxSpeed_);
		this->get_parameter("minTurn", minTurn_);
		this->get_parameter("maxTurn", maxTurn_);
		this->get_parameter("minTurboSpeed", minTurboSpeed_);
		this->get_parameter("maxTurboSpeed", maxTurboSpeed_);
		this->get_parameter("minTurboTurn", minTurboTurn_);
		this->get_parameter("maxTurboTurn", maxTurboTurn_);
		this->get_parameter("useStamped", useStamped_);
		this->get_parameter("frameId", frameId_);
		this->get_parameter("deadband", deadband_);
		this->get_parameter("turboChannelIndx", turboChannelIndx_);
		this->get_parameter("turboChannelValue", turboChannelValue_);
		this->get_parameter("disableCmdVelChannelIndx", disableCmdVelChannelIndx_);
		this->get_parameter("disableCmdVelChannelValue", disableCmdVelChannelValue_);
		this->get_parameter("enableUtilCommands", enableUtilCommands_);
		this->get_parameter("enableControlChannelIndex", enableControlChannelIndex_);
		this->get_parameter("enableControlChannelValue", enableControlChannelValue_);
		this->get_parameter("batteryChannelIndex", batteryChannelIndex_);
		this->get_parameter("batteryChannelValue", batteryChannelValue_);
		this->get_parameter("batteryChannelToggle", batteryChannelToggle_);
		this->get_parameter("lightChannelIndex", lightChannelIndex_);
		this->get_parameter("lightChannelValue", lightChannelValue_);
		this->get_parameter("lightChannelToggle", lightChannelToggle_);
		this->get_parameter("hornChannelIndex", hornChannelIndex_);
		this->get_parameter("hornChannelValue", hornChannelValue_);
		this->get_parameter("hornChannelToggle", hornChannelToggle_);

		sbusRange_ = sbusMaxValue_ - sbusMinValue_; // Calculate range once

		isFirstRead_ = true;

		sbus_sub_ = this->create_subscription<sbus_serial::msg::Sbus>("/sbus", 1, std::bind(&SbusCommands::sbusCallback, this, std::placeholders::_1));

		// Only create publisher for stamped/unstamped Twist messages
		if (useStamped_)
			cmd_vel_stamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/output/sbus/cmd_vel", 10);
		else
			cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/output/sbus/cmd_vel", 10);

		enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/joy_enable", 10);
		commands_pub_ = this->create_publisher<sbus_serial::msg::SbusCommands>("/joy_commands", 10);

		RCLCPP_INFO(this->get_logger(), "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s", this->get_name(), sbusMinValue_, sbusMaxValue_, maxSpeed_, maxTurn_);
	}

private:
	rclcpp::Subscription<sbus_serial::msg::Sbus>::SharedPtr sbus_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
	rclcpp::Publisher<sbus_serial::msg::SbusCommands>::SharedPtr commands_pub_;

	int sbusMinValue_;
	int sbusMaxValue_;
	int sbusRange_; // Calculated when reading min/max
	double minSpeed_;
	double maxSpeed_; // m/sec
	double minTurboSpeed_;
	double maxTurboSpeed_;
	double minTurn_;
	double maxTurn_; // radians/sec
	double minTurboTurn_;
	double maxTurboTurn_;
	double deadband_;
	int forwardChannelIndx_;
	int turnChannelIndx_;
	bool useStamped_;	  // If true, use TwistStamped message with timestamp
	std::string frameId_; // Only used if useStamped_ is true
	int turboChannelIndx_;
	double turboChannelValue_;
	int disableCmdVelChannelIndx_;
	double disableCmdVelChannelValue_;
	int enableControlChannelIndex_;
	double enableControlChannelValue_;
	bool enableUtilCommands_;
	int batteryChannelIndex_;
	float batteryChannelValue_;
	bool batteryChannelToggle_;
	int lightChannelIndex_;
	float lightChannelValue_;
	bool lightChannelToggle_;
	int hornChannelIndex_;
	float hornChannelValue_;
	bool hornChannelToggle_;
	double currentBatteryState_;
	double currentLightState_;
	double currentHornState_;
	double previousHornState_;
	double previousBatteryState_;
	double previousLightState_;
	bool isFirstRead_;

	void sbusCallback(const sbus_serial::msg::Sbus::SharedPtr msg)
	{
		double proportional;
		double minSpeedFinal;
		double maxSpeedFinal;
		double minTurnFinal;
		double maxTurnFinal;

		bool turboMode = false;

		if (disableCmdVelChannelIndx_ != -1 && msg->mapped_channels[disableCmdVelChannelIndx_] == disableCmdVelChannelValue_)
		{
			return;
		}

		if (turboChannelIndx_ != -1)
		{
			turboMode = msg->mapped_channels[turboChannelIndx_] == turboChannelValue_;
		}

		minSpeedFinal = turboMode ? minTurboSpeed_ : minSpeed_;
		maxSpeedFinal = turboMode ? maxTurboSpeed_ : maxSpeed_;
		minTurnFinal = turboMode ? minTurboTurn_ : minTurn_;
		maxTurnFinal = turboMode ? maxTurboTurn_ : maxTurn_;

		proportional = static_cast<double>(msg->mapped_channels[forwardChannelIndx_] - sbusMinValue_) / sbusRange_;
		double fwdSpeed = minSpeedFinal + (maxSpeedFinal - minSpeedFinal) * proportional;

		proportional = static_cast<double>(msg->mapped_channels[turnChannelIndx_] - sbusMinValue_) / sbusRange_;
		double turn = -(minTurnFinal + (maxTurnFinal - minTurnFinal) * proportional);

		// Adjust for convenient (human-understandable) reverse motion
		if (fwdSpeed < 0)
			turn = -turn;

		// Adjust for deadband
		if (std::abs(fwdSpeed) < deadband_)
			fwdSpeed = 0.0;
		if (std::abs(turn) < deadband_)
			turn = 0.0;

		geometry_msgs::msg::Twist twist;
		twist.linear.x = fwdSpeed;
		twist.angular.z = turn;

		// Publish either Twist or TwistStamped message
		if (useStamped_)
		{
			geometry_msgs::msg::TwistStamped twistStamped;
			twistStamped.twist = twist;
			twistStamped.header.stamp = rclcpp::Time(msg->header.stamp);
			twistStamped.header.frame_id = frameId_;

			if (msg->mapped_channels[enableControlChannelIndex_] == enableControlChannelValue_)
				cmd_vel_stamped_pub_->publish(twistStamped);
			else
			{
				twistStamped.twist.linear.x = 0.0;
				twistStamped.twist.linear.y = 0.0;
				twistStamped.twist.linear.z = 0.0;
				twistStamped.twist.angular.x = 0.0;
				twistStamped.twist.angular.y = 0.0;
				twistStamped.twist.angular.z = 0.0;
				cmd_vel_stamped_pub_->publish(twistStamped);
			}
		}
		else
		{
			if (msg->mapped_channels[enableControlChannelIndex_] == enableControlChannelValue_)
				cmd_vel_pub_->publish(twist);
			else
			{
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				twist.linear.z = 0.0;
				twist.angular.x = 0.0;
				twist.angular.y = 0.0;
				twist.angular.z = 0.0;
				cmd_vel_pub_->publish(twist);
			}
		}
		// Enable joystick control
		std_msgs::msg::Bool enableControl;
		enableControl.data = false;
		if (msg->mapped_channels[enableControlChannelIndex_] == enableControlChannelValue_)
		{
			enableControl.data = true;
		}
		enable_pub_->publish(enableControl);

		if (!enableUtilCommands_)
			return;

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

		sbus_serial::msg::SbusCommands sbusCommands;
		sbusCommands.battery_charge = false;
		sbusCommands.lights = false;
		sbusCommands.horn = false;

		// Battery control
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
			{
				sbusCommands.battery_charge = true;
			}
		}
		// Lights control
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
			{
				sbusCommands.lights = true;
			}
		}
		// Horn control
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
			{
				sbusCommands.horn = true;
			}
		}

		commands_pub_->publish(sbusCommands);
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