// Node-level test of winch mock mode: with winchMock=true the node must not
// call any service; instead it publishes the name of the service it would
// have called on "winch_mock", once per change, and still zeroes the twist
// while in winch mode.
#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sbus_commands_node.hpp"

namespace
{
constexpr int CH_FORWARD = 1;
constexpr int CH_MODE = 5;
constexpr int CH_ENABLE = 7;
}  // namespace

class SbusCommandsMockTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
	static void TearDownTestSuite() { rclcpp::shutdown(); }

	void SetUp() override
	{
		rclcpp::NodeOptions opts;
		opts.parameter_overrides({
			rclcpp::Parameter("sbusMinValue", -100),
			rclcpp::Parameter("sbusMaxValue", 100),
			rclcpp::Parameter("forwardChannelIndx", CH_FORWARD),
			rclcpp::Parameter("turnChannelIndx", 3),
			rclcpp::Parameter("enableControlChannelIndex", CH_ENABLE),
			rclcpp::Parameter("enableControlChannelValue", 100.0),
			rclcpp::Parameter("modeChannelIndex", CH_MODE),
			rclcpp::Parameter("winchModeIndex", 2),
			rclcpp::Parameter("winchChannelIndx", CH_FORWARD),
			rclcpp::Parameter("winchMock", true),
			rclcpp::Parameter("timeoutThreshold", 10.0),  // keep the loss-of-signal timer out of the way
		});
		node_ = std::make_shared<SbusCommands>(opts);
		probe_ = std::make_shared<rclcpp::Node>("probe");

		sbus_pub_ = probe_->create_publisher<sbus_serial::msg::Sbus>("sbus", 10);
		mock_sub_ = probe_->create_subscription<std_msgs::msg::String>(
			"winch_mock", 10,
			[this](std_msgs::msg::String::SharedPtr m) { mock_.push_back(m->data); });
		twist_sub_ = probe_->create_subscription<geometry_msgs::msg::TwistStamped>(
			"output/sbus/cmd_vel", 10,
			[this](geometry_msgs::msg::TwistStamped::SharedPtr m) { last_x_ = m->twist.linear.x; ++twists_; });

		exec_.add_node(node_);
		exec_.add_node(probe_);
		pump(300);  // discovery
	}

	void pump(int ms)
	{
		const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
		while (std::chrono::steady_clock::now() < end)
		{
			exec_.spin_some();
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}

	// Publish the same frame `repeat` times (the node runs once per frame).
	void frames(int mode_ch, int enable_ch, int fwd_ch, int repeat = 4)
	{
		sbus_serial::msg::Sbus msg;
		msg.mapped_channels.fill(0);
		msg.mapped_channels[CH_MODE] = mode_ch;
		msg.mapped_channels[CH_ENABLE] = enable_ch;
		msg.mapped_channels[CH_FORWARD] = fwd_ch;
		for (int i = 0; i < repeat; ++i)
		{
			sbus_pub_->publish(msg);
			pump(40);
		}
	}

	std::shared_ptr<SbusCommands> node_;
	rclcpp::Node::SharedPtr probe_;
	rclcpp::executors::SingleThreadedExecutor exec_;
	rclcpp::Publisher<sbus_serial::msg::Sbus>::SharedPtr sbus_pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mock_sub_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
	std::vector<std::string> mock_;
	double last_x_ = 0.0;
	int twists_ = 0;
};

TEST_F(SbusCommandsMockTest, DrivingModePublishesTwistAndNoMockCommands)
{
	frames(-100, 100, 50);  // mode 0, enabled, half forward
	ASSERT_GT(twists_, 0);
	EXPECT_NEAR(last_x_, 0.5, 1e-6);
	EXPECT_TRUE(mock_.empty());
}

TEST_F(SbusCommandsMockTest, WinchModeZeroesTwistAndPublishesServiceNamesOncePerChange)
{
	frames(100, 100, 0);  // winch mode, stick centred: arms, twist zero
	EXPECT_DOUBLE_EQ(last_x_, 0.0);
	EXPECT_TRUE(mock_.empty());

	frames(100, 100, -100, 6);  // full back, held: wind, exactly once
	EXPECT_DOUBLE_EQ(last_x_, 0.0);
	ASSERT_EQ(mock_, (std::vector<std::string>{"/winch/winch/wind"}));

	frames(100, 100, 0, 6);  // centre: stop, exactly once
	ASSERT_EQ(mock_, (std::vector<std::string>{"/winch/winch/wind", "/winch/winch/stop"}));

	frames(100, 100, 100, 6);  // full forward: unwind
	ASSERT_EQ(mock_.size(), 3u);
	EXPECT_EQ(mock_.back(), "/winch/winch/unwind");

	frames(-100, 100, 100, 6);  // back to driving with the stick still held: stop, twist live again
	ASSERT_EQ(mock_.size(), 4u);
	EXPECT_EQ(mock_.back(), "/winch/winch/stop");
	EXPECT_NEAR(last_x_, 1.0, 1e-6);
}
