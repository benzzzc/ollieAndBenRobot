// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// 	http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <thread>
#include <functional>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <array>
#include <cmath>
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GoalMovementMover6 : public rclcpp::Node
{
public:
	GoalMovementMover6()
	: Node("GoalMovementMover6"), count_(0) {
    	//RCLCPP_INFO(this->get_logger(),"Constructor");
    	cb_group_=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    	tm_group_=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    	rclcpp::SubscriptionOptions options;
    	options.callback_group=cb_group_;
    	publisherJointPosition_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
    	subscriptionJointPosition_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&GoalMovementMover6::topic_jointStatesCallback, this, _1),options);
    	timer_ = this->create_wall_timer(4000ms, std::bind(&GoalMovementMover6::timer_callback, this),tm_group_);
   	 
	}
protected:

private:
	void timer_callback() {
    	move_my_robot();
	}
	void topic_jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
    	int i=0;
    	for(std::vector<double>::const_iterator it = msg->position.begin(); it != msg->position.end(); ++it) {
        	if(i==0) {
            	joint1_angle=*it;
        	}
        	if(i==1) {
            	joint2_angle=*it;
        	}
        	if(i==2) {
            	joint3_angle=*it;
        	}
        	if(i==3) {
            	joint4_angle=*it;
        	}
        	if(i==4) {
            	joint5_angle=*it;
        	}
        	if(i==5) {
            	joint6_angle=*it;
        	}   	 
    	i++;
    	}
    	known_states=true;
    	//RCLCPP_INFO(this->get_logger(),"Received State %f\t%f\t%f\t%f\t%f\t%f", joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle);
	}
	void send_msg(const std::array<float, 6> &velocities) {
    	auto msg_start = control_msgs::msg::JointJog();
		for (size_t i = 0; i < velocities.size(); ++i) {
	    	std::stringstream ss;
	    	ss << "joint" << (i + 1);
	    	msg_start.joint_names.push_back(ss.str());
	    	msg_start.velocities.push_back(velocities[i]);
		}
    	publisherJointPosition_->publish(msg_start);
	}
	void move_my_robot() {
    	//RCLCPP_INFO(this->get_logger(),"move_my_robot");
    	rclcpp::Rate loop_rate(20);  
    	while(known_states==false) {
        	RCLCPP_INFO(this->get_logger(),"Waiting");
        	loop_rate.sleep();
    	}
		// Set desired targets for all six joints (radians). Adjust to suit your move goal.
		std::array<float, 6> joint_demands = {
			0.9f,   // joint 1
			0.5f,   // joint 2
			0.0f,   // joint 3
			0.3f,   // joint 4
			0.0f,   // joint 5
			0.0f    // joint 6
		};

    	if(known_states) {
			while (rclcpp::ok()) {
				std::array<float, 6> current = {
					joint1_angle, joint2_angle, joint3_angle,
					joint4_angle, joint5_angle, joint6_angle
				};

				std::array<float, 6> velocities{};
				for (size_t i = 0; i < joint_demands.size(); ++i) {
					float error = joint_demands[i] - current[i];
					if (std::abs(error) > 0.03f) {
						velocities[i] = 0.25f * (error / std::abs(error));
					} else {
						velocities[i] = 0.0f;
					}
				}

				send_msg(velocities);
				loop_rate.sleep();
			}
    	}
	}
	bool known_states=false;
	float joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle;
	float jointdemand_1, jointdemand_2, jointdemand_3, jointdemand_4, jointdemand_5, jointdemand_6;
	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp::CallbackGroup::SharedPtr tm_group_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisherJointPosition_;  
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointPosition_;
	unsigned long long state=0;
	size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	//auto node = rclcpp::Node::make_shared("GoalMovementMover6");
	auto node = std::make_shared<GoalMovementMover6>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	//rclcpp::spin(std::make_shared<GoalMovementMover6>());
	rclcpp::shutdown();
	return 0;
}
