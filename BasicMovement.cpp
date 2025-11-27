// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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
#include <vector>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class GoalMovementMover6 : public rclcpp::Node
{
public:
    GoalMovementMover6()
    : Node("GoalMovementMover6"), count_(0) {
        
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        tm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        rclcpp::SubscriptionOptions options;
        options.callback_group = cb_group_;
        publisherJointPosition_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
        subscriptionJointPosition_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",  10, std::bind(&GoalMovementMover6::topic_jointStatesCallback, this, _1), options);
		// new subscriber, with the new call back function
        subscriptionJointDemand_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_demands", 10, std::bind(&GoalMovementMover6::topic_jointDemandCallback, this, _1), options);
        timer_ = this->create_wall_timer(50ms, std::bind(&GoalMovementMover6::timer_callback, this), tm_group_);
		//set all demands to zero, intially 
        jointdemand_1 = 0.0;
        jointdemand_2 = 0.0;
        jointdemand_3 = 0.0;
        jointdemand_4 = 0.0;
        jointdemand_5 = 0.0;
        jointdemand_6 = 0.0;
    }

private:
    void timer_callback() {
        move_my_robot();
    }

    // changed callback, bc idk what the hell the other one meant, hopefully ths works if not change back to og
    void topic_jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        if(msg->position.size() >= 6) {
            joint1_angle = msg->position[0];
            joint2_angle = msg->position[1];
            joint3_angle = msg->position[2];
            joint4_angle = msg->position[3];
            joint5_angle = msg->position[4];
            joint6_angle = msg->position[5];
            known_states = true;
        }
    }

    // demands callback
    void topic_jointDemandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        if(msg->position.size() >= 6) {
            jointdemand_1 = msg->position[0];
            jointdemand_2 = msg->position[1];
            jointdemand_3 = msg->position[2];
            jointdemand_4 = msg->position[3];
            jointdemand_5 = msg->position[4];
            jointdemand_6 = msg->position[5];
			valid_jointDemands = true;
			RCLCPP_INFO(this->get_logger(), "Valid joint demands recieved");
        }
    }

    void send_msg(std::vector<float> vels) {
        auto msg_start = control_msgs::msg::JointJog();
        msg_start.header.stamp = this->now();
        for(int i = 0; i < 6; i++) {
            std::stringstream ss;
            ss << "joint" << (i + 1); 
            msg_start.joint_names.push_back(ss.str());
            msg_start.velocities.push_back(vels[i]);
        }
        publisherJointPosition_->publish(msg_start);
    }

	void move_my_robot() {
		rclcpp::Rate loop_rate(20); // set the loop to 20hz

		while(!known_states) {
			RCLCPP_INFO(this->get_logger(), "Waiting");
			loop_rate.sleep();
		}
	
		while(!valid_jointDemands) {
			RCLCPP_INFO(this->get_logger(), "Waiting for CLI command");
			loop_rate.sleep();
		}
	
		while(1) {
			std::vector<float> demands = {
				jointdemand_1, jointdemand_2, jointdemand_3, 
				jointdemand_4, jointdemand_5, jointdemand_6
			};
			std::vector<float> currents = {
				joint1_angle, joint2_angle, joint3_angle, 
				joint4_angle, joint5_angle, joint6_angle
			};
			std::vector<float> velocities_to_send;
			
			// control logic
			for(int i = 0; i < 6; i++) {
				float error = demands[i] - currents[i];
				float vel = 0;
				if(std::abs(error) > 0.03) {
					vel = 0.25 * (error / std::abs(error));
				} else {
					vel = 0;
				}
				velocities_to_send.push_back(vel);
			}
			send_msg(velocities_to_send);
			loop_rate.sleep(); // ensures loop is at 20hz 
		}
	}

	// boolean flags
    bool known_states = false;
	bool valid_jointDemands = false;
    
    float joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle; 
    float jointdemand_1, jointdemand_2, jointdemand_3, jointdemand_4, jointdemand_5, jointdemand_6; 
    
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::CallbackGroup::SharedPtr tm_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisherJointPosition_;  
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointPosition_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointDemand_; //new subscriber 
    
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalMovementMover6>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
