#include <stdint.h>
#include <rclcpp/rclcpp.hpp>

// Messages
#include <px4_msgs/msg/vehicle_odometry.hpp> // TODO noter quelque part comment on en arrive la

// My includes
#include "my_px4_tutorial/OffboardControl.hpp"
#include "my_px4_tutorial/Planner.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

using namespace std::chrono;
using namespace std::chrono_literals;

Planner::Planner() : Node("planner_node")
{
	std::cout << "start planner node" << std::endl;
	_flag_odom_sub = 0;

	// Setup QoS and subscriber
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile); // originalement 5

	_vehicle_odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/out/vehicle_odometry", 
		qos, 
		std::bind(&Planner::vehicle_odometry_callback, 
				  this,
		          std::placeholders::_1));

	// wait to receive a first message
	while(!_flag_odom_sub)
	{
		rclcpp::spin_some(this->get_node_base_interface());
		RCLCPP_INFO(this->get_logger(), "No sub yet");
	}

	setup(_controller);

	follow_position(_controller, _goal_poses);

	go_back(_controller, _initial_pose);

}

Planner::~Planner()
{
	// nothing
}

// ###################
// ### Subscribers ###
// ###################

/**
 * @brief Call back function of the odometry
 * @param odom_msg: odometry message
 */
void Planner::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
	_current_odom_msg = odom_msg;
	_flag_odom_sub = 1;
	// std::cout << "POSITION : " << std::endl;
	// std::cout << "x : " << _current_odom_msg->position[0] << std::endl;
	// std::cout << "y : " << _current_odom_msg->position[1] << std::endl;
	// std::cout << "z : " << _current_odom_msg->position[2] << std::endl;
}

/**
 * @brief setup the drone: get initial pose and arm the drone
 * @param controller: drone's controler
*/
void Planner::setup(OffboardControl& controller) // mettre const pour protéger ? 
{
	_initial_pose = {_current_odom_msg->position[0],
					 _current_odom_msg->position[1],
					 _current_odom_msg->position[2]};

	// hardcoded test for the follow poses function
	_goal_poses.push_back({1,1,-5});
	_goal_poses.push_back({2,-2,-5});
	_goal_poses.push_back({5,3,-10});
	_goal_poses.push_back({12,-4,-8});

	controller.publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	
	// Arm the vehicle
	controller.arm();
}

/**
 * @brief check if the drone reached the goal.
 * 
 * @param pose: array that represents current drone's pose
 * @param goal: array that represents drone's goal
 * @param tolerance: tolerance value to reach the goal
 * @return true goal is receahed
 * @return false goal is not reached
 */
bool Planner::is_goal_reached(std::array<float,3> pose, std::array<float,3> goal, float tolerance)
{
	return((std::abs(pose[0] - goal[0]) <= tolerance) &&
		   (std::abs(pose[1] - goal[1]) <= tolerance) &&
		   (std::abs(pose[2] - goal[2]) <= tolerance));
}

/**
 * @brief simple takeoff -> drone arms, takes off for 7 secondes and then disarms
 * @param controller: drone's controler
*/
void Planner::take_off(OffboardControl& controller)
{
	// control the drone on position
	controller.publish_offboard_control_mode(true, false); 

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;
	int flag = 1;
	while(flag)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;
		if(elapsed_time < 10000) 
		{
			controller.publish_trajectory_setpoint(0.0, 0.0, -7.0, -3.14);
		}
		else
		{
			flag = 0;
			RCLCPP_INFO(this->get_logger(), "Sutdowning");
			controller.disarm();
		}
	}
}

/**
 * @brief: does a sqare around the global origin. Control in position
 * @param controller: drone's controler
*/
void Planner::square_hardcoded(OffboardControl& controller)
{
	RCLCPP_INFO(this->get_logger(), "INSIDE");
	int square_flag = 1; // if 1 fly, otherwise, disarm

	// prediod for each pose
	int period = 5000;
	// lenght of the square
	int length = 5;

	// get the initial position pose when this function is launched from subscriber
	px4_msgs::msg::VehicleOdometry::SharedPtr initial_pose = _current_odom_msg;
	int x0 = initial_pose->position[0];
	int y0 = initial_pose->position[1];

	auto start_time = this->get_clock()->now().nanoseconds() / 1000000;

	while(square_flag)
	{
		auto current_time = this->get_clock()->now().nanoseconds() / 1000000;
		auto elapsed_time = current_time - start_time;

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary


		if(elapsed_time  < 2000)
		{
			RCLCPP_INFO(this->get_logger(), "position 0");
			controller.publish_trajectory_setpoint(x0, y0, -3.0, -3.14);
		}
			
		else if(elapsed_time  < 2*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 1");
			controller.publish_trajectory_setpoint(x0 + length, y0, -6.0, -3.14);
		}
		
		else if(elapsed_time  < 3*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 2");
			controller.publish_trajectory_setpoint(x0, y0 + length, -7.0, -3.14);
		}
		
		else if(elapsed_time  < 4*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 3");
			controller.publish_trajectory_setpoint(x0 - length, y0, -3.0, -3.14);
		}

		else if(elapsed_time  < 5*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 4");
			controller.publish_trajectory_setpoint(x0, y0 - length, -3.0, -3.14);
		}

		else if(elapsed_time  < 6*period)
		{
			RCLCPP_INFO(this->get_logger(), "position 0 again");
			controller.publish_trajectory_setpoint(x0, y0, -3.0, -3.14);
		}
		else
		{
			square_flag = 0;
			controller.disarm();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		rclcpp::spin_some(this->get_node_base_interface()); // ça marche mais ce serait bien de trouver une autre option
	}
}

/**
 * @brief: makes the drone reach every positions of goal_poses
 * 
 * @param controller: drone's controler
 * @param goal_poses: vector of 3D poses to reach
 */
void Planner::follow_position(OffboardControl& controller, std::vector<std::array<float,3>>& goal_poses)
{
	RCLCPP_INFO(this->get_logger(), "Enter follow position");

	std::array<float,3> current_goal;
	
	int counter = 1;
	while(!goal_poses.empty()) // check if there are other goals to reach
	{
		// current goal is the first element in goal_poses
		current_goal = goal_poses[0];
		std::cout << "Current goal (number " << counter++ << ") :" << current_goal[0] << " " << current_goal[1]<< " " << current_goal[2] << std::endl;

		// call go to pose function in a thread
		std::thread go_to_pose_thread = std::thread(&Planner::go_to_pose, this, std::ref(controller), std::ref(current_goal));
		go_to_pose_thread.join(); 

		RCLCPP_INFO(this->get_logger(), "New goal set");
		std::cout << "New goal: "<< current_goal[0] << " " << current_goal[1]<< " " << current_goal[2] << std::endl;

		// once the goal reached, erased the first element
		goal_poses.erase(goal_poses.begin());
		
    }
	RCLCPP_INFO(this->get_logger(), "No more goal");
}

/**
 * @brief makes the drone goes to the given position of goal_pose 
 * 
 * @param controller: drone's controler
 * @param goal_pose: 3D array containing one goal to reach
*/
void Planner::go_to_pose(OffboardControl& controller, std::array<float,3> goal_pose)
{
	std::array<float,3> current_pose;

	goal_pose[2] = -5; // set an accesible height position.

	// get current position
	current_pose = {_current_odom_msg->position[0],
					_current_odom_msg->position[1],
					_current_odom_msg->position[2]};

	while(!is_goal_reached(current_pose, goal_pose, 0.5))
	{
		// get current position
		current_pose = {_current_odom_msg->position[0],
						_current_odom_msg->position[1],
						_current_odom_msg->position[2]};

		// control the drone on position
		controller.publish_offboard_control_mode(true, false); // must be published regulary
		controller.publish_trajectory_setpoint(goal_pose[0],
											   goal_pose[1],
									    	   goal_pose[2],
											   -3.14); 
		// "wait" for 100ms and make all node spin (update the drones position information)
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		rclcpp::spin_some(this->get_node_base_interface());
	}
	RCLCPP_INFO(this->get_logger(), "Goal reached");
	// controller.disarm();
}

/**
 * @brief go back to initial position 
 * 
 * @param home_pose: position to reach back
*/
void Planner::go_back(OffboardControl& controller, std::array<float,3> home_pose)
{
	RCLCPP_INFO(this->get_logger(), "Go back to home position");	
	
	// call go to pose function in a thread, 
	std::thread go_to_pose_thread = std::thread(&Planner::go_to_pose, this, std::ref(controller), std::ref(home_pose));
	go_to_pose_thread.join(); 

	RCLCPP_INFO(this->get_logger(), "Back to home position");

	// Disarm the drone
	controller.disarm();
}

//
/**
 * @brief do circle around a position, control in speed // in another function
 * 
*/


int main(int argc, char *argv[])
{
	std::cout << "Starting Planner node" << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Planner>());
	// rclcpp::shutdown();
	return 0;
}



/*

	faire le control en vitesse

*/