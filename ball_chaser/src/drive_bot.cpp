#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//create a publisher to publish velocity commands
ros::Publisher vel_pub; 

bool CommandRequestCallback(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
	
	
	if (!(req.linear_x == 0.0 && req.angular_z == 0.0))
		ROS_WARN("Request to drive received - linear velocity: %1.2f, angular velocity: %1.2f", req.linear_x, req.angular_z);
	
	geometry_msgs::Twist vel_cmd;
	
	vel_cmd.linear.x = req.linear_x;
	vel_cmd.angular.z = req.angular_z;
	
	vel_pub.publish(vel_cmd);
	
	res.msg_feedback = "Sending the requested velocity commands to the robot.";
	
	if (!(req.linear_x == 0.0 && req.angular_z == 0.0))
		ROS_INFO_STREAM(res.msg_feedback);
	
	return true;
}
	
int main(int argc, char** argv){
	
	//initialize a ros node
	ros::init(argc, argv, "drive_bot");
	
	//create node handle
	ros::NodeHandle nh;
	
	//The declared publisher will publish velocity commands to type geometry_msgs/Twist to the topic /cmd_vel
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	//define a ros service with a callback function to handle to command requests
	ros::ServiceServer service = nh.advertiseService("/ball_chaser/command_robot", CommandRequestCallback);
	
	ros::spin();
	
	return 0;
}

	
