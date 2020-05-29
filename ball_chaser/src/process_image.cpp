#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;


void drive_robot(float lin, float ang){ 
	
	//request a service and pass the required control commands
	ball_chaser::DriveToTarget srv; 
	
	srv.request.linear_x = lin;
	srv.request.angular_z = ang;
	
	if(!client.call(srv))
		ROS_ERROR("Unable to call the requested service.");
}

void process_image_callback(const sensor_msgs::Image img){ 
	
	
	int data_size = img.height*img.step;
	int section_factor = img.step/3;     //divides image in three parts: left, center & right
	int regional_factor = 0;
	bool ball_located = false;
	
	for (int i=0; i < data_size; i = i+3)
	{
		if ((img.data[i] == 255) && (img.data[i+1] == 255) && (img.data[i+2] == 255)) //check RGB value for each pixel
		{
			regional_factor = i % img.step;
			
			if (regional_factor < section_factor) //drive left
				drive_robot(0.3, 0.5);
			else if (regional_factor > 2*section_factor)  //drive right
				drive_robot(0.3, -0.5);
			else 
				drive_robot(0.3, 0.0);  //drive straight
			ball_located = true;
			break;
		}
	}
		
	if (ball_located == false)
		drive_robot(0.0,0.0);  //stop driving
				
}

int main(int argc, char** argv){
	
	//initialize the node
	ros::init(argc, argv, "process_image");
	
	//create a node handle
	ros::NodeHandle nh;
	
	//define client service
	client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
	
	
	//subscribe to the recorded RGB camera data
	ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
	
	ros::spin();
	
	return 0;
	
}
