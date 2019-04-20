#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
// #include <opencv2/highgui.hpp>
// #include <iostream>

int main(int argc,char **argv)
{
	ros::init(argc,argv,"patch");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	// cv::Mat image;
	 while (ros::ok())
	 {
	 	std_msgs::String msg;
	 	msg.data = "hi";
	 	chatter_pub.publish(msg);
	 	ros::spinOnce();
	 	loop_rate.sleep();
	 }
	 return 0;
}