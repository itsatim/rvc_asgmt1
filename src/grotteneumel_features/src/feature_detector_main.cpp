#include <ros/ros.h>
#include "FeatureDetectorNode.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "feature_detector");
	ros::NodeHandle nh;
	FeatureDetectorNode fdn(nh);
	ros::spin();
	return 0;
}
