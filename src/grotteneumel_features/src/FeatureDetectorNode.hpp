#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

class FeatureDetectorNode
{
	public:
		FeatureDetectorNode(ros::NodeHandle nh);

	private:
		image_transport::ImageTransport it;
		image_transport::Subscriber sub;
		image_transport::Publisher pub;

		void imageMessageCallback(const sensor_msgs::ImageConstPtr& msg);
};


FeatureDetectorNode::FeatureDetectorNode(ros::NodeHandle nh) : it(nh)
{
	sub = it.subscribe("/pseye/image_color", 1, &FeatureDetectorNode::imageMessageCallback, this);
	pub = it.advertise("image", 1);
}

void FeatureDetectorNode::imageMessageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);			

	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	pub.publish(cv_ptr->toImageMsg());
}
