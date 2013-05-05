#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "HarrisDetector.hpp"

class FeatureDetectorNode
{
	public:
		FeatureDetectorNode(ros::NodeHandle nh);

	private:
		image_transport::ImageTransport it;
		image_transport::Subscriber sub;
		image_transport::Publisher pub;
		boost::shared_ptr<HarrisDetector> harrisDetector;

		void imageMessageCallback(const sensor_msgs::ImageConstPtr& msg);
};


FeatureDetectorNode::FeatureDetectorNode(ros::NodeHandle nh) : it(nh)
{
	sub = it.subscribe("/pseye/image_mono", 1, &FeatureDetectorNode::imageMessageCallback, this);
	pub = it.advertise("image", 1);

	boost::shared_ptr<HarrisDetector> tempPtr(new HarrisDetector());
	harrisDetector = tempPtr;
}

void FeatureDetectorNode::imageMessageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);			

	std::vector<cv::KeyPoint> keyPoints;

	harrisDetector->detectImpl(cv_ptr->image,keyPoints);
	harrisDetector->drawKeyPoints(cv_ptr->image,keyPoints);

	pub.publish(cv_ptr->toImageMsg());
}
