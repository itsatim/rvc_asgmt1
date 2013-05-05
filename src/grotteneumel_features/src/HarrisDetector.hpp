#include <vector>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HarrisDetector : public cv::FeatureDetector
{
private:
	int windowSize;
	double threshold;
protected:
	void harrisCorners(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints) const
	{
		cv::Mat x_deriv;
		cv::Mat y_deriv;

		cv::Sobel(image,x_deriv,CV_32FC1,1,0);
		cv::Sobel(image,y_deriv,CV_32FC1,0,1);

		cv::Mat A = x_deriv.mul(x_deriv);
		cv::Mat B = x_deriv.mul(y_deriv);
		cv::Mat C = y_deriv.mul(y_deriv);

		cv::Size ksize;
		ksize.width = windowSize;
		ksize.height = windowSize;

		cv::GaussianBlur(A,A,ksize,0);
		cv::GaussianBlur(B,B,ksize,0);
		cv::GaussianBlur(C,C,ksize,0);

		cv::Mat H;

		cv::add(A,C,H);
		H.mul(H,0.04);
		cv::add(B.mul(B),H,H);
		cv::subtract(A.mul(C),H,H);

		double h_max;
		cv::minMaxLoc(H,0,&h_max);

		for (int x = 0; x < image.cols; x++)
		{
			for (int y = 0; y < image.rows; y++)
			{
				if (H.at<float>(y,x) > threshold * (float)h_max && H.at<float>(y,x) > 10)
				{
					cv::KeyPoint kp((float)x,(float)y,-1.0,0.0,0.0,-1.0);
					keyPoints.push_back(kp);
				}
			}
		}
	}

public:
	void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints, const cv::Mat& mask=cv::Mat()) const
	{
		harrisCorners(image, keyPoints);
	}

	void drawKeyPoints(cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints)
	{
		for (int i = 0; i < keyPoints.size(); i++)
		{
			cv::circle(image,keyPoints.at(i).pt,10,CV_RGB(255,0,0));
		}
	}

	HarrisDetector() : threshold(0.1), windowSize(5) {};
};


