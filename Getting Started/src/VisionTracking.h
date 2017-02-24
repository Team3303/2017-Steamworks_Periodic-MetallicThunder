#pragma once
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {
// VisionTracking class.
// An OpenCV pipeline generated by GRIP.
class Pipeline {
	private:
		cv::Mat hsvThresholdOutput;
		cv::Mat maskOutput;
		void hsvThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void mask(cv::Mat &, cv::Mat &, cv::Mat &);

	public:
		Pipeline();
		void Process(cv::Mat& source0);
		cv::Mat* GetHsvThresholdOutput();
		cv::Mat* GetMaskOutput();
};
} // end namespace grip