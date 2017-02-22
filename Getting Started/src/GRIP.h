///*
// * GRIP.h
// *
// *  Created on: Feb 4, 2017
// *      Author: Owner
// */
//
//#ifndef SRC_GRIP_H_
//#define SRC_GRIP_H_
//
//#pragma once
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d.hpp>
//#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>
//#include <map>
//#include <vector>
//#include <string>
//#include <math.h>
//
//namespace grip {
//
///**
//* GripPipeline class.
//*
//* An OpenCV pipeline generated by GRIP.
//*/
//class GripPipeline {
//	private:
//		cv::Mat hsvThresholdOutput;
//		cv::Mat maskOutput;
//		std::vector<cv::KeyPoint> findBlobsOutput;
//		void hsvThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
//		void mask(cv::Mat &, cv::Mat &, cv::Mat &);
//		void findBlobs(cv::Mat &, double , double [], bool , std::vector<cv::KeyPoint> &);
//
//	public:
//		GripPipeline();
//		void Process(cv::Mat& source0);
//		cv::Mat* GetHsvThresholdOutput();
//		cv::Mat* GetMaskOutput();
//		std::vector<cv::KeyPoint>* GetFindBlobsOutput();
//};
//
//
//} // end namespace grip




#pragma once
#include "vision/VisionPipeline.h"

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

/**
* GripPipeline class.
*
* An OpenCV pipeline generated by GRIP.
*/
class GripPipeline : public frc::VisionPipeline {
	private:
		cv::Mat hsvThresholdOutput;
		void hsvThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);

	public:
		GripPipeline();
		void Process(cv::Mat& source0) override;
		cv::Mat* GetHsvThresholdOutput();
};


} // end namespace grip


