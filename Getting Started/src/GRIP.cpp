///*
// * GRIP.cpp
// *
// *  Created on: Feb 4, 2017
// *      Author: Owner
// */
//
//#include "GRIP.h"
//
//namespace grip {
//
//GripPipeline::GripPipeline() {
//}
///**
//* Runs an iteration of the pipeline and updates outputs.
//*/
//void GripPipeline::Process(cv::Mat& source0){
//	//Step HSV_Threshold0:
//	//input
//	cv::Mat hsvThresholdInput = source0;
//	double hsvThresholdHue[] = {24.280575539568343, 54.06143344709896};
//	double hsvThresholdSaturation[] = {71.08812949640287, 150.5631399317406};
//	double hsvThresholdValue[] = {110.0719424460432, 255.0};
//	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
//	//Step Mask0:
//	//input
//	cv::Mat maskInput = source0;
//	cv::Mat maskMask = hsvThresholdOutput;
//	mask(maskInput, maskMask, this->maskOutput);
//	//Step Find_Blobs0:
//	//input
//	cv::Mat findBlobsInput = maskOutput;
//	double findBlobsMinArea = 20.0;  // default Double
//	double findBlobsCircularity[] = {0.0, 0.7440273037542662};
//	bool findBlobsDarkBlobs = false;  // default Boolean
//	findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, this->findBlobsOutput);
//}
//
///**
// * This method is a generated getter for the output of a HSV_Threshold.
// * @return Mat output from HSV_Threshold.
// */
//cv::Mat* GripPipeline::GetHsvThresholdOutput(){
//	return &(this->hsvThresholdOutput);
//}
///**
// * This method is a generated getter for the output of a Mask.
// * @return Mat output from Mask.
// */
//cv::Mat* GripPipeline::GetMaskOutput(){
//	return &(this->maskOutput);
//}
///**
// * This method is a generated getter for the output of a Find_Blobs.
// * @return BlobsReport output from Find_Blobs.
// */
//std::vector<cv::KeyPoint>* GripPipeline::GetFindBlobsOutput(){
//	return &(this->findBlobsOutput);
//}
//	/**
//	 * Segment an image based on hue, saturation, and value ranges.
//	 *
//	 * @param input The image on which to perform the HSL threshold.
//	 * @param hue The min and max hue.
//	 * @param sat The min and max saturation.
//	 * @param val The min and max value.
//	 * @param output The image in which to store the output.
//	 */
//	void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
//		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
//		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
//	}
//
//		/**
//		 * Filter out an area of an image using a binary mask.
//		 *
//		 * @param input The image on which the mask filters.
//		 * @param mask The binary image that is used to filter.
//		 * @param output The image in which to store the output.
//		 */
//		void GripPipeline::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
//			mask.convertTo(mask, CV_8UC1);
//			cv::bitwise_xor(output, output, output);
//			input.copyTo(output, mask);
//		}
//
//	/**
//	 * Detects groups of pixels in an image.
//	 *
//	 * @param input The image on which to perform the find blobs.
//	 * @param minArea The minimum size of a blob that will be found.
//	 * @param circularity The minimum and maximum circularity of blobs that will be found.
//	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
//	 * @param blobList The output where the MatOfKeyPoint is stored.
//	 */
//	//void findBlobs(Mat *input, double *minArea, double circularity[2],
//		//bool *darkBlobs, vector<KeyPoint> *blobList) {
//	void GripPipeline::findBlobs(cv::Mat &input, double minArea, double circularity[], bool darkBlobs, std::vector<cv::KeyPoint> &blobList) {
//		blobList.clear();
//		cv::SimpleBlobDetector::Params params;
//		params.filterByColor = 1;
//		params.blobColor = (darkBlobs ? 0 : 255);
//		params.minThreshold = 10;
//		params.maxThreshold = 220;
//		params.filterByArea = true;
//		params.minArea = minArea;
//		params.filterByCircularity = true;
//		params.minCircularity = circularity[0];
//		params.maxCircularity = circularity[1];
//		params.filterByConvexity = false;
//		params.filterByInertia = false;
//		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
//		detector->detect(input, blobList);
//	}
//
//
//
//} // end grip namespace


#include "GrIP.h"

namespace grip {

GripPipeline::GripPipeline() {
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripPipeline::Process(cv::Mat& source0){
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = source0;
	double hsvThresholdHue[] = {85.7913669064748, 117.03071672354949};
	double hsvThresholdSaturation[] = {64.20863309352518, 255.0};
	double hsvThresholdValue[] = {0.0, 255.0};
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
}

/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* GripPipeline::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}



} // end grip namespace
