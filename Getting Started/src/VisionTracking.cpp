#include "VisionTracking.h"

namespace grip {
Pipeline::Pipeline() {}
//Runs an iteration of the pipeline and updates outputs.
void Pipeline::Process(cv::Mat& source0){
	// HSV Threshold
	cv::Mat hsvThresholdInput = source0; // Input Image
	double hsvThresholdHue[] = {0.0, 180.0}; // Hue
	double hsvThresholdSaturation[] = {0.0, 255.0}; // Saturation
	double hsvThresholdValue[] = {0.0, 255.0}; // Value
	//Operation
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);

	// Mask
	cv::Mat maskInput = source0; // Input Image
	cv::Mat maskMask = hsvThresholdOutput; // Mask Image
	mask(maskInput, maskMask, this->maskOutput); // Operation
}

/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* Pipeline::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Mask.
 * @return Mat output from Mask.
 */
cv::Mat* Pipeline::GetMaskOutput(){
	return &(this->maskOutput);
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
	void Pipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

		/**
		 * Filter out an area of an image using a binary mask.
		 *
		 * @param input The image on which the mask filters.
		 * @param mask The binary image that is used to filter.
		 * @param output The image in which to store the output.
		 */
		void Pipeline::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
			mask.convertTo(mask, CV_8UC1);
			cv::bitwise_xor(output, output, output);
			input.copyTo(output, mask);
		}
} // end VisionTracking namespace

