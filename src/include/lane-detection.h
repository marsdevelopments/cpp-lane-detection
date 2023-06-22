/*                                  
 * lane-detection.h
 *
 * Created on: 10 July 2022
 * Author: EtoileScintillante
 */

#ifndef __LANE_DETECTION__
#define __LANE_DETECTION__

#include <cmath>
#include "regression.h"

cv::Mat pre_process_frame(const cv::Mat& source);

/**
 * @brief Apply grayscale transform on image.
 * 
 * @param source image that needs to be transformed
 * @return grayscale image
 */
cv::Mat apply_grayscale(const cv::Mat& source);

/**
 * @brief Apply Gaussian blur to image.
 * 
 * @param source image that needs to be blurred
 * @return blurred image
 */
cv::Mat apply_gaussian_blur(const cv::Mat& source);

/**
 * @brief Apply thresholding to image.
 * 
 * @param source image that needs to be thresholded
 * @return thresholded image
 */
cv::Mat apply_thresholding(const cv::Mat& source);

/**
 * @brief Detect edges of image by applying canny edge detection.
 * 
 * @param source image of which the edges needs to be detected
 * @return image with detected edges
 */
cv::Mat applyCanny(cv::Mat source);

/**
 * @brief Filter source image so that only the white and yellow pixels remain.
 * A gray filter will also be added if the source image is classified as taken during the night.
 * One assumption for lane detection here is that lanes are either white or yellow.
 * 
 * @param source source image
 * @param isDayTime true if image is taken during the day, false if at night
 * @see isDayTime
 * @return Mat filtered image
 */
cv::Mat filterColors(cv::Mat source, bool isDayTime);

/**
 * @brief Apply an image mask. 
 * Only keep the part of the image defined by the
 * polygon formed from four points. The rest of the image is set to black.
 * 
 * @param source image on which to apply the mask
 * @return Mat image with mask
 */
cv::Mat RegionOfInterest(cv::Mat source);

/**
 * @brief Creates mask and blends it with source image so that the lanes
 * are drawn on the source image.
 * 
 * @param source source image
 * @param lines vector < vec4i > holding the lines
 * @return Mat image with lines drawn on it
 */
cv::Mat drawLanes(cv::Mat source, std::vector<cv::Vec4i> lines);

/**
 * @brief Returns a vector with the detected hough lines.
 * 
 * @param canny image resulted from a canny transform
 * @param source image on which hough lines are drawn
 * @param drawHough draw detected lines on source image if true. 
 * It will also show the image with de lines drawn on it, which is why
 * it is not recommended to pass in true when working with a video. 
 * @see applyCanny
 * @return vector<Vec4i> contains hough lines.
 */
std::vector<cv::Vec4i> houghLines(cv::Mat canny, cv::Mat source, bool drawHough);

/**
 * @brief Determine whether a picture is taken during day or night time.
 * Returns true when the image is classified as daytime. 
 * Note: this is based on the mean pixel value of an image and might not
 * always lead to accurate results.
 * 
 * @param source image
 * @return true 
 * @return false 
 */
bool isDayTime(cv::Mat source);

#endif /*__LANE_DETECTION__*/