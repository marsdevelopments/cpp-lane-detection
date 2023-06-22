#pragma once

#include <opencv2/opencv.hpp>

#include "constants.h"

class PreProcessing
{
public:
    PreProcessing();

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    cv::Mat process_threshold(const cv::Mat &source);

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    cv::Mat process_canny(const cv::Mat &source);

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    cv::Mat process_custom(const cv::Mat &source);

private:
    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    void apply_roi_trapezoid();

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    void apply_roi_rectangle();

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    void apply_grayscale();

    /**
     * @brief Apply Gaussian blur to image.
     *
     * @param source image that needs to be blurred
     * @return blurred image
     */
    void apply_gaussian_blur();

    /**
     * @brief Apply thresholding to image.
     *
     * @param source image that needs to be thresholded
     * @return thresholded image
     */
    void apply_thresholding();

    /**
     * @brief Apply thresholding to image.
     *
     * @param source image that needs to be thresholded
     * @return thresholded image
     */
    void apply_range();

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    void apply_canny();

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    void apply_lane_filter(const int tau = cst::kLaneLineWidth);

    /**
     * @brief Apply grayscale transform on image.
     *
     * @param source image that needs to be transformed
     * @return grayscale image
     */
    bool is_near_range(const int row_index, const int col_index, const uint8_t margin);

    cv::Mat frame_;

    const std::array<cv::Point, 8> points = {
        cv::Point((cst::kVideoWidth * (1 - cst::kTrapezoidBottomWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight),
        cv::Point((cst::kVideoWidth * (1 - cst::kTrapezoidTopWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight - cst::kVideoHeight * cst::kTrapezoidHeight),
        cv::Point(cst::kVideoWidth - (cst::kVideoWidth * (1 - cst::kTrapezoidTopWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight - cst::kVideoHeight * cst::kTrapezoidHeight),
        cv::Point(cst::kVideoWidth - (cst::kVideoWidth * (1 - cst::kTrapezoidBottomWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight),

        cv::Point((cst::kVideoWidth * (1 - cst::kTrapezoidBottomWidth * cst::kSmallBottomWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight),
        cv::Point((cst::kVideoWidth * (1 - cst::kTrapezoidTopWidth * cst::kSmallTopWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight - cst::kVideoHeight * cst::kTrapezoidHeight * cst::kSmallHeight),
        cv::Point(cst::kVideoWidth - (cst::kVideoWidth * (1 - cst::kTrapezoidTopWidth * cst::kSmallTopWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight - cst::kVideoHeight * cst::kTrapezoidHeight * cst::kSmallHeight),
        cv::Point(cst::kVideoWidth - (cst::kVideoWidth * (1 - cst::kTrapezoidBottomWidth * cst::kSmallBottomWidth)) / 2 + cst::kTrapezoidOffsetX, cst::kVideoHeight)};
};
