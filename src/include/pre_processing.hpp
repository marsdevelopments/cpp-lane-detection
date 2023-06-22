#pragma once

#include <opencv2/opencv.hpp>

#include "constants.hpp"

class PreProcessing
{
public:
    PreProcessing();

    /**
     * @brief Proccess a frame using threshold method.
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_threshold(const cv::Mat &source);

    /**
     * @brief Proccess a frame using adaptive threshold method.
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_adaptive_threshold(const cv::Mat &source);

    /**
     * @brief Proccess a frame using canny edge detection method.
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_canny(const cv::Mat &source);

    /**
     * @brief Proccess a frame using custom method.
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_custom(const cv::Mat &source);

private:
    /**
     * @brief Extract Region Of Interest in the trapezoid shape from the frame class property.
     */
    void apply_roi_trapezoid();

    /**
     * @brief Extract Region Of Interest in the rectangle shape from the frame class property.
     */
    void apply_roi_rectangle();

    /**
     * @brief Apply the grayscale filter the frame class property.
     */
    void apply_grayscale();

    /**
     * @brief Apply the gaussian blur filter the frame class property.
     */
    void apply_gaussian_blur();

    /**
     * @brief Apply the thresholding filter the frame class property.
     */
    void apply_thresholding();

    /**
     * @brief Apply the thresholding filter the frame class property.
     */
    void apply_adaptive_thresholding();

    /**
     * @brief Apply the thresholding filter the frame class property.
     */
    void set_new_threshold(int difference);

    /**
     * @brief Apply the intensity range filter the frame class property.
     */
    void apply_range();

    /**
     * @brief Apply the canny filter the frame class property.
     */
    void apply_canny();

    /**
     * @brief Apply the lane custom filter the frame class property.
     */
    void apply_lane_filter(const int tau = cst::kLaneLineWidth);

private:
    cv::Mat frame_;

    const std::array<cv::Point, 8> points = {
        // outer trapezoid
        cv::Point(
            static_cast<int>((cst::kVideoWidth * (1.0 - cst::kTrapezoidBottomWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight),
        cv::Point(
            static_cast<int>((cst::kVideoWidth * (1.0 - cst::kTrapezoidTopWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight - static_cast<int>(cst::kVideoHeight * cst::kTrapezoidHeight)),
        cv::Point(
            static_cast<int>(cst::kVideoWidth - (cst::kVideoWidth * (1.0 - cst::kTrapezoidTopWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight - static_cast<int>(cst::kVideoHeight * cst::kTrapezoidHeight)),
        cv::Point(
            static_cast<int>(cst::kVideoWidth - (cst::kVideoWidth * (1.0 - cst::kTrapezoidBottomWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight),

        // inner trapezoid
        cv::Point(
            static_cast<int>((cst::kVideoWidth * (1.0 - cst::kTrapezoidBottomWidth * cst::kSmallBottomWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight),
        cv::Point(
            static_cast<int>((cst::kVideoWidth * (1.0 - cst::kTrapezoidTopWidth * cst::kSmallTopWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight - static_cast<int>(cst::kVideoHeight * cst::kTrapezoidHeight * cst::kSmallHeight)),
        cv::Point(
            static_cast<int>(cst::kVideoWidth - (cst::kVideoWidth * (1.0 - cst::kTrapezoidTopWidth * cst::kSmallTopWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight - static_cast<int>(cst::kVideoHeight * cst::kTrapezoidHeight * cst::kSmallHeight)),
        cv::Point(
            static_cast<int>(cst::kVideoWidth - (cst::kVideoWidth * (1.0 - cst::kTrapezoidBottomWidth * cst::kSmallBottomWidth)) / 2.0) + cst::kTrapezoidOffsetX,
            cst::kVideoHeight)};

    const int ammount_change_delta_ = 10000;

    int last_white_ammount_ = -1;

    const uint8_t min_threshold_ = 80;
    const uint8_t max_threshold_ = 140;
    uint8_t current_threshold_ = 130;
    
    cv::Mat debug_frame;
};
