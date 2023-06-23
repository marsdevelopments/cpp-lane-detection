#pragma once

#include <opencv2/opencv.hpp>

#include "constants.hpp"

#define DEBUG_PROC 1

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
    bool is_white_ammount_acceptable(const int &ammount);

    /**
     * @brief Apply the thresholding filter the frame class property.
     */
    void set_threshold(const int &white_ammount);

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

    // const int ammount_change_delta_ = 2000;

    const int min_white = 650;
    const int max_white = 3500;

    // int last_white_ammount_ = -1;

    const uint8_t min_threshold_ = 60;
    const uint8_t max_threshold_ = 160;
    uint8_t current_threshold_ = 140;

#if DEBUG_PROC
    cv::Mat debug_frame;
    const cv::Scalar debug_color{255, 255, 255}; // white
#endif
};
