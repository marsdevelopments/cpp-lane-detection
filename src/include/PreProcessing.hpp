#pragma once

#include <opencv2/opencv.hpp>

#include "constants.hpp"

#define DEBUG_PROC 0

class PreProcessing
{
public:
    PreProcessing();

    /**
     * @brief Proccess a frame using threshold method.
     *
     * @param source frame that needs to be processed
     * @return Processed frame
     */
    cv::Mat process_threshold(const cv::Mat &source);

    /**
     * @brief Proccess a frame using adaptive threshold method
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_adaptive_threshold(const cv::Mat &source);

    /**
     * @brief Proccess a frame using canny edge detection method
     *
     * @param source frame that needs to be processed
     * @return processed frame
     */
    cv::Mat process_canny(const cv::Mat &source);

private:

#pragma region General functions

    /**
     * @brief Extract Region Of Interest in the trapezoid shape from the stored frame
     */
    void apply_roi_trapezoid();

    /**
     * @brief Apply the grayscale filter to the stored frame
     */
    void apply_grayscale();

    /**
     * @brief Apply the gaussian blur filter to the stored frame
     */
    void apply_gaussian_blur();

    /**
     * @brief Apply the thresholding filter to the stored frame
     */
    void apply_thresholding();

    /**
     * @brief Apply the canny filter to the stored frame
     */
    void apply_canny();

#pragma endregion


#pragma region Adaptive thresholding functions

    /**
     * @brief Apply the adaptive thresholding filter to the stored frame
     */
    void apply_adaptive_thresholding();

    /**
     * @brief Check if ammount of white pixels falls within acceptable range
     */
    bool is_white_ammount_acceptable(const int &ammount);

    /**
     * @brief Set the threshold which will be used to threshold the stored frame
     */
    void set_threshold(const int &white_ammount);

#pragma endregion

private:
    // frame to be processed
    cv::Mat frame_;


#pragma region Adaptive threshold settings

    // Range of acceptable ammount of white pixels
    const int min_white = 650;
    const int max_white = 3500;

    // Range of possible thresholds
    const uint8_t min_threshold_ = 60;
    const uint8_t max_threshold_ = 160;
    uint8_t current_threshold_ = 140;

#pragma endregion

#if DEBUG_PROC
    cv::Mat debug_frame;
    const cv::Scalar debug_color{255, 255, 255}; // white
#endif
};
