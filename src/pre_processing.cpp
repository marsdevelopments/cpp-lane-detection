#include "include/pre_processing.hpp"

#include "constants.hpp"

PreProcessing::PreProcessing() {}

cv::Mat PreProcessing::process_threshold(const cv::Mat &source)
{
    frame_ = source;

    apply_roi_trapezoid();
    apply_grayscale();
    apply_gaussian_blur();
    apply_thresholding();

    return frame_;
}

cv::Mat PreProcessing::process_adaptive_threshold(const cv::Mat &source)
{
    frame_ = source;
    debug_frame = frame_.clone();

    apply_roi_trapezoid();
    apply_grayscale();
    apply_gaussian_blur();
    apply_adaptive_thresholding();

    return frame_;
}

cv::Mat PreProcessing::process_canny(const cv::Mat &source)
{
    frame_ = source;

    apply_grayscale();
    apply_gaussian_blur();
    apply_canny();
    // apply_roi_rectangle();
    apply_roi_trapezoid();

    return frame_;
}

cv::Mat PreProcessing::process_custom(const cv::Mat &source)
{
    frame_ = source;

    apply_roi_trapezoid();
    apply_gaussian_blur();
    apply_grayscale();
    apply_range();
    // apply_lane_filter();
    // apply_thresholding();

    return frame_;
}

void PreProcessing::apply_roi_trapezoid()
{
    cv::Mat mask = cv::Mat::zeros(frame_.size(), frame_.type());
    fillPoly(mask, points, cv::Scalar(255, 255, 255));

    bitwise_and(frame_, mask, frame_);
}

void PreProcessing::apply_roi_rectangle()
{
    constexpr double width = cst::kVideoWidth;
    constexpr double height = cst::kVideoHeight;

    // const int x = (width * (1 - cst::kTrapezoidBottomWidth)) / 2 + cst::kTrapezoidOffsetX;
    // const int y = height - height * cst::kTrapezoidHeight;
    // const int rect_width = width * cst::kTrapezoidBottomWidth;
    // const int rect_height = height * cst::kTrapezoidHeight;
    constexpr int x = static_cast<int>(width * (1.0 - cst::kRectangleWidth));
    constexpr int y = static_cast<int>(height - height * cst::kRectangleHeight);
    constexpr int rect_width = static_cast<int>(width * cst::kRectangleWidth) - 1;
    constexpr int rect_height = static_cast<int>(height * cst::kRectangleHeight) - 1;

    cv::Mat mask = cv::Mat::zeros(frame_.size(), frame_.type());
    mask(cv::Rect(x, y, rect_width, rect_height)) = cv::Scalar(255, 255, 255);

    bitwise_and(frame_, mask, frame_);
}

void PreProcessing::apply_grayscale()
{
    cv::cvtColor(frame_, frame_, cv::COLOR_BGR2GRAY);
}

void PreProcessing::apply_gaussian_blur()
{
    cv::GaussianBlur(frame_, frame_, cv::Size(9, 9), 0.0);
}

void PreProcessing::apply_thresholding()
{
    cv::threshold(frame_, frame_, 130, 255, 0);
}

void PreProcessing::apply_adaptive_thresholding()
{
    cv::Mat frame_copy = frame_.clone();
    cv::threshold(frame_copy, frame_copy, current_threshold_, 255, cv::ThresholdTypes::THRESH_BINARY);
    int new_white_ammount = cv::countNonZero(frame_copy);

    int difference = new_white_ammount - last_white_ammount_;

    if (last_white_ammount_ == -1)
        difference = 0;

    last_white_ammount_ = new_white_ammount;

    cv::putText(debug_frame, "Difference: " + std::to_string(difference), cv::Point(10, 70), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));
    cv::putText(debug_frame, "Threshold: " + std::to_string(current_threshold_), cv::Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));

    set_new_threshold(difference);

    cv::threshold(frame_, frame_, current_threshold_, 255, 0);

    cv::imshow("a", debug_frame);
    cv::waitKey(0);
}

void PreProcessing::set_new_threshold(int difference)
{
    if (abs(difference) <= ammount_change_delta_)
        return;

    if (difference >= 0) // if difference is positive => # too much white pixels => threshold should be increased
    {
        current_threshold_ = (max_threshold_ + current_threshold_) / 2;
        // current_threshold_ = max_threshold_;
        cv::putText(debug_frame, "Incresing threshold...: ", cv::Point(10, 170), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));
    }
    else
    {
        current_threshold_ = (min_threshold_ + current_threshold_) / 2;
        // current_threshold_ = min_threshold_;
        cv::putText(debug_frame, "Decreasing threshold...: ", cv::Point(10, 170), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));
    }

    cv::putText(debug_frame, "New threshold: " + std::to_string(current_threshold_), cv::Point(10, 220), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));

    cv::imshow("a", debug_frame);
}

void PreProcessing::apply_range()
{
    cv::Mat range_mask;
    cv::inRange(frame_, 130, 255, range_mask);
    bitwise_and(frame_, range_mask, frame_);
}

void PreProcessing::apply_canny()
{
    cv::Canny(frame_, frame_, cst::kCannyLowThreshold, cst::kCannyHighThreshold);
}

void PreProcessing::apply_lane_filter(const int tau)
{
    cv::Mat result(cv::Size(cst::kVideoWidth, cst::kVideoHeight), CV_8UC1);

    // std::cout << "Frame type: " << frame_.type() << std::endl;

    for (int i = 0; i < frame_.rows; ++i)
    {
        for (int j = 0; j < frame_.cols; ++j)
        {
            if (j - tau < 0 ||
                j + tau >= frame_.cols)
            {
                result.at<uint8_t>(i, j) = 0;
                continue;
            }

            const uint8_t value = frame_.at<uint8_t>(i, j);

            if (value < 130)
                continue;

            // if (is_near_range(i, j, 3))
            //     result.at<uint8_t>(i, j) = 255;

            const uint8_t x_tau = frame_.at<uint8_t>(i, j - tau);
            const uint8_t tau_x = frame_.at<uint8_t>(i, j + tau);

            // const uint8_t result_value =

            const int resulted_value =
                2 * value - (x_tau + tau_x) - abs(x_tau - tau_x);

            // if (resulted_value > 255)
            //     std::cout << "resulted_value is: " << resulted_value << std::endl;

            result.at<uint8_t>(i, j) = resulted_value > 255 ? 255 : resulted_value;
        }
    }

    // cv::imshow("a", result);
    // cv::waitKey(0);

    frame_ = result;
}