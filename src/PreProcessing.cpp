#include "PreProcessing.hpp"

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

#if DEBUG_PROC
    debug_frame = frame_.clone();
#endif

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

void PreProcessing::apply_roi_trapezoid()
{
    cv::Mat mask = cv::Mat::zeros(frame_.size(), frame_.type());
    fillPoly(mask, cst::trapezoid_roi_points, cv::Scalar(255, 255, 255));

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
    // apply current threshold value, and extract ammount of resulting white pixels
    cv::Mat frame_copy = frame_.clone();
    cv::threshold(frame_copy, frame_copy, current_threshold_, 255, cv::ThresholdTypes::THRESH_BINARY);
    const int white_ammount = cv::countNonZero(frame_copy);

    // change the threshold if ammount of white pixels is not acceptable
    try_set_threshold(white_ammount);

    cv::threshold(frame_, frame_, current_threshold_, 255, 0);

#if DEBUG_PROC
    cv::putText(debug_frame, "White ammount: " + std::to_string(white_ammount), cv::Point(10, 70), cv::FONT_HERSHEY_PLAIN, 1.5, debug_color);
    cv::putText(debug_frame, "Threshold: " + std::to_string(current_threshold_), cv::Point(10, 120), cv::FONT_HERSHEY_PLAIN, 1.5, debug_color);
    cv::imshow("a", debug_frame);
    cv::waitKey(0);
#endif
}

void PreProcessing::try_set_threshold(const int &white_ammount)
{
    if (is_white_ammount_acceptable(white_ammount))
        return;

#if DEBUG_PROC
    std::string text;
#endif

    // if there are too much white pixels =>
    // threshold should be increased to decrease the ammount of white pixels
    if (white_ammount > max_white) 
    {
        // new threshold value is average between current value and maximum possible value
        current_threshold_ = (max_threshold_ + current_threshold_) / 2;

#if DEBUG_PROC
        text = "Incresing threshold...: ";
#endif
    }
    // if not, there are too little white pixels =>
    // threshold should be decreased to increase the ammount of white pixels
    else 
    {
        // new threshold value is average between current value and minimum possible value
        current_threshold_ = (min_threshold_ + current_threshold_) / 2;

#if DEBUG_PROC
        text = "Decreasing threshold...: ";
#endif
    }

#if DEBUG_PROC
    cv::putText(debug_frame, text, cv::Point(10, 170), cv::FONT_HERSHEY_PLAIN, 1.5, debug_color);
    cv::putText(debug_frame, "New threshold: " + std::to_string(current_threshold_), cv::Point(10, 220), cv::FONT_HERSHEY_PLAIN, 1.5, debug_color);

    // cv::imshow("a", debug_frame);
#endif
}

bool PreProcessing::is_white_ammount_acceptable(const int &ammount)
{
    return (ammount >= min_white && ammount <= max_white);
}


void PreProcessing::apply_canny()
{
    cv::Canny(frame_, frame_, cst::kCannyLowThreshold, cst::kCannyHighThreshold);
}