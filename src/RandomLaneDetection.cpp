#include "RandomLaneDetection.hpp"

#include <vector>
#include <numeric>
#include <random>

#include "constants.hpp"
#include "RandomLineBuilder.hpp"

#define DEBUG_LINE_DET 1

RandomLaneDetection::RandomLaneDetection() {}

LanePoints RandomLaneDetection::find_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame)
{
    original_frame_ = original_frame;
    edited_frame_ = edited_frame;

    populate_lines();

    const std::pair<int, int> left_line = RandomLineBuilder::select_best_line(edited_frame_, left_lines_);
    const std::pair<int, int> right_line = RandomLineBuilder::select_best_line(edited_frame_, right_lines_);

    return LanePoints{left_line.first, left_line.second, right_line.first, right_line.second};
}

void RandomLaneDetection::populate_lines()
{
    const int x_center = cst::kVideoWidth / 2;

    cv::imshow("Built Lines", edited_frame_);

    left_lines_ = RandomLineBuilder::build_lines<cst::lines_array_size>(top_left_x_, x_center, bottom_left_x_, x_center);
    right_lines_ = RandomLineBuilder::build_lines<cst::lines_array_size>(x_center, top_right_x_, x_center, bottom_right_x_);

#if DEBUG_LINE_DET

    cv::Mat lanes_frame = original_frame_.clone();

    for (const std::pair<int, int> line : left_lines_)
    {
        cv::line(lanes_frame, cv::Point(line.first, y_top_), cv::Point(line.second, y_bottom_), cv::Scalar(0, 0, 255), 2 /* , cv::LINE_AA */);
    }
    for (const std::pair<int, int> line : right_lines_)
    {
        cv::line(lanes_frame, cv::Point(line.first, y_top_), cv::Point(line.second, y_bottom_), cv::Scalar(0, 0, 255), 2 /* , cv::LINE_AA */);
    }
    cv::imshow("Built Lines", lanes_frame);

#endif
}