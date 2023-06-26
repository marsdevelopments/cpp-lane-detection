#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "constants.hpp"
#include "LanePoints.hpp"

class RandomLaneDetection
{
public:
    RandomLaneDetection();

    LanePoints find_lines(const cv::Mat &original_frame, const cv::Mat &edited_frame);

private:
    cv::Mat edited_frame_;
    cv::Mat original_frame_;

    std::array<std::pair<int, int>, cst::lines_array_size> left_lines_;
    std::array<std::pair<int, int>, cst::lines_array_size> right_lines_;

    // trapezoid points
    const int bottom_left_x_ = cst::trapezoid_roi_points.at(0).x;
    const int top_left_x_ = cst::trapezoid_roi_points.at(1).x;
    const int top_right_x_ = cst::trapezoid_roi_points.at(2).x;
    const int bottom_right_x_ = cst::trapezoid_roi_points.at(3).x;
    const int y_top_ = cst::trapezoid_roi_points.at(1).y;
    const int y_bottom_ = cst::trapezoid_roi_points.at(0).y;

private:
    void populate_lines();

    // std::array<std::pair<int, int>, cst::lines_array_size> get_lines_in_range(const int top_min, const int top_max, const int bottom_min, const int bottom_max);

    std::pair<int, int> select_best_line(const std::array<std::pair<int, int>, cst::lines_array_size> &lines);

    int get_line_score(const int x1, const int y1, const int x2, const int y2);
};