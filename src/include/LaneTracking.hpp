#pragma once

#include <array>

#include <opencv2/opencv.hpp>

#include "LanePoints.hpp"
#include "constants.hpp"

class LaneTracking
{
public:
  LanePoints restart_tracking(const cv::Mat& edited_frame, const cv::Mat& debug_frame);

  LanePoints retrack(const cv::Mat& edited_frame, const cv::Mat& debug_frame);

private:
  static const size_t average_size = 5;
//   static const int x_center = cst::kVideoWidth / 2;

  std::pair<int, int> left_line;
  std::pair<int, int> right_line;

  // std::array<std::pair<int, int>, cst::lines_array_size> left_lines_;
  // std::array<std::pair<int, int>, cst::lines_array_size> right_lines_;

  // std::array<std::pair<int, int>, cst::lines_array_size / 2>
  // retrack_left_lines_; std::array<std::pair<int, int>, cst::lines_array_size
  // / 2> retrack_right_lines_;

  const int bottom_left_x_ = cst::trapezoid_roi_points.at(0).x;
  const int top_left_x_ = cst::trapezoid_roi_points.at(1).x;
  const int top_right_x_ = cst::trapezoid_roi_points.at(2).x;
  const int bottom_right_x_ = cst::trapezoid_roi_points.at(3).x;

  cv::Mat edited_frame_;

  bool long_missing_flag = false;
  size_t missing_frames_counter = 0;

  std::array<std::pair<int, int>, average_size> left_average = {
    { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }
  };

  std::array<std::pair<int, int>, average_size> right_average = {
    { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } }
  };

  LanePoints last_lane_points;

private:
  void clear_averages();

  void add_average(std::array<std::pair<int, int>, average_size>& source,
                   const int& x1,
                   const int& x2);

  void add_left_average(const int& x1, const int& x2);

  void add_right_average(const int& x1, const int& x2);

  std::pair<int, int> get_average(
    const std::array<std::pair<int, int>, average_size>& source);

  std::pair<int, int> get_left_average();

  std::pair<int, int> get_right_average();

  LanePoints get_from_average(bool drawLeftLane, bool drawRightLane);

  // template <size_t S>
  // void track_lines(const bool retrack)
  // {
  //     const std::array<std::pair<int, int>, S> left_lines_ =
  //     RandomLineBuilder::build_lines<S>(top_left_x_, x_center,
  //     bottom_left_x_, x_center); const std::array<std::pair<int, int>, S>
  //     right_lines_;

  //     left_lines_;
  //     right_lines_ =
  //     RandomLineBuilder::build_lines<cst::lines_array_size>(x_center,
  //     top_right_x_, x_center, bottom_right_x_);

  //     const std::pair<int, int> left_line =
  //     RandomLineBuilder::select_best_line(edited_frame_, left_lines_); const
  //     std::pair<int, int> right_line =
  //     RandomLineBuilder::select_best_line(edited_frame_, right_lines_);
  // }

  // void draw_points(const std::pair<int, int> &left, const std::pair<int, int>
  // &right);
};