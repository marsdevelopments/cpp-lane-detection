#include "LaneTracking.hpp"

#include <span>

#include "RandomLineBuilder.hpp"

#define DEBUG_LANE 1

LanePoints
LaneTracking::restart_tracking(const cv::Mat& edited_frame,
                               const cv::Mat& debug_frame)
{
  edited_frame_ = edited_frame;

  std::array<std::pair<int, int>, cst::lines_array_size> left_lines =
    RandomLineBuilder::build_lines<cst::lines_array_size>(
      top_left_x_, cst::x_center, bottom_left_x_, cst::x_center);

  std::array<std::pair<int, int>, cst::lines_array_size> right_lines =
    RandomLineBuilder::build_lines<cst::lines_array_size>(
      cst::x_center, top_right_x_, cst::x_center, bottom_right_x_);

#if DEBUG_LANE

  cv::Mat lanes_frame = debug_frame.clone();

  cv::putText(lanes_frame,
              "Restarting tracking",
              cv::Point{ 10, 30 },
              cv::FONT_HERSHEY_PLAIN,
              1.5,
              cv::Scalar(255, 255, 255));

  for (const std::pair<int, int> line : left_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::y_top),
             cv::Point(line.second, cst::y_bottom),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  for (const std::pair<int, int> line : right_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::y_top),
             cv::Point(line.second, cst::y_bottom),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  cv::imshow("Built Lines", lanes_frame);

#endif

  const std::pair<int, int> left_line =
    RandomLineBuilder::select_best_line(edited_frame_, left_lines);
  const std::pair<int, int> right_line =
    RandomLineBuilder::select_best_line(edited_frame_, right_lines);

  const bool left_line_found = left_line.first != -1 || left_line.second != -1;
  const bool right_line_found =
    right_line.first != -1 || right_line.second != -1;

  if (left_line_found)
    add_left_average(left_line.first, left_line.second);

  if (right_line_found)
    add_right_average(right_line.first, right_line.second);

  last_lane_points = get_from_average(left_line_found, right_line_found);

  return last_lane_points;
}

LanePoints
LaneTracking::retrack(const cv::Mat& edited_frame, const cv::Mat& debug_frame)
{
  edited_frame_ = edited_frame;

  std::array<std::pair<int, int>, cst::lines_array_size / 2> left_lines =
    RandomLineBuilder::build_lines<cst::lines_array_size / 2>(
      last_lane_points.left_top - cst::retrack_x_delta,
      last_lane_points.left_top + cst::retrack_x_delta,
      last_lane_points.left_bottom - cst::retrack_x_delta,
      last_lane_points.left_bottom + cst::retrack_x_delta);

  std::array<std::pair<int, int>, cst::lines_array_size / 2> right_lines =
    RandomLineBuilder::build_lines<cst::lines_array_size / 2>(
      last_lane_points.right_top - cst::retrack_x_delta,
      last_lane_points.right_top + cst::retrack_x_delta,
      last_lane_points.right_bottom - cst::retrack_x_delta,
      last_lane_points.right_bottom + cst::retrack_x_delta);

#if DEBUG_LANE

  cv::Mat lanes_frame = debug_frame.clone();

  cv::putText(lanes_frame,
              "Retracking",
              cv::Point{ 10, 30 },
              cv::FONT_HERSHEY_PLAIN,
              1.5,
              cv::Scalar(255, 255, 255));

  for (const std::pair<int, int> line : left_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::y_top),
             cv::Point(line.second, cst::y_bottom),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  for (const std::pair<int, int> line : right_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::y_top),
             cv::Point(line.second, cst::y_bottom),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  cv::imshow("Built Lines", lanes_frame);

#endif

  const std::pair<int, int> left_line =
    RandomLineBuilder::select_best_line(edited_frame_, left_lines);
  const std::pair<int, int> right_line =
    RandomLineBuilder::select_best_line(edited_frame_, right_lines);

  const bool left_line_found = left_line.first != -1 || left_line.second != -1;
  const bool right_line_found =
    right_line.first != -1 || right_line.second != -1;

  if (left_line_found)
    add_left_average(left_line.first, left_line.second);

  if (right_line_found)
    add_right_average(right_line.first, right_line.second);

  last_lane_points = get_from_average(left_line_found, right_line_found);

  return last_lane_points;
}

void
LaneTracking::clear_averages()
{
  for (size_t i = 0; i < average_size; ++i) {
    left_average[i].first = 0;
    left_average[i].second = 0;

    right_average[i].first = 0;
    right_average[i].second = 0;
  }
}

void
LaneTracking::add_average(std::array<std::pair<int, int>, average_size>& source,
                          const int& x1,
                          const int& x2)
{
  for (size_t i = source.size() - 1, end = 1; i >= end; --i) {
    source[i] = source[i - 1];
  }

  source[0] = { x1, x2 };
}

void
LaneTracking::add_left_average(const int& x1, const int& x2)
{
  add_average(left_average, x1, x2);
}

void
LaneTracking::add_right_average(const int& x1, const int& x2)
{
  add_average(right_average, x1, x2);
}

std::pair<int, int>
LaneTracking::get_average(
  const std::array<std::pair<int, int>, average_size>& source)
{
  std::pair<int, int> average;

  int divider = 0;

  for (auto cord : source) {
    if (cord.first == 0)
      continue;

    average.first += cord.first;
    average.second += cord.second;
    ++divider;
  }

  if (divider == 0)
    return std::pair<int, int>(0, 0);

  average.first /= divider;
  average.second /= divider;

  return average;
}

std::pair<int, int>
LaneTracking::get_left_average()
{
  return get_average(left_average);
}

std::pair<int, int>
LaneTracking::get_right_average()
{
  return get_average(right_average);
}

LanePoints
LaneTracking::get_from_average(bool left_line_found, bool right_line_found)
{
  const LanePoints default_points;

  const std::pair<int, int> left = get_left_average();
  const std::pair<int, int> right = get_right_average();

  if (right_line_found && left_line_found) {
    missing_frames_counter = 0;

    if (long_missing_flag) {
      long_missing_flag = false;
      return default_points;
    }
  } else if (!right_line_found && !left_line_found) {
    ++missing_frames_counter;
  }

  if (missing_frames_counter >= average_size) {
    long_missing_flag = true;
    clear_averages();
    return default_points;
  }

  return LanePoints{ left.first,   left.second,     right.first,
                     right.second, left_line_found, right_line_found };
}