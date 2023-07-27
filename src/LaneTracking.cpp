#include "LaneTracking.hpp"

#include <span>

#include "RandomLineBuilder.hpp"

#define DEBUG_LANE 1

LanePoints
LaneTracking::restart_tracking(const cv::Mat& edited_frame,
                               const cv::Mat& debug_frame)
{
  edited_frame_ = edited_frame;

  // build random lines across the left half of the ROI
  std::array<std::pair<int, int>, cst::kLinesArraySize> left_lines =
    RandomLineBuilder::build_lines<cst::kLinesArraySize>(
      top_left_x_, cst::kCenterX, bottom_left_x_, cst::kCenterX);

  // build random lines across the right half of the ROI
  std::array<std::pair<int, int>, cst::kLinesArraySize> right_lines =
    RandomLineBuilder::build_lines<cst::kLinesArraySize>(
      cst::kCenterX, top_right_x_, cst::kCenterX, bottom_right_x_);

#if DEBUG_LANE

  cv::Mat lanes_frame = debug_frame.clone();

  cv::putText(lanes_frame,
              "Restarting tracking",
              cv::Point{ 10, 30 },
              cv::FONT_HERSHEY_PLAIN,
              1.5,
              cv::Scalar(255, 255, 255));

  // draw lines
  for (const std::pair<int, int> line : left_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::kTopY),
             cv::Point(line.second, cst::kBottomY),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  for (const std::pair<int, int> line : right_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::kTopY),
             cv::Point(line.second, cst::kBottomY),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  cv::imshow("Built Lines", lanes_frame);

#endif

  left_line_ = RandomLineBuilder::select_best_line(edited_frame_, left_lines);
  right_line_ = RandomLineBuilder::select_best_line(edited_frame_, right_lines);

  process_lines();

  return last_lane_points;
}

LanePoints
LaneTracking::retrack(const cv::Mat& edited_frame, const cv::Mat& debug_frame)
{
  edited_frame_ = edited_frame;

  // build random lines across the left half of the ROI
  // but only near previously found line
  std::array<std::pair<int, int>, cst::kLinesArraySize / 2> left_lines =
    RandomLineBuilder::build_lines<cst::kLinesArraySize / 2>(
      last_lane_points.left_top - cst::kRetrackDeltaX,
      last_lane_points.left_top + cst::kRetrackDeltaX,
      last_lane_points.left_bottom - cst::kRetrackDeltaX,
      last_lane_points.left_bottom + cst::kRetrackDeltaX);

  // build random lines across the right half of the ROI
  // but only near previously found line
  std::array<std::pair<int, int>, cst::kLinesArraySize / 2> right_lines =
    RandomLineBuilder::build_lines<cst::kLinesArraySize / 2>(
      last_lane_points.right_top - cst::kRetrackDeltaX,
      last_lane_points.right_top + cst::kRetrackDeltaX,
      last_lane_points.right_bottom - cst::kRetrackDeltaX,
      last_lane_points.right_bottom + cst::kRetrackDeltaX);

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
             cv::Point(line.first, cst::kTopY),
             cv::Point(line.second, cst::kBottomY),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  for (const std::pair<int, int> line : right_lines) {
    cv::line(lanes_frame,
             cv::Point(line.first, cst::kTopY),
             cv::Point(line.second, cst::kBottomY),
             cv::Scalar(0, 0, 255),
             2 /* , cv::LINE_AA */);
  }
  cv::imshow("Built Lines", lanes_frame);

#endif

  left_line_ = RandomLineBuilder::select_best_line(edited_frame_, left_lines);
  right_line_ = RandomLineBuilder::select_best_line(edited_frame_, right_lines);

  process_lines();

  return last_lane_points;
}

void
LaneTracking::process_lines()
{
  const bool left_line_found =
    left_line_.first != -1 || left_line_.second != -1;
  const bool right_line_found =
    right_line_.first != -1 || right_line_.second != -1;

  if (left_line_found)
    add_left_average(left_line_.first, left_line_.second);

  if (right_line_found)
    add_right_average(right_line_.first, right_line_.second);

  last_lane_points = get_from_average(left_line_found, right_line_found);
}

void
LaneTracking::clear_averages()
{
  for (size_t i = 0; i < average_size_; ++i) {
    left_average_[i].first = 0;
    left_average_[i].second = 0;

    right_average_[i].first = 0;
    right_average_[i].second = 0;
  }
}

void
LaneTracking::add_average(
  std::array<std::pair<int, int>, average_size_>& source,
  const int& x1,
  const int& x2)
{
  // shift all cells to the left
  for (size_t i = source.size() - 1, end = 1; i >= end; --i) {
    source[i] = source[i - 1];
  }

  // and add new value at the beggining
  source[0] = { x1, x2 };
}

void
LaneTracking::add_left_average(const int& x1, const int& x2)
{
  add_average(left_average_, x1, x2);
}

void
LaneTracking::add_right_average(const int& x1, const int& x2)
{
  add_average(right_average_, x1, x2);
}

std::pair<int, int>
LaneTracking::get_average(
  const std::array<std::pair<int, int>, average_size_>& source)
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
  return get_average(left_average_);
}

std::pair<int, int>
LaneTracking::get_right_average()
{
  return get_average(right_average_);
}

LanePoints
LaneTracking::get_from_average(bool left_line_found, bool right_line_found)
{
  const LanePoints default_points;

  // left line, built from the average of previous points
  const std::pair<int, int> left = get_left_average();
  // right line, built from the average of previous points
  const std::pair<int, int> right = get_right_average();

  // if both lines are found
  if (right_line_found && left_line_found) {
    // reset missing frames counter
    missing_frames_counter_ = 0;

    // if long missing flag is set
    if (long_missing_flag_) {
      // reset it
      long_missing_flag_ = false;
      // and return invalid lane points
      // return default_points;
    }
    // if both were not found
  } else if (!right_line_found && !left_line_found) {
    // increment counter
    ++missing_frames_counter_;
  }

  // if the ammount of missing frames exceeds average filter array size
  if (missing_frames_counter_ >= average_size_) {
    // set missing flag
    long_missing_flag_ = true;
    // clear averages
    clear_averages();
    // and return invalid points to restart tracking
    return default_points;
  }

  // return LanePoints if at least one lane line was found
  return LanePoints{ left.first,   left.second,     right.first,
                     right.second, left_line_found, right_line_found };
}